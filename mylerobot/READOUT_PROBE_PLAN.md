# Readout hypothesis + object-grounding probe (pi0 + LIBERO)

Status: 2026-05-28. Diagnostic in flight; this doc captures the reasoning chain,
the testbed decision, and the probe design. Results appended at the bottom.

## The chain of reasoning

### 1. Two prior negative results, one suspected root cause
- **pi0_voxel on LIBERO-Plus** (H100): extrinsics-corruption ablation showed
  identity extrinsics ≈ correct extrinsics → the voxel-cross-attention is
  functionally inert as 3D grounding. ABANDONED (see PROJECT_STATUS rows 2f–2h.3,
  `pi0_voxel/extrinsics_libero.py`).
- **act_bev on IsaacSim** (RTX5090): a cube-heatmap aux head made the BEV encoder
  metrically grounded (1.18 cm localization), yet closed-loop was 0/20. The action
  transformer ignored the BEV tokens in favor of proprio.

Both looked like *causal confusion / copycat*: "the action head won't consume
geometric perception when a proprioceptive shortcut exists."

### 2. The causal-shortcut toy re-diagnosed it: READOUT, not consumption
`causal_shortcut_study/` (torch+numpy, ~8 s/run) isolated the variable. Headline
(see its README, commits 30aae5d / dad98a4 / b9d26e1):

- A **mean-pool** readout (concat/xattn/replace) destroys spatial location and
  caps target-decode at ~0.29 L2 (≫ the 0.05 success radius), so aux supervision
  can't help (closed-loop ~0.16). This reproduced the act_bev pattern.
- A **location-preserving soft-argmax** readout + the SAME aux loss drives decode
  to 0.009 L2 and closed-loop to **0.98**; two-stage decouple (freeze
  aux-pretrained encoder, train only the action head) → **0.99**. This validates
  the "decoupled representation + policy" hypothesis once the readout is right.
- **Clutter test** (`--n_distractors`): soft-argmax degrades *gracefully*
  (0.98 → 0.83 at 4 distractors); mean-pool *collapses* (0.07 at 2). The conv
  front-end learns to select the target peak.
- Dual-system routing (xattn) does NOT beat concat — refutes the Road B-4 prior.

**Re-diagnosis:** the dominant failure was most likely a **lossy readout**
(mean-pool / weak token aggregation discarding spatial location), NOT an intrinsic
inability of the action head to consume geometry.

### 3. Why NOT validate on act_bev — the data is a confound
The act_bev / IsaacSim demos were collected via **IK (inverse kinematics)
computation**, not real teleop. They are information-limited (no contact/retry/
correction structure) and trained poorly. So act_bev's failure has a SECOND,
independent cause (data). Testing a readout fix there is uninterpretable: a
post-fix failure could be the readout OR the data. **A data-confounded testbed
yields no clean causal conclusion.**

### 4. Why pi0 + LIBERO is the chosen testbed
- Clean teleop data; working pipeline (libero_spatial ~60%); established
  LIBERO-Plus fragility baselines; it IS the publication track (sensor
  robustness). Minimal confounds → clean attribution.
- **Do NOT re-run voxel-cross-attention** (abandoned, proven inert). Translate the
  toy as a location-preserving readout (soft-argmax / keypoint head) on SigLIP
  features + GT-object-pose aux, feeding an EXPLICIT decoded location into the
  action expert.

### 5. Honest caveat on expected magnitude
Base pi0 already works at 60% and its vision is attention over ViT patches (not
mean-pooled), so it is *less* readout-starved than the toy. Expect a smaller
effect; the real bet is **LIBERO-Plus camera-viewpoint robustness**, not overall
success. The toy proves the mechanism, not that a specific real module will work.

## The de-risking step: object-grounding probe (NO retraining)

Before committing 1–2 weeks to a readout-fix build (which needs object-pose GT in
the training data — see below), run the cheap real-system analog of the toy's
`perception_probe` on the EXISTING trained pi0. Script:
`scripts/probe_object_grounding.py`.

It captures, over policy-driven rollouts, the agentview SigLIP tokens + the
ground-truth position of the task's object-of-interest (read from the live
MuJoCo sim, no extra render), then fits three ridge probes (episode-grouped
70/15/15 split, λ chosen on val):

| probe | input | question |
|---|---|---|
| **pooled** | mean over SigLIP tokens | what a pooled readout sees |
| **spatial** | tokens kept per-position (fixed per-token random proj) | location preserved |
| **proprio** | robot EEF position | is object pos trivially in proprio? (should NOT be) |

**Reading the result:**
- `pooled ≪ chance` → object geometry is present even after pooling → the issue is
  *consumption*, not readout (the toy's mechanism would NOT transfer).
- `pooled ≈ chance` & `spatial ≪ pooled` → location IS in the features but pooling
  destroys it → the toy's readout diagnosis HOLDS on the real system → the
  readout-fix build is well-motivated.
- both `≈ chance` → pi0's encoder doesn't linearly encode object geometry at all →
  a readout fix on top won't help; need better features first.

## Object-pose GT availability (gates the eventual build)
- At **train time**: NOT available. The raw LIBERO HDF5 demos store only robot
  proprio (`create_dataset.py` skips object obs though the env can produce
  `obj_pos`/`obj_quat`); the converted LeRobotDataset keeps only the 8-D robot
  state.
- At **eval/sim time**: fully available — `env._env.env.sim` exposes every
  object's world pose, and `obj_of_interest` names the task-relevant objects.
- **To enable aux training cheaply:** the HDF5 stores the MuJoCo flattened
  `states[T]`, so object poses can be extracted by an OFFLINE replay
  (`set_state` → read body xpos), NO re-render / re-collection. (Hypothesis;
  verify on one file before scaling.)

## Decision tree / next steps
1. **[in flight]** Object-grounding probe on pi0 v044 (this doc's diagnostic).
2. If `spatial ≪ pooled` (readout diagnosis holds) → offline-extract object-pose
   GT → build the readout fix (soft-argmax/keypoint head + GT-pose aux + decoded
   coord into the state token) → retrain → eval LIBERO-Plus camera dim.
3. If `pooled ≪ chance` (geometry already present) → the lever is *consumption*
   (anti-copycat regularizer / shortcut-breaking), not readout.
4. If both `≈ chance` → the vision encoder is the bottleneck; readout fix premature.

## Pointers
- Toy: `../causal_shortcut_study/` (README has the full results + caveats).
- Probe: `scripts/probe_object_grounding.py`
  (`--libero_plus 1` required — the installed benchmark is the LIBERO-Plus
  variant whose task files carry perturbation suffixes).
- Fragility harness reused for rollout plumbing: `scripts/eval_libero_plus_fragility.py`.

## Results (2026-05-28, libero_object, N=1000 over 100 distinct scenes, 12 min)

| probe | test RMSE (m) | vs chance | feat dim |
|---|---|---|---|
| chance (predict mean) | 0.1199 | 1.0× | — |
| proprio (EEF pos) | 0.0661 | 1.8× | 3 |
| pooled (mean over tokens) | 0.0469 | 2.6× | 2048 |
| **spatial (location-preserving)** | **0.0169** | **7.1×** | 8192 |

object position spread ‖std‖ = 0.113 m. (`outputs/eval/probe_object_grounding.json`)

**Verdict — the geometry IS present, and the readout matters a lot.**
- pi0's SigLIP features linearly encode object position. Even mean-pooled they
  decode it to **4.7 cm** (vs 12 cm chance, 6.6 cm from EEF alone) → the encoder
  is NOT the bottleneck.
- A **location-preserving** readout extracts the SAME features **~2.8× more
  precisely (1.7 cm vs 4.7 cm)**. Mean-pooling discards most of the spatial
  precision — exactly the toy's mechanism, confirmed on the real system.
- spatial (1.7 cm) ≪ proprio (6.6 cm) → the visual features carry object geometry
  well beyond what the robot's EEF state trivially implies (not just "decode the
  arm position").

**Honest nuance — what this does NOT yet prove.**
- This is the *informative* but not the *extreme* case: pooled still beats chance
  (4.7 cm), so pooling doesn't fully destroy location. And pi0 does NOT use a
  naive global mean-pool — it feeds all 256 SigLIP tokens to the LLM via
  attention, which CAN in principle reach per-token spatial info. So "pooled
  4.7 cm" is a worst-case readout, not literally pi0's.
- The probe proves the spatial signal is PRESENT and readout-sensitive; it does
  NOT prove the action expert actually CONSUMES it.
- Confound noted: along rollouts the EEF approaches the object, so EEF↔object
  correlate (hence proprio 6.6 cm). spatial ≪ proprio shows the visual decode is
  genuine, not just arm-reading.

**Next gate — the "perception-use" half (toy's closed_loop − ablated).** Perturb
/ remove the object's visual evidence at eval and measure the success drop:
- barely drops → policy ignores the (present, decodable) geometry → a
  location-preserving readout the head is FORCED to consume is well-motivated;
- drops a lot → the head already uses it → headroom is small.
Reuse `eval_libero_plus_fragility.py --drop_cam image` (agentview) for a coarse
version; an object-region image mask would be the clean version.

**Bottom line:** first gate PASSED (geometry present + strongly readout-sensitive,
2.8×). The use/ablation test decides whether the readout-fix build has headroom
before committing to object-pose GT extraction + retraining.

## Ablation (perception-use) experiment — running 2026-05-28

The real-system analog of the toy's `perception_use = closed_loop − ablated`.
Same trained pi0 v044, no retraining. Run the fragility harness on libero_object
in three conditions on the SAME deterministically-sampled tasks (so the gap is
fair), via `eval_libero_plus_fragility.py --drop_cam {none,image,image2}`:

| condition | what it removes | tests |
|---|---|---|
| `none` | nothing | baseline closed-loop success |
| `image` | **agentview** cam zeroed every step | reliance on the SCENE camera (the only view that sees the object) |
| `image2` | **wrist** cam zeroed every step | control — does losing a camera per se break it? |

Categories: `Robot Initial States` (closest to clean) + `Camera Viewpoints`
(headline), 10 tasks/category, max_steps 280.

**Reading:**
- `none ≈ image` (dropping agentview barely hurts) → the policy is NOT using the
  scene camera → it ignores the present, decodable geometry → a location-preserving
  readout the action head is FORCED to consume is well-motivated. Big headroom.
- `none ≫ image` (dropping agentview craters success) → the policy DOES rely on the
  scene camera → it already consumes visual scene info; the readout-fix headroom is
  the *precision* gap (probe: 4.7→1.7 cm), smaller.
- `image2` is the control: if dropping the wrist also craters, the test is
  confounded by general OOD-sensitivity rather than object-geometry use.

Caveat: `--drop_cam image` is blunt (removes object AND table/basket/layout
context, not just the object). A per-object image mask would be the clean version;
this coarse cut is the cheap first read.

### Ablation results (2026-05-28, libero_object, n=8/cell — PRELIMINARY)

| condition | Robot Init States | Camera Viewpoints |
|---|---|---|
| none (baseline) | 12.5% (1/8) | **75.0%** (6/8) |
| drop **agentview** (scene cam) | 12.5% (1/8) | **62.5%** (5/8) |
| drop **wrist** | 12.5% (1/8) | **25.0%** (2/8) |

**The story flipped — the policy is WRIST-centric, not scene-centric.**
- On Camera Viewpoints, dropping the **agentview** (the only camera that sees the
  object layout from afar, and where the probe found object geometry at 1.7 cm)
  barely hurt: 75 → 62.5% (one episode of 8). Dropping the **wrist** cam hurt ~4×
  more: 75 → 25% (four episodes).
- So the policy leans heavily on the **wrist** close-up and **under-uses the scene
  camera** — exactly the view that carries the decodable global object geometry.
- This also plausibly explains the high (75%) Camera-Viewpoints baseline: under a
  viewpoint perturbation the agentview is perturbed but the **wrist cam moves with
  the arm (unperturbed)**, so a wrist-centric policy is naturally camera-robust —
  by *ignoring* the perturbed global view, not by being geometrically grounded.
- Robot Initial States sat at the 12.5% floor for all three → uninformative cell
  (the sampled tasks were hard); ignore it.

**HARD CAVEAT — n=8.** Every cell is 8 episodes; the agentview gap is literally one
episode (within Wilson CI ±30pp). The *direction* (wrist ≫ scene reliance) is
suggestive and the wrist gap (−50pp, 4 episodes) is larger than noise, but the
magnitudes are not trustworthy. A larger confirmation (Camera Viewpoints only,
~20–30 tasks, 3 conditions) is needed before acting.

**Refined hypothesis for the readout fix.** The scene-camera object geometry is
present (probe) but under-consumed (this ablation). A readout that surfaces
scene-camera object geometry explicitly AND that the action head is forced to
consume is the candidate lever for *novel-object-position* / harder camera
generalization — distinct from the wrist-centric near-field strategy the policy
currently defaults to. Confirm the ablation at larger n first.
