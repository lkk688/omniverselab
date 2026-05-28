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

## Results
_Pending — appended when `probe_object_grounding.py` (120 tasks, ~1000 samples,
libero_object) finishes._
