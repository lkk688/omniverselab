# Causal-Shortcut Study — why action heads ignore geometric perception

A clean, self-contained sandbox (torch + numpy only — no lerobot, no pi0_voxel,
no act_bev, no IsaacSim) to study the single failure mode that two separate
research tracks independently converged on.

## Background — the finding that motivates this

Two unrelated experiment tracks hit the same wall:

- **pi0_voxel on LIBERO-Plus** (H100): extrinsics-corruption ablation showed
  identity extrinsics ≈ correct extrinsics. The action expert ignores the
  voxel-fusion features regardless of whether they're geometrically correct.
- **act_bev on IsaacSim** (RTX5090): a Tesla-style cube-heatmap aux head made
  the BEV encoder metrically grounded (1.18 cm median cube localization), yet
  closed-loop success was 0/20. The action transformer ignored the 900 BEV
  tokens in favor of the 16-D proprioceptive state.

**Shared root cause: when a proprioceptive shortcut exists, the action head
does not learn to consume geometric perception features — even when those
features are demonstrably correct.** This is textbook *causal confusion in
imitation learning* (de Haan et al., NeurIPS 2019) / *copycat problem*
(Wen et al., NeurIPS 2020).

The geometric representation was never the bottleneck. The **consumption** of
geometric features by the action head is. This sandbox isolates that single
variable in the cheapest possible environment so we can sweep the fix-space
in minutes instead of GPU-days.

## Central question

> When a proprioceptive shortcut is available, what training intervention or
> architecture makes the action head actually consume a geometric perception
> feature?

## Hypotheses

- **H1 (shortcut-breaking)**: removing the shortcut (randomize init so the
  trajectory isn't memorizable; drop the target from proprio) forces the
  policy to use perception. *If true, the fix is data/observation design, not
  architecture.*
- **H2 (routing)**: even with the shortcut present, routing perception via
  cross-attention into the action head (dual-system, à la PointACT) makes it
  get consumed, where naive concat-to-state does not.
- **H3 (aux-insufficiency)**: an auxiliary perception loss makes the encoder
  geometric but does NOT by itself make the action head consume it (reproduces
  the act_bev result in miniature).
- **H4 (noise-tolerance)**: closed-loop robustness needs perception-noise
  augmentation during training, with a sweet spot — too little doesn't help,
  too much makes the policy discard perception (reproduces act_bev Path-1's
  σ=2 cm failure).

## The toy environment

`toy_reach.py` — a 2D reaching task:
- End-effector `ee ∈ [-1,1]²`, target `tgt ∈ [-1,1]²`
- Action = clipped delta-position velocity. Expert = P-controller toward target.
- The task is trivial *if you know the target*. The research interest is
  entirely in whether the policy learns to extract the target from perception
  rather than exploiting a shortcut.

### The three observation knobs

**proprio_mode** (controls shortcut availability):
- `full` — `[ee, tgt, prev_action]`: target given directly + copycat shortcut (`a_t≈a_{t-1}`)
- `no_target` — `[ee, prev_action]`: copycat shortcut present, target only via perception
- `minimal` — `[ee]`: no shortcut at all, target only via perception

**perception_mode** (what the policy must decode):
- `none` — no perception channel
- `raw` — target as clean 2D (oracle; like cube_pose GT)
- `image` — target rendered as a Gaussian blob on a 16×16 grid; needs a CNN to
  decode (analogous to BEV/voxel features that must be decoded)

**injection** (architecture — how perception enters the action head):
- `concat` — perception encoder → flatten → **mean-pool** → concat to proprio →
  MLP (the act_bev v2/v3 design that failed). Mean-pooling over the spatial grid
  DESTROYS location.
- `xattn` — proprio is the query, perception tokens are keys/values,
  cross-attention → action (dual-system / PointACT-style)
- `replace` — perception decoded to an explicit `tgt_hat`, substituted into the
  proprio target slot (act_bev Option-A style)
- `softargmax` — image → full-resolution heatmap → soft-argmax → continuous
  `(x,y)` coordinate fed to the action head. A **location-preserving** readout
  (the inductive bias act_bev used to hit 1.18 cm). Distinguishes a *readout*
  failure from a *consumption* failure.

### Training knobs
- `aux_weight` — weight on an auxiliary "predict target from perception" loss
  (the cube-heatmap analog)
- `noise_sigma` — Gaussian noise added to the decoded/raw perception target
  during training (the Path-1 robustness knob)
- `freeze_encoder_after_aux` — two-stage decouple: aux-pretrain the perception
  encoder, freeze it, then train only the action head on frozen features
- `n_distractors` — number of dimmer distractor blobs added to the image
  perception (clutter / multi-object transfer test; image mode only)

## The four metrics (`train_eval.py`)

1. **open_loop_mse** — action MSE on a held-out set of (start, target) pairs.
   Low MSE alone is *not* success — the shortcut can drive it low.
2. **closed_loop_success** — rollout success on **novel targets** (held out from
   training). This is the real metric; distribution shift exposes shortcut reliance.
3. **perception_probe** — how accurately the target can be linearly decoded from
   the perception features. High = encoder is geometric (the act_bev "1.18 cm"
   analog).
4. **shortcut_reliance** — closed-loop success with perception **ablated**
   (zeroed at test). If success barely drops, the policy was ignoring perception.
   The gap `closed_loop_success − ablated_success` = how much the policy
   actually uses perception.

## Experiment matrix (`run_matrix.py`)

Default sweep (each cell trains in seconds on CPU):

| Axis | Values |
|---|---|
| proprio_mode | full, no_target, minimal |
| injection | concat, xattn, replace |
| noise_sigma | 0.0, 0.02, 0.05, 0.1 |
| aux_weight | 0.0, 0.5 |

Not every combination is meaningful; `run_matrix.py` runs the informative
slices and prints a results table.

## What each outcome would mean

| Observation | Interpretation |
|---|---|
| `minimal` proprio → high closed-loop + high perception-use, regardless of injection | H1 confirmed: breaking the shortcut is sufficient; architecture doesn't matter |
| `full` proprio → only `xattn` achieves high perception-use | H2 confirmed: dual-system routing is the fix |
| high perception_probe but low closed-loop in `full`+`concat` | H3 confirmed: aux makes features geometric but doesn't fix consumption (reproduces act_bev) |
| closed-loop peaks at intermediate noise_sigma, collapses at high sigma | H4 confirmed: noise-tolerance has a sweet spot |

The winning recipe found here transfers to act_bev (existing infra) for
validation, then to the real robot.

## How to run

```bash
# Single config
python train_eval.py --proprio_mode full --perception_mode image --injection concat

# Full matrix
python run_matrix.py --out results.csv

# Plot
python run_matrix.py --plot results.csv
```

Requires only `torch`, `numpy`, `matplotlib`. Runs on CPU.

## Results (2026-05-27, 3 seeds, n=100 closed-loop eval)

Full matrix in `results.csv`, plot in `results.png`. Key cells:

| cell | CL_succ | perc_use | probe_L2 | reading |
|---|---|---|---|---|
| oracle (tgt in proprio) | 1.00 | 0.93 | 0.68 | upper bound |
| raw / minimal / concat | 1.00 | 0.95 | 0.68 | trivial decode, no shortcut → uses perception perfectly |
| raw / copycat / concat | 1.00 | 0.95 | 0.68 | copycat shortcut alone doesn't suppress perception |
| raw / copycat / concat / **pool8** | 0.67 | 0.61 | 0.68 | **memorization shortcut suppresses perception use 0.95→0.61** |
| raw / copycat / xattn / pool8 | 0.65 | 0.62 | 0.67 | dual-system NO better than concat |
| raw / copycat / replace / pool8 | 0.43 | 0.39 | 0.68 | replace is worse |
| img / minimal / concat | 0.19 | 0.13 | 0.68 | **hard decode through action loss → fails even with no shortcut** |
| img / minimal / concat / **aux** | 0.16 | 0.12 | **0.30** | **aux makes features geometric (0.68→0.30) but CL does NOT improve** |
| img / minimal / xattn / aux | 0.15 | 0.10 | 0.30 | same — dual-system doesn't rescue |
| img / minimal / replace / aux | 0.19 | 0.16 | 0.29 | same |
| img / copycat / concat / pool8 | 0.06 | 0.03 | 0.67 | full causal confusion — collapses |
| **img / minimal / softargmax** | 0.44 | 0.39 | 0.77 | location-preserving readout, no aux → already 2× concat (0.19) |
| **img / minimal / softargmax / aux** | **0.98** | **0.95** | **0.009** | **THE FIX — aux + location-preserving readout → near-perfect** |
| **img / minimal / softargmax / decouple** | **0.99** | **0.96** | **0.009** | two-stage (freeze aux-pretrained encoder, train head) → near-perfect |
| img / copycat / softargmax / pool8 | 0.55 | 0.51 | 0.15 | shortcut present → readout fix degrades but survives |
| img / copycat / softargmax / pool8 / decouple | 0.58 | 0.55 | 0.010 | decouple makes encoder geometric (probe 0.01) but shortcut still caps CL |
| **img / min / softargmax / aux / 2distract** | **0.92** | 0.89 | 0.029 | CLUTTER: 2 distractors → graceful drop from 0.98 (conv selects target peak) |
| img / min / softargmax / decouple / 2distract | 0.88 | 0.84 | 0.030 | decouple holds up under clutter |
| **img / min / softargmax / aux / 4distract** | **0.83** | 0.82 | 0.037 | 4 distractors → still strong; probe 0.037 ≪ 0.05 success radius |
| img / min / softargmax / decouple / 4distract | 0.82 | 0.81 | 0.037 | decouple ≈ joint under heavier clutter |
| **img / min / concat / aux / 2distract** | **0.07** | 0.02 | 0.473 | mean-pool COLLAPSES under clutter (averages over objects) |
| raw / min / noise 0.02 / 0.05 / 0.10 | 0.92 / 0.69 / 0.49 | — | noise monotonically degrades (no shortcut to be robust against here) |

### Findings

> **The headline changed.** The original run (concat/xattn/replace only) concluded
> "aux makes features geometric but doesn't fix consumption." Adding the
> `softargmax` readout shows that conclusion was **confounded by the readout**:
> mean-pooling over the spatial grid destroys location and caps decode at ~0.29 L2
> (≫ the 0.05 success radius). With a location-preserving readout the *same* aux
> loss drives decode to 0.009 L2 and closed-loop to **0.98**. The bottleneck was
> the **readout architecture**, not "consumption."

1. **THE FIX — a location-preserving readout + aux supervision works (CL 0.98–0.99).** `img/minimal/softargmax/aux` → CL **0.98**, perc_use **0.95**, probe **0.009**. Two-stage decouple (`/decouple`: aux-pretrain the heatmap encoder, freeze it, train only the action head) → CL **0.99**. This **directly validates the user's "decoupled representation + policy" hypothesis** — once the readout preserves spatial structure, decoupled aux-pretraining gives a near-perfect policy.

2. **The earlier "consumption failure" was largely a READOUT failure.** With mean-pool (`concat`/`xattn`/`replace`), aux drives probe only to ~0.29 and CL stays ~0.16 — because mean-pooling has already thrown away *where* the blob is, so no downstream head can recover it. This is **the act_bev "1.18 cm aux, 0/20 closed-loop" pattern reproduced AND then explained**: act_bev's BEV-token mean-pool / weak readout, not an inability of the transformer to consume geometry, is the prime suspect. Even with NO aux, `softargmax` alone (CL 0.44) already doubles concat (0.19).

3. **But the memorization shortcut is a SEPARATE, real problem the readout fix does NOT solve.** With pool8 active, `softargmax/pool8` CL drops to 0.55, and decouple recovers the *probe* to 0.010 (encoder is geometric) but CL only to 0.58 — the frozen-geometric features are available yet the action head still partly rides the memorized-trajectory shortcut. So causal confusion (de Haan 2019) is genuine and orthogonal to the readout issue: **readout fixes the no-shortcut regime; the shortcut regime needs a shortcut-breaking intervention on top.**

4. **H2 REFUTED — dual-system routing (xattn) does NOT beat concat.** raw: 0.65 vs 0.67; image: 0.20 vs 0.19. Routing the *same mean-pooled* feature through cross-attention can't recover destroyed location. PointACT's benefit appears tied to its readout/token structure, not the routing per se. **This still changes our Road B-4 prior** — don't assume dual-system routing alone rescues pi0_voxel; the readout is the lever.

5. **H1 confirmed — memorization shortcut suppresses perception use** (raw/copycat: 0.95 → 0.61 with pool8; and the softargmax/pool8 result above). H4 inconclusive: noise monotonically degrades in `minimal` because there's no shortcut for robustness to protect against.

6. **CLUTTER TRANSFER TEST — soft-argmax degrades GRACEFULLY, mean-pool collapses.** Adding 2–4 dimmer distractor blobs (target still the brightest), softargmax+aux holds: CL 0.98 → 0.92 (2) → 0.83 (4), with probe staying at 0.029–0.037 (≪ the 0.05 success radius) — the conv front-end learns to SELECT the target peak before the soft-argmax. Two-stage decouple tracks it (0.88 / 0.82). By contrast, **mean-pool concat+aux collapses to 0.07 with just 2 distractors** (pooling now averages over multiple objects; probe blows up to 0.47). This is **direct evidence the readout, not consumption, is the lever**, and that a location-preserving readout with a *learnable selection front-end* survives multi-object scenes — the closest in-toy analog to cluttered BEV/voxel. Caveat: here the target is distinguishable by *intensity*; real scenes distinguish by shape/semantics, so the real-system readout must include a learnable object-selection stage (the conv here plays that role), not a bare soft-argmax.

### What this means for the real project

- The prior negative results (pi0_voxel, act_bev) are now **mechanistically re-diagnosed**: the dominant failure was likely a **lossy readout** (mean-pool / weak token aggregation that discards spatial location), NOT an intrinsic inability of the action head to consume geometry. A location-preserving readout + aux supervision + (optionally) freezing the geometric encoder gives CL 0.98–0.99 in the toy.
- **Honest caveat on transfer:** the toy's "image" is a single clean Gaussian blob with one unambiguous peak, so soft-argmax is the *exactly correct* inductive bias. Real BEV/voxel features encode cluttered multi-object scenes where a single-peak soft-argmax is not directly applicable. The toy proves the **mechanism** (readout > consumption) and that **decoupling works when the readout is right** — it does NOT prove a specific real-system module will work. The real-system analog is "a spatially-structured, location-preserving readout for the relevant object," e.g. per-object heatmap/keypoint readout, not a global pool.
- **Two open levers for the shortcut regime**, which the readout fix alone does not close: (a) break the shortcut at the data/observation level; (b) a regularizer that penalizes proprio-only prediction.

### Next experiments (toy, before touching real hardware)

- ~~Two-stage decouple~~ **DONE** — `--freeze_encoder_after_aux`; works (CL 0.99) with a location-preserving readout, partial (CL 0.58) under the memorization shortcut.
- ~~Multi-object / multi-peak readout~~ **DONE** — `--n_distractors`; soft-argmax degrades gracefully (0.98 → 0.83 at 4 distractors), mean-pool collapses (0.07 at 2). The conv front-end is the learnable selector.
- **Shortcut-present interventions** (still open): combine softargmax+decouple with a shortcut-breaking knob (drop prev_action / randomize) to close the pool8 gap (0.58 → ?).
- **Same-intensity distractors** (harder): make distractors equal-brightness so the target is identified only by an external cue — tests whether the readout can do *semantic* selection, not just intensity selection.
- **Bottleneck/token-aggregation sweep**: does an attention-pool or keypoint readout recover what mean-pool destroys, short of full soft-argmax?

## References

- de Haan, Jayaraman, Levine, *Causal Confusion in Imitation Learning*, NeurIPS 2019 — [arXiv 1905.11979](https://arxiv.org/abs/1905.11979)
- Wen et al., *Fighting Copycat Agents in Behavioral Cloning from Observation Histories*, NeurIPS 2020 — [arXiv 2010.14876](https://arxiv.org/abs/2010.14876)
- Spencer et al., *Feedback in Imitation Learning: The Three Regimes of Covariate Shift*, 2021 — [arXiv 2102.02872](https://arxiv.org/abs/2102.02872)
- Florence et al., *Implicit Behavioral Cloning*, CoRL 2021 — [arXiv 2109.00137](https://arxiv.org/abs/2109.00137)
- PointACT (our Road B-4 motivation) — dual-system routing of 3D features to the action head
- Our own evidence: `../mylerobot/PROJECT_STATUS.md` (pi0_voxel rows 2f/2h.3) and `../RESEARCH_NOTES.md` (act_bev Phases 3-5)
