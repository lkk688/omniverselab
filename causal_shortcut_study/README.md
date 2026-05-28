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
- `concat` — perception encoder → flatten → concat to proprio → MLP (the
  act_bev v2/v3 design that failed)
- `xattn` — proprio is the query, perception tokens are keys/values,
  cross-attention → action (dual-system / PointACT-style)
- `replace` — perception decoded to an explicit `tgt_hat`, substituted into the
  proprio target slot (act_bev Option-A style)

### Training knobs
- `aux_weight` — weight on an auxiliary "predict target from perception" loss
  (the cube-heatmap analog)
- `noise_sigma` — Gaussian noise added to the decoded/raw perception target
  during training (the Path-1 robustness knob)

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
| raw / min / noise 0.02 / 0.05 / 0.10 | 0.92 / 0.69 / 0.49 | — | noise monotonically degrades (no shortcut to be robust against here) |

### Findings

1. **H3 CONFIRMED (the headline) — aux supervision makes features geometric but does NOT fix consumption.** `img/minimal` with aux drives the perception-probe from 0.68 (chance) to 0.30, yet closed-loop success stays flat at ~0.17. This is **the act_bev result (1.18 cm aux localization, 0/20 closed-loop) reproduced in an 8-second toy.** Geometric representation ≠ the action head using it.

2. **The real bottleneck is decode-through-action-loss, not architecture.** `raw` (trivial 2-number decode) → 1.00 success; `image` (CNN decode of a blob) → 0.19, regardless of injection. The credit-assignment problem of learning to extract geometry from raw sensory input *through the action loss* is the wall.

3. **H2 REFUTED in this toy — dual-system routing (xattn) does NOT beat concat.** raw: 0.65 vs 0.67; image: 0.20 vs 0.19. PointACT's dual-system benefit appears to be task/scale-specific, not a universal fix. **This is an important negative result that changes our Road B-4 prior** — we should not assume dual-system routing alone will rescue pi0_voxel.

4. **H1 PARTIALLY confirmed — memorization shortcut suppresses perception use** (raw/copycat: 0.95 → 0.61 with pool8), demonstrating causal confusion. But the *image-decode* failure is not a shortcut problem — it persists even in `minimal` (no shortcut at all).

5. **H4 inconclusive here** — noise monotonically degrades in `minimal` mode because there's no shortcut for noise-robustness to protect against. The sweet-spot test needs a shortcut-present setting (future cell).

### What this means for the real project

- The two prior negative results (pi0_voxel, act_bev) are now **mechanistically explained by a 30-line toy**: the action head cannot learn to extract+consume geometry from raw perception through the action loss alone, and neither aux supervision nor dual-system routing fixes it in isolation.
- **The most promising untested lever is two-stage decoupling**: freeze an aux-pretrained (geometric) encoder, then train ONLY the action head on the frozen geometric features. If that works, the problem was joint-training credit assignment; if it still fails, the action head genuinely can't consume geometry. This is the next toy experiment, and it directly tests the user's original "decoupled representation + policy" hypothesis.

### Next experiments (toy, before touching real hardware)

- **Two-stage decouple**: `--freeze_encoder_after_aux` — pretrain encoder with aux, freeze, train action head. The decisive test.
- **Shortcut-present noise sweep**: noise-tolerance sweet spot with pool8 shortcut active (proper H4 test).
- **Bottleneck width sweep**: does a wider perception bottleneck / more tokens change the image-decode wall?

## References

- de Haan, Jayaraman, Levine, *Causal Confusion in Imitation Learning*, NeurIPS 2019 — [arXiv 1905.11979](https://arxiv.org/abs/1905.11979)
- Wen et al., *Fighting Copycat Agents in Behavioral Cloning from Observation Histories*, NeurIPS 2020 — [arXiv 2010.14876](https://arxiv.org/abs/2010.14876)
- Spencer et al., *Feedback in Imitation Learning: The Three Regimes of Covariate Shift*, 2021 — [arXiv 2102.02872](https://arxiv.org/abs/2102.02872)
- Florence et al., *Implicit Behavioral Cloning*, CoRL 2021 — [arXiv 2109.00137](https://arxiv.org/abs/2109.00137)
- PointACT (our Road B-4 motivation) — dual-system routing of 3D features to the action head
- Our own evidence: `../mylerobot/PROJECT_STATUS.md` (pi0_voxel rows 2f/2h.3) and `../RESEARCH_NOTES.md` (act_bev Phases 3-5)
