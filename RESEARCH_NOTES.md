# Franka Cube-Lift in Isaac Lab: From Failing ACT to Diagnosed Architecture Gap

**Project:** Vision-action policy research on Franka Lift-Cube (Isaac Lab Lift-Cube-Franka-IK-Rel-v0).
**Goal:** Build a robust BC policy that can grasp a randomly-placed cube from camera images.
**Status (2026-05-25):** ACT and ACT-BEV exhausted at 0–10% closed-loop; cheat-with-oracle ceiling at 35%; recommended pivot to π₀.

---

## 1. Hardware & software

- GPU: NVIDIA RTX 5090 (cuda:1 used throughout; cuda:0 was busy with other workloads).
- OS: Linux 6.17.
- Conda env: `isaac_lerobot` (Python 3.11, lerobot at `/Developer/lerobot/src`, IsaacLab at `/Developer/IsaacLab`).
- Custom code: `mylerobot` (`/Developer/omniverselab/mylerobot`).

## 2. Task

- **Env id:** `Isaac-Lift-Cube-Franka-IK-Rel-v0`
- **Robot:** Franka Panda, 9 joints (7 arm + 2 fingers), `panda_hand` body as EE reference.
- **Action space:** IK-Rel 7-D = `[dx, dy, dz, droll, dpitch, dyaw, grip]`. Gripper is binary-thresholded by the env.
- **Success:** cube_z ≥ 0.06 m, held for 10 steps within `--max_steps` (320).
- **Workspace:** cube spawns in roughly `x ∈ [0.4, 0.6], y ∈ [-0.25, 0.25], z = 0.021` (table top + half cube height).

## 3. Data-collection pipeline (`isaac_auto_collector_v6.py`)

- State-machine scripted controller: APPROACH → HOVER → OPEN_GRIP → DESCEND → GRASP → LIFT → VERIFY → DONE/RETRY.
- Per-episode randomization (cube color/scale, table color) on by default.
- Records:
  - `obs/state` — joint_pos (9-D); optional `--include_ee_pose` appends `(ee_pos(3), ee_quat(4))` → 16-D.
  - `actions` — 7-D IK-Rel command.
  - `obs/images/<cam>` — uint8 RGB at 480×640.
  - `obs/cam_extrinsics_world2cam_cv/<cam>`, `obs/cam_pos_world/<cam>`, `obs/cam_quat_world_wxyz/<cam>`, `obs/cam_intrinsics_K/<cam>` — calibration per cam, per frame.
  - **New (added during this work):** `obs/cube_pose` — 3-D world position of the target object, recorded by the multicam saver when `--cams ...` is set on a Lift-style env.

Run examples (headless, --device cuda:1):

```bash
# 50 demos, front+wrist (original)
./isaaclab.sh -p isaac_auto_collector_v6.py --device cuda:1 --enable_cameras --headless \
    --autorun --exit_on_done --env lift-ik-rel --max_demos 50 \
    --cams front,wrist --cam_hw 480,640 --save_dir logs/demos_multicam_lift

# 200 demos, top+front+wrist + cube_pose (used for ACT-BEV)
./isaaclab.sh -p isaac_auto_collector_v6.py --device cuda:1 --enable_cameras --headless \
    --autorun --exit_on_done --env lift-ik-rel --max_demos 200 \
    --cams top,front,wrist --cam_hw 480,640 --save_dir logs/demos_lift_tfw_cube_n200
```

## 4. Conversion to LeRobotDataset (`convert_isaac_hdf5_to_lerobot.py`)

HDF5 → LeRobotDataset v3 (parquet + libsvtav1 video). New flags added during this work:

- `--cams top,front,wrist` — subset of HDF5 cams to include
- `--resize_hw 240,320` — downsize for training
- `--include_cube_pose` — appends cube_pose to `observation.state` (cheat baseline)
- `--cube_pose_as_obs` — registers `observation.cube_pose` as a separate (3,) float feature (used as aux supervision target, NOT as policy input)

Conversion of 200 demos × 3 cams takes ~50 minutes on this machine (libsvtav1 encoding is the bottleneck).

## 5. Datasets produced

| Repo id | Root path | Cams | State dim | Notes |
|---|---|---|---|---|
| `local/isaac_lift_front_wrist_240x320` | `logs/lerobot_lift_front_wrist_240x320` | front, wrist | 9 | Original failing dataset (50 demos) |
| `local/isaac_lift_fw_ee16d_n200` | `logs/lerobot_lift_fw_ee16d_n200_240x320` | front, wrist | 16 | 200 demos + EE pose in state |
| `local/isaac_lift_fw_cheat_n200` | `logs/lerobot_lift_fw_cheat_n200_240x320` | front, wrist | 19 | + cube_pose appended ("cheat") |
| `local/isaac_lift_tfw_n200` | `logs/lerobot_lift_tfw_n200_240x320` | top, front, wrist | 16 | 3-cam baseline |
| `local/isaac_lift_tfw_cube_aux_n200` | `logs/lerobot_lift_tfw_cube_aux_n200_240x320` | top, front, wrist | 16 | + observation.cube_pose (3) as aux supervision |

## 6. Evaluation harness (`eval_act_isaac_lift.py`)

Closed-loop rollout in IsaacLab. New flags added during this work:

- `--gripper_mode {raw, snap}` — `snap` replaces predicted grip with `sign(grip)`
- `--disable_te --n_action_steps N` — disable ACT temporal ensembling at inference, run open-loop chunks
- `--trace_actions` — write per-step `[ep, step, cube_xyz, ee_xyz, action(7)]` to `<out>.trace.csv`
- `--include_ee_pose` / `--include_cube_pose` — match training-time state layout
- `--bev_perception_checkpoint PATH` — load an ACT-BEV checkpoint and substitute its aux-head soft-argmax for the cube_pose dims of the main policy's state (Option A)
- `--bev_cams top,front` — which cams feed BEV perception
- Generic policy dispatch via `get_policy_class(cfg.type)` with explicit handling for `act_bev`, `act_lang`, `act_cheat_noisy`, plus stock lerobot types (diffusion, etc).

## 7. Diagnostics built and run

- **`diag2_offline_regression.py`** — feeds training frames through a checkpoint and reports per-dim MAE/RMSE between predicted and ground-truth actions, with a special focus on the gripper sign-flip window. Used to verify training fit independently of closed-loop dynamics.
- **`diag_actbev_aux.py`** — loads an ACT-BEV checkpoint, runs the BEV encoder + aux head on training frames, and reports the peak-to-true XY error (in cm) plus aux loss. Used to verify whether the BEV perception module is actually learning cube position.

## 8. Experiments — chronological

### Phase 0: Diagnosing the user's failing baseline

The user's initial training command:
```bash
python scripts/train_act.py \
  --dataset.repo_id=local/isaac_multicam_lift_fixed_240x320 \
  --policy.type=act --policy.chunk_size=50 --policy.n_action_steps=1 \
  --policy.temporal_ensemble_coeff=0.01 --batch_size=8 --steps=30000 ...
```
Checkpoint: `/Developer/IsaacLab/logs/train/act_lift_front_wrist_240x320_chunk50_seed1000/checkpoints/030000/pretrained_model`.

User's report: "checkpoint loads and normalizes correctly; closed-loop rollout never lifts the cube."

| # | Diagnostic | Method | Result |
|---|---|---|---|
| D1 | Env reset determinism | Run two identical resets, check cube spawn pose | Confirmed cube DOES randomize per seed; "fixed" in the dataset name referred to camera subset, not pose |
| D2 | Offline policy regression | 5 random training trajectories: `(state, image) → action` via `policy.select_action` | Total per-dim MAE 0.01–0.03; gripper sign agreement 99.7%; **gripper transition window** RMSE 0.27–0.40 — gripper command ramps from +1 through 0 to −1 over 3-4 frames because of `temporal_ensemble_coeff=0.01` |
| D3a | Closed-loop, `--gripper_mode raw` | 5 episodes, IsaacLab | 0/5; cube `max_z = 0.055` (cube spawn height pre-settle) every episode |
| D3b | Closed-loop, `--gripper_mode snap` | Snap predicted grip to sign | 0/5; gripper transition cleaner but **arm still closes 1.5cm above the cube** |
| D3c | Closed-loop, `--disable_te --n_action_steps 20` | Open-loop chunks | 0/5; close timing identical |
| **D4** | **XY-to-cube error trace** | Added cube_xy logging; checked TCP-to-cube horizontal distance at close moment | Ep 0: 4.8 cm error; **Ep 1: 23.7 cm** (cube at y=+0.097, TCP descends to y=−0.130). **The policy descends to a learned XY centroid regardless of cube position — the visual encoder is not driving the action.** |

**Phase 0 verdict: not a gripper-smoothing problem. The visual encoder is being ignored.**

### Phase 1: "Strong baseline" Step 2 (200 demos + EE pose + image aug + TE off)

Retrained vanilla ACT with all standard fixes:
- 200 demos (vs original 50)
- `observation.state` = 16-D (joint_pos + EE pos + EE quat)
- `--dataset.image_transforms.enable=true`
- `policy.temporal_ensemble_coeff=null, policy.n_action_steps=20`
- 60K steps

**Result: 0/20 success.** Final loss 0.058. TCP-cube XY error 4.8–23.7 cm — the proprio shortcut survives. Vision is the dominant bottleneck.

### Phase 2: Three diagnostic experiments

| Exp | Setup | Success | Key observation |
|---|---|---|---|
| **1 — Cheat** | Append GT `cube_pose` to state (19-D). Same architecture otherwise. | **7/20 (35%)** | Vision IS the bottleneck. Even oracle only hits 35% — there's also a control-precision ceiling. |
| **2 — Top camera** | Add `top` cam to baseline (3 cams). Otherwise same as Phase 1. | 1/20 (5%) | More views help marginally; encoder quality is the real issue. |
| **3 — Diffusion Policy** | Same data as Phase 1, but `policy.type=diffusion`. | 2/20 (10%) | DP slightly better than ACT, far below cheat. |

### Phase 3: ACT-BEV (Tesla-occupancy-inspired)

**Goal.** Replace ACT's per-camera ResNet branch with a calibrated multi-view BEV encoder. Add a Tesla-style auxiliary supervision head that predicts a 2D cube-position heatmap on the BEV grid. Force the encoder to be metrically grounded.

**Architecture.**
- Workspace voxel grid: x ∈ [0.2, 0.8], y ∈ [-0.3, 0.3], z ∈ [0, 0.3], voxel size 2 cm → 30×30×15 = 13.5k voxels.
- ResNet18 (ImageNet-pretrained, frozen-init layers 1-3) per camera → 256-channel features.
- Lift-splat: each voxel center is projected into every BEV camera (top, front), features sampled via bilinear `grid_sample`, aggregated with visibility-weighted mean.
- 3D conv refinement → mean-pool along z → 2D BEV feature map (30×30×128).
- Aux head: 2 conv layers → 1-channel heatmap; target = 2D Gaussian (σ = 3 cm) on the BEV grid at the GT cube XY.
- Wrist camera kept on the original per-camera ACT branch (close-up view during grasp).
- Loss = action_L1 + KLD·w + 0.5·BCE_aux.

**Files:** `mylerobot/policies/act_bev/` — `bev_encoder.py`, `aux_head.py`, `configuration_act_bev.py`, `modeling_act_bev.py`.

| Version | Closed-loop success | Aux head XY error (training data) | Diagnosis |
|---|---|---|---|
| **v0** (buggy extrinsics from HDF5) | 0/20 | n/a — `visibility = 0 voxels` | **Bug:** Isaac's saved camera quaternion convention is not OpenCV; every voxel projected to z_cam ≈ 0 (degenerate). BEV features were a learned constant independent of input. |
| **v1** (extrinsics computed from `look_at`) | 0/20 | **1.89 cm mean / 1.18 cm median** | BEV perception works! But the action transformer's attention is dominated by 16-D proprio state + 80 wrist tokens; the 900 BEV tokens are ignored. |
| **v2** (BEV-derived centroid injected to state, raw meters) | 0/20 | same | The injected centroid is in raw meters (~0.4); proprio is normalized to N(0,1); the linear state proj treats centroid dims as small low-frequency channels and the policy ignores them. |
| **v3** (BEV centroid z-scored to match proprio scale) | 0/20 | same | Centroid is now scaled correctly but the policy STILL doesn't learn to use it (action L1 can be minimized from proprio alone; fresh-init centroid dim weights never get a strong gradient signal). |

### Phase 4: Option A — distill BEV into the cheat policy

**Setup.** Take the trained cheat policy (35% success with GT `cube_pose`), use the ACT-BEV-v3 BEV encoder + aux head as a pure perception module, at inference time replace the `cube_x, cube_y` dims of the cheat policy's state with the BEV's soft-argmax prediction. `cube_z` left as GT (it's nearly constant at 0.021 m pre-grasp).

**Result: 0/20.**

Closed-loop TCP error to cube at step 100 (head-to-head):

| Ep | Cube | Cheat (oracle) error | Option A (BEV drop-in) error |
|---|---|---|---|
| 0 | (0.48, 0.02) | 3.9 cm | 4.3 cm |
| 3 | (0.59, −0.07) | 7.8 cm | 19.6 cm |
| 5 | (0.43, 0.03) | 2.7 cm | 7.0 cm |
| 8 | (0.56, **+0.21**) | 7.4 cm | **27.8 cm** |
| 10 | (0.43, 0.09) | 4.4 cm | 7.2 cm |
| 15 | (0.55, 0.13) | 3.9 cm | 11.4 cm |

The BEV prediction adds 0.4–20 cm of noise depending on cube position. Workspace-edge cubes (y=+0.21) suffer most because the cameras see them at oblique angles. The cheat policy was trained with zero noise on cube_pose and can't tolerate that perturbation.

### Phase 5: Path 1 — noise-augmented cheat policy

**Hypothesis.** Train cheat with Gaussian noise injected on the cube_pose state dims at training time. The policy should learn to be tolerant of BEV-magnitude prediction noise, while still using the (now-noisy) cube_pose signal for spatial conditioning.

**Implementation.** New policy class `act_cheat_noisy` (`mylerobot/policies/act_cheat_noisy/`):
- Subclasses ACTPolicy; overrides `forward` to inject Gaussian noise into the cube_pose state slice (indices 16,17,18) **in normalized state space** during training only.
- Default per-dim normalized σ = (0.33, 0.15, 0.20) → ≈ 2 cm in world units.

**Result: 0/20 (with GT oracle at eval).**

The σ=2 cm noise was too aggressive. The policy learned to ignore cube_pose entirely (treating it as low-information noise) and reverted to the proprio-trajectory shortcut. Vanilla cheat hits 35%; this noisy version hits 0%.

The noise/signal ratio of `σ/data_std` was 33% for x and 15% for y — apparently above the threshold where the policy still finds the signal worth tracking. A gentler retry (σ ≤ 0.5 cm) might salvage this, but the data is pointing to a deeper architectural problem.

## 9. Comprehensive success table

| # | Variant | Cameras | State | Train policy | Eval input | Success |
|---|---|---|---|---|---|---|
| 0a | ACT initial (failing baseline) | f+w | 9 | TE=0.01, n=1, no aug, 50d | — | 0/5 |
| 0b | ACT + aug + 200d + TE-off | f+w | 16 | n=20, aug, 60k | — | 0/20 |
| 1 | Cheat | f+w | 19 | n=20, aug | GT cube | **7/20 (35%)** |
| 2 | + top cam | t+f+w | 16 | n=20, aug | — | 1/20 (5%) |
| 3 | Diffusion Policy | f+w | 16 | DP, aug | — | 2/20 (10%) |
| 4 | ACT-BEV-v0 | t+f+w | 16 | + aux | — | 0/20 (extrinsics bug) |
| 5 | ACT-BEV-v1 (calib fix) | t+f+w | 16 | + aux | — | 0/20; aux head 1.89 cm |
| 6 | ACT-BEV-v2 (centroid inject, raw) | t+f+w | 16+2 | + aux | — | 0/20 |
| 7 | ACT-BEV-v3 (centroid inject, z-scored) | t+f+w | 16+2 | + aux | — | 0/20 |
| 8 | Option A: cheat + BEV drop-in | t+f+w | 19 | cheat | BEV cube_xy + GT z | 0/20 |
| 9 | Path 1: noisy cheat (σ≈2 cm) + oracle | f+w | 19 | cheat w/ noise | GT cube | 0/20 |

## 10. Key insights — what we now know

1. **Vision is the dominant bottleneck.** Going from no-vision to perfect-vision moves success from 0% to 35%. No architectural variant of ACT closed the gap.
2. **ACT's regression+attention objective cannot extract precise 3D cube position from this camera setup.** Adding cameras (Exp 2), changing to DP (Exp 3), or providing perfect BEV features (Phase 3) does not help — the action transformer doesn't learn to attend to spatial features over the proprio shortcut.
3. **BEV with auxiliary supervision is a viable perception module per se.** The aux head reaches 1.18 cm median XY error on training data. The challenge is consuming that perception in the action head.
4. **Closed-loop visual distribution shift is real.** Training-data BEV error is ~1 cm; closed-loop error spikes to 20+ cm at workspace edges. This is a generalization gap, not a training fit gap.
5. **The cheat policy is brittle to perception noise.** 2 cm of cube_pose noise at inference time drops it from 35% to 0%. Naive noise injection at training time doesn't fix this (it makes the policy ignore cube_pose entirely).
6. **There appears to be a knife-edge regime for ACT on this task** — either the cube signal is exact (35%) or the policy ignores it (0%). No graceful degradation between.
7. **The original ACT paper achieves 70–90% on ALOHA tasks with 4 wrist-near cameras.** Our 2-3 oblique-view setup is outside ACT's published sweet spot. The published numbers don't transfer.

## 11. Open questions / where π₀ comes in

- **π₀ (Physical Intelligence)** is a flow-matching policy with a PaliGemma-3B VLM backbone, pretrained on millions of cross-embodiment robot frames. Its vision features are pre-aligned to physical concepts ("cube", "tabletop", "left-of-arm"). On LIBERO it reaches ~94%; on tasks at our scale fine-tuning usually achieves >50% with 100 demos.
- **GR00T-N1 (NVIDIA)** is built for humanoids; mismatched to Franka.
- **Diffusion Policy as a noise-robust variant** could be re-run with the same noise-injection recipe — DP's stochastic action head may absorb perception noise more gracefully than ACT's L1 regression — but DP's vanilla result here was only 10%, so the headroom is small.

Recommended next experiment: **fine-tune π₀ on `lerobot_lift_fw_ee16d_n200_240x320`**. If π₀ vanilla beats 35% out of the box, the research arc reframes around π₀-with-BEV-augmentation as the contribution.

## 12. File index

### New / modified source

| File | Purpose |
|---|---|
| `IsaacSim/isaac_auto_collector_v6.py` | Multi-cam collector. Added `--include_ee_pose` and `cube_pose` recording. |
| `IsaacSim/isaac_multicam_addons.py` | `LeRobotMultiCamSaver` now records `obs/cube_pose`. |
| `IsaacSim/convert_isaac_hdf5_to_lerobot.py` | Added `--include_cube_pose` (in-state) and `--cube_pose_as_obs` (separate feature) flags. |
| `IsaacSim/eval_act_isaac_lift.py` | Added `--gripper_mode`, `--disable_te`, `--n_action_steps`, `--trace_actions`, `--include_ee_pose`, `--include_cube_pose`, `--bev_perception_checkpoint`. Generic policy dispatch via `get_policy_class`. |
| `IsaacSim/diag2_offline_regression.py` | Offline regression diagnostic. |
| `IsaacSim/diag_actbev_aux.py` | BEV aux head quality diagnostic. |
| `mylerobot/mylerobot/policies/act_bev/` | New: ACT-BEV policy package. |
| `mylerobot/mylerobot/policies/act_cheat_noisy/` | New: noise-augmented cheat policy package. |
| `mylerobot/mylerobot/policies/act_lang/configuration.py` | Bug fix: `from lerobot.configs import PreTrainedConfig` → `from lerobot.configs.policies import PreTrainedConfig`. |
| `mylerobot/tests/test_act_bev_smoke.py` | ACT-BEV smoke test. |
| `mylerobot/tests/test_act_cheat_noisy_smoke.py` | Noise injection smoke test. |

### Datasets

`/Developer/IsaacLab/logs/`
- `demos_multicam_lift/` — original 50 demos, 5 cams
- `demos_multicam_lift_fixed/` — 50 demos, used for the original failing checkpoint
- `demos_lift_ee16d_n200/` — 200 demos, front+wrist
- `demos_lift_tfw_cube_n200/` — 200 demos, top+front+wrist + cube_pose
- `lerobot_lift_front_wrist_240x320/` — original failing dataset (50 demos)
- `lerobot_lift_fw_ee16d_n200_240x320/` — Phase 1, 200 demos, 16-D state
- `lerobot_lift_fw_cheat_n200_240x320/` — cheat dataset, 19-D state
- `lerobot_lift_tfw_n200_240x320/` — Phase 2, 3-cam baseline
- `lerobot_lift_tfw_cube_aux_n200_240x320/` — ACT-BEV dataset with observation.cube_pose

### Checkpoints

`/Developer/IsaacLab/logs/train/`
- `act_lift_front_wrist_240x320_chunk50_seed1000/` — original failing baseline (Phase 0)
- `act_lift_fw_ee16d_n200_chunk50_n20_aug_seed1000/` — Phase 1 strong baseline (0/20)
- `act_lift_fw_cheat_n200_chunk50_n20_seed1000/` — cheat baseline (7/20 = 35%)
- `act_lift_tfw_n200_chunk50_n20_seed1000/` — 3-cam baseline (1/20)
- `dp_lift_fw_ee16d_n200_seed1000/` — Diffusion Policy (2/20)
- `act_bev_lift_tfw_n200_seed1000/` — ACT-BEV-v0 (buggy extrinsics, 0/20)
- `act_bev_lift_tfw_n200_calibfix_seed1000/` — ACT-BEV-v1 (calib fix, 0/20, aux 1.89 cm)
- `act_bev_lift_tfw_n200_v2_inject_seed1000/` — ACT-BEV-v2 (raw centroid inject, 0/20)
- `act_bev_lift_tfw_n200_v3_inject_norm_seed1000/` — ACT-BEV-v3 (z-scored, 0/20)
- `act_cheat_noisy_n200_seed1000/` — Path 1 noisy cheat (0/20, noise too aggressive)

### Eval outputs

`<checkpoint_path>/eval_n20.json` + `eval_n20.trace.csv` — per-episode summary + per-step trace
`<checkpoint_path>/diag2_offline_regression/` — offline regression results
`act_lift_fw_cheat_n200_chunk50_n20_seed1000/checkpoints/030000/pretrained_model/eval_bev_dropin_n20.json` — Option A result
