# Real-World Robot Deployment — Step-by-Step Plan

Created 2026-05-27 after concluding the voxel-fusion-on-VLA line on LIBERO-Plus
yielded only negative empirical results. This document pivots the research
trajectory toward **factory-deployable manipulation**, with two parallel hardware
tiers and a phased plan that produces a publishable contribution roughly every
3 months while building toward the deployment goal.

This is the master plan. Companion docs:
- `PROJECT_STATUS.md` — historical sim-only experiments (LIBERO-Plus)
- `ROAD_B_PLAN.md` — current Direction A (Road B-4 / B-5) finishing on H100
- `LIBERO_PLUS_GUIDE.md` — LIBERO-Plus benchmark + eval procedures
- `SETUP.md` — software environment recipe

---

## 1. North-star goal

**Deploy a vision-language-action policy on a real robotic arm performing
useful factory tasks (pick/place, sorting, assembly), with multi-camera depth
input and edge-only compute.** Two parallel hardware tiers serve different
task complexity and budget envelopes; both share the same software stack.

This is a multi-quarter program, not a single paper. The plan below is
structured so that **each phase produces a publishable artifact** along the
way — even if the long-term goal slips, we accumulate venues-worth of work.

---

## 2. Hardware roster

### Tier H — High-end (research arm + flagship edge compute)

| Component | Spec | Role |
|---|---|---|
| **Panthera-HT arm** | 7-DoF, high-torque servos, [HighTorque-Robotics/Panthera-HT_Main](https://github.com/HighTorque-Robotics/Panthera-HT_Main) | Heavier-payload, precision-required tasks (assembly, peg-in-hole, dual-arm coordination) |
| **OpenArm-X** (alternative) | 6/7-DoF, open-source design | Backup if Panthera-HT integration drags |
| **Jetson Thor AGX** | 128 GB unified memory, 2 PFLOPS FP4, Blackwell GPU | On-arm inference for full π0 / Pi0-5 / Open-VLA-OFT scale models |
| **Orbbec Femto Bolt** ×1 | ToF, 4096×3072 RGB + 320×288 depth, range 0.25–2.88 m, mic array | Overhead fixed agentview |
| **Orbbec Gemini 336** ×2 | Stereo+IR + RGB, 1280×800 depth, range 0.15–3 m, IMU | Lateral views (one on each side of workspace) for true multi-view stereo voxel fusion |
| **PoE switch + tripods** | — | Multi-cam infrastructure |

**Total target inference latency**: ≤200 ms (5 Hz control) for π0-scale models on Thor.

### Tier L — Low-end (cheap arm + entry-level edge)

| Component | Spec | Role |
|---|---|---|
| **SO-ARM101** | 6-DoF, ~$300 hobbyist arm | Simple pick-and-place, drawer-open, button-press |
| **Jetson Orin Nano** | 8 GB unified mem, 40 TOPS INT8, Ampere GPU | Edge inference for distilled small VLAs (ACT-scale, ≤200 M params) |
| **Orbbec Gemini 336** ×1 | (same as above) | Single overhead RGB-D |
| **(Optional)** Webcam ×1 | $30 USB | Wrist-mount cheap RGB only |

**Total target inference latency**: ≤100 ms (10 Hz) for ACT-class models on Orin Nano.

### Shared development infrastructure

- **H100 HPC cluster** — pretraining, large-scale fine-tunes, sim experiments
- **RTX 5090 + Pro 6000 Blackwell workstation** — IsaacSim data collection,
  policy distillation, multi-cam dataset curation
- **Existing software**: `mylerobot`, `lerobot 0.4.4`, `openpi`, IsaacSim 5.0,
  LIBERO/LIBERO-Plus, robosuite, mujoco — see SETUP.md

### Camera mount + cabling parts list

Specific items to procure (US pricing as of mid-2026, approximate):

**Tier L mount kit** (~$250 total)
- Overhead tripod: **Manfrotto 290 Xtra** ($120) — 1.6 m height, stable enough for a Gemini 336
- Wrist-cam clamp: **SmallRig 2070 mini ball head + 1095 magic arm** ($45) — mounts a small webcam to the SO-ARM101 gripper bracket
- USB 3.0 cable: **5 m active USB 3.0 A-to-C** for the Gemini 336 ($30) — cheaper passive cables drop frames above 3 m
- Workspace lighting: **Neewer 660 LED panel** ($60) — uniform diffuse light; eliminates direct sunlight which kills IR depth

**Tier H mount kit** (~$700 total)
- Overhead boom: **Manfrotto 055XPRO3 + 196B-3 horizontal extension** ($330) — 2 m reach, holds a Femto Bolt at 1 m above workspace
- Two lateral tripods: 2 × **Manfrotto 290 Xtra** ($240) — left/right Gemini 336 mounts
- ChArUco calibration target: **A2-size aluminum plate** with printed-and-bonded 7×5 ChArUco pattern, ~$60 (print at any sign shop, mount with 3M VHB)
- USB 3.0 active cables: 3 × 5 m ($90) — one per camera
- PoE switch + Cat6: **Netgear GS305P** ($60) — optional; only if you go PoE on cameras that support it; the Orbbec cameras here use USB so PoE isn't strictly needed
- Workspace lighting: 2 × **Godox SL-60W LED + softbox** ($300) — bigger workspace needs more uniform illumination
- Optional wrist cam: **Intel RealSense D405** ($330) or any small USB camera; mount via SmallRig clamps

**Do NOT skip:**
- Active USB cables for runs > 3 m. Passive cables drop frames silently — you discover it at training time and waste a week.
- A real diffuser-equipped LED light source. Direct sunlight or fluorescent flicker breaks IR-based depth cameras (Gemini 336 especially).

---

## 3. The two-tier strategy (why both?)

Reviewers + funders consistently respond to: *"works on cheap hardware"* + *"scales to research-grade hardware"*. Doing only the high-end version reads as "rich-lab demo"; doing only the low-end reads as "toy". Together they make a defensible deployment story.

**Concretely:**
- Tier L is the **rapid-iteration** track: cheap, low-friction, lets us validate sim-to-real pipelines and edge-deployment quickly.
- Tier H is the **research-quality** track: precision tasks, multi-camera 3D, full-scale VLA models. This is where the architectural contributions (revived voxel-fusion with REAL depth — see §6) live.

---

## 4. Camera placement + calibration guide

The geometric correctness of every voxel/3D experiment depends on this. The
LIBERO-Plus failure mode (silently-wrong agentview xmat → EEF projected to
v=−2180, see PROJECT_STATUS row 2h) is exactly the class of bug that bites
hardest in the real world too.

### 4.1 Tier L camera placement (SO-ARM101)

```
                    ▲ Overhead Gemini 336
                    │   ~50 cm above workspace centre
                    │   pitched 90° (straight down)
                    │   color + depth aligned
                    ▼
      ┌─────────────────────────────┐
      │     20 cm × 30 cm workspace │
      │                             │
      │  [SO-ARM101 base + arm]     │
      └─────────────────────────────┘
                                     ╲  Optional wrist cam ($30 webcam)
                                      ╲  mounted on gripper bracket
                                       ▶ pointing 30° downward toward TCP
```

**Why this layout:**
- A single overhead RGB-D gives reliable, calibrated workspace 3D coverage.
- Wrist cam is RGB-only because its extrinsics change every frame — voxel-fusion
  math gets noisy fast. The wrist view contributes per-grasp detail that the
  policy learns to use *separately*, not as part of the 3D fusion stream.
- Workspace ≤ 20×30 cm keeps every point inside the Gemini 336's reliable
  depth range (0.15–3 m, sweet spot 0.3–1.5 m).

### 4.2 Tier H camera placement (Panthera-HT)

```
                    ▲ Femto Bolt overhead
                    │   ~1 m above workspace centre
                    │   straight down, 90° pitch
                    │   Femto Bolt has best ToF quality for fixed depth
                    ▼
       ◀──┐                                   ┌──▶
   Gemini  │   ╔═══════════════════════════╗  │  Gemini
   336    │   ║   80 cm × 60 cm workspace ║  │  336
   left ◀──┤   ║                           ║  ├──▶ right
   45°    │   ║   [Panthera-HT base]      ║  │   45°
   above ──┘   ╚═══════════════════════════╝  └── above
   ~50 cm                                              ~50 cm
   from edge                                           from edge
   pitched 30° down toward centre                      same
```

**Why this layout:**
- Three rigid cameras at 90°-ish angular separation give the voxel grid three
  independent depth observations per workspace point. TSDF fusion fills the
  specular/transparent holes that any single camera will have.
- Lateral cameras at 45° elevation see the *side* of objects that the overhead
  camera doesn't — important for grasp planning on tall objects.
- Femto Bolt (ToF) overhead is more accurate than Gemini 336 (stereo) on
  textureless surfaces (clean tables, white objects).
- All three cameras are RIGID-MOUNTED on tripods. No wrist cam in the fusion
  stream — add as a separate RGB feed if needed.

### 4.3 Coordinate frames you must keep straight

You will get this wrong at least once. Document it:

| Frame | Definition | Used by |
|---|---|---|
| `world` | A point you pick on the table (e.g. front-right corner) | All extrinsics; the voxel grid sits here |
| `base` | Robot base link as defined by URDF | IK, FK, robot commands |
| `cam_<name>` | Per-camera OpenCV convention (X right, Y down, Z forward) | Image projection |
| `gripper` | TCP (tool centre point) at the gripper fingers' contact point | Action targets, EEF state |

You need at minimum:
- `T_base_to_world` — usually identity if you set world = base
- `T_cam_to_world` for each fixed camera (extrinsics — this is what `derive_libero_extrinsics_batch` was doing in sim, but for real cameras it comes from calibration)
- `T_gripper_to_base` — comes from FK, varies per timestep

### 4.4 ChArUco calibration recipe

A ChArUco board is a chessboard with ArUco markers in the white squares. It
gives sub-pixel corner accuracy plus marker IDs (so partial occlusion still
works). This is the standard for multi-cam extrinsic calibration in 2026 and
is supported out-of-the-box by OpenCV's `cv2.aruco` and Orbbec's SDK.

**One-time setup:**
1. Generate the board: 7×5 squares, ArUco dictionary `DICT_5X5_50`, square size
   40 mm (so the board is ~28×20 cm — fits in workspace and is visible from
   all cameras simultaneously). Print on rigid foam board or mount printed
   paper on aluminum plate with 3M VHB tape.
2. Place the board flat on the workspace. **Mark the corner you'll call the
   world origin** (e.g. front-right corner of the board = (0, 0, 0)).

**Per-session calibration (do this whenever a camera moves more than a few mm):**
3. For each camera, capture ~20 RGB frames with the board in different
   poses (translated + tilted across the workspace).
4. Run `cv2.aruco.calibrateCameraCharuco` to get intrinsics + per-frame
   board-to-cam transforms.
5. From the frame where the board sits flat on the workspace at the world
   origin, extract `T_board_to_cam` — this IS `T_world_to_cam` if you set
   world = board's marked corner.
6. **Hand-eye calibration** to align robot base with the world frame: hold
   the ChArUco board in the gripper, move to 20+ poses, run
   `cv2.calibrateHandEye` with `CALIB_HAND_EYE_DANIILIDIS` method to get
   `T_base_to_world` (or directly `T_base_to_cam` if you prefer to skip
   the world frame).
7. **Sanity check** (do not skip): print a different ChArUco target, place
   it at known world coordinates, project through your computed extrinsics
   + intrinsics, check that the corners land within 2 px of the expected
   pixel locations. **If this fails, your calibration is wrong.** Our
   LIBERO-Plus xmat bug was exactly this kind of silent-bad-calibration
   that no synthetic test caught.

**Target accuracy:**
- Intrinsics: reprojection error < 0.5 px
- Per-cam extrinsics: < 3 mm translation, < 0.3° rotation
- World ↔ base: < 5 mm
- Total end-to-end (workspace point → projected pixel): < 5 px in a 1280×800 image

If you can't hit those numbers, voxel-fusion will silently fail.

### 4.5 Drift verification (weekly)

Cameras get bumped. Tripods sag. Temperature changes shift focal length on
some camera models. Build a one-button drift check:

1. Each Monday morning, place the calibration board at a fixed reference pose.
2. Re-detect the board in each camera.
3. Compute the board's world pose using the current saved extrinsics.
4. Diff against the reference pose from the last calibration.
5. If translation diff > 5 mm or rotation diff > 1°, re-calibrate.

Log the drift to a CSV. The pattern of drift teaches you which cameras
need re-mounting (sagging tripod) vs which ones are stable.

### 4.6 References for calibration

- **ChArUco original paper**: Garrido-Jurado et al., *Generation of fiducial marker dictionaries using mixed integer linear programming*, Pattern Recognition 2016 ([paper](https://www.sciencedirect.com/science/article/abs/pii/S0031320315003544)).
- **OpenCV tutorial**: [calibrateCameraCharuco docs](https://docs.opencv.org/4.x/da/d13/tutorial_aruco_calibration.html) — copy-pasteable Python.
- **Hand-eye calibration methods**: Tsai-Lenz 1989 (`CALIB_HAND_EYE_TSAI`),
  Park-Martin 1994, **Daniilidis 1999 (`CALIB_HAND_EYE_DANIILIDIS` — recommended default)**.
  See [OpenCV calibrateHandEye](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#gaebfc1c9f7434196a374c382abf43439b).
- **Multi-camera bundle adjustment** (for higher accuracy when you have many cameras): [Kalibr](https://github.com/ethz-asl/kalibr) is the standard open-source tool, originally for VI-SLAM but works for multi-cam-only.
- **Recent best-practice survey for robot manipulation**: Allan et al. 2023 *A practical guide to multi-camera calibration for robot manipulation*.

---

## 5. Phased plan with publishable milestones

Each phase is sized at ~10–12 weeks and ends with a concrete deliverable. The
phases run mostly sequential because hardware learning compounds, but data
collection from earlier phases continues feeding the later ones.

### Phase 0 — Foundation (weeks 0–4)

**Goal:** every piece of hardware works, every camera is calibrated, every arm can be teleoped, every observation can be saved in lerobot dataset format.

| Subtask | Outcome |
|---|---|
| 0.1 SO-ARM101 + Orin Nano: assemble, calibrate, teleop via SpaceMouse/leader-arm | Tier L hardware works |
| 0.2 Panthera-HT or OpenArm-X integration: ROS2 bringup, joint-position+torque control, gripper interface | Tier H arm works |
| 0.3 Jetson Thor flashing + ROS2 + lerobot inference smoke test | Edge compute works |
| 0.4 Orbbec multi-cam calibration: intrinsics + extrinsics to robot base via ChArUco | All cams in a single world frame, < 5 mm error |
| 0.5 lerobot dataset capture format: extend `LeRobotDataset` schema with per-cam depth + extrinsics + intrinsics | Data pipeline ready |
| 0.6 IsaacSim digital twin of each arm + camera setup | Sim-to-real bridge ready |

**Publishable artifact:** Technical report / blog post — *"Open-source 5-camera RGB-D capture rig for sub-$5k research arms."* Workshop-grade.

### Phase 1 — Tier L baseline (weeks 4–14)

**Goal:** train and deploy a baseline VLA on SO-ARM101 / Orin Nano. Establish what "works at all" looks like.

| Subtask | Outcome |
|---|---|
| 1.1 Collect 200–500 teleop demos for 5 simple tasks (pick cube, stack, push, drawer-open, button-press) | Real-world dataset |
| 1.2 Train ACT on the dataset (no voxel, no depth — RGB-only baseline) | Baseline success rate per task |
| 1.3 Distill to Orin Nano: quantize to INT8 + TensorRT export | Edge-deployable model |
| 1.4 On-robot eval: 30 episodes per task, measure success + latency + failure modes | Tier L performance table |
| 1.5 Repeat 1.1–1.4 with depth as 4th channel (RGB-D ACT) | Depth's marginal contribution measured |

**Publishable artifact:** ICRA/IROS short paper — *"VLA distillation for sub-$1k robotic arms: an empirical baseline."* Real numbers on a real cheap arm is genuinely under-published.

### Phase 2 — Sim-to-real bridge + voxel fusion REVIVAL (weeks 14–26)

**Goal:** generate large IsaacSim datasets with the exact camera config of our real setup, train policies in sim, transfer to real arm. **This is where voxel-fusion gets a second chance — with REAL DEPTH from real cameras, not just synthetic projections from RGB.**

Why voxel-fusion might actually work this time (unlike LIBERO-Plus):
- **Depth is genuinely new information** that PaliGemma never saw during pretraining. Adding a depth-conditioned voxel module is no longer "redundant capacity."
- **Multi-view stereo from physically separated Orbbec cameras** gives stronger geometric constraints than the single-agentview LIBERO setup.
- **Calibration is reproducible and trustworthy** — we control the rig, vs LIBERO's hardcoded constants we discovered were broken.
- **The benchmark perturbations match the architecture**: real-world camera shift + lighting change are exactly what 3D-grounded representations should resist.

| Subtask | Outcome |
|---|---|
| 2.1 IsaacSim digital-twin scene generation with domain randomization (lighting, background textures, object poses) | Sim dataset with extrinsics, depth, segmentation, perfect GT |
| 2.2 Cosmos / SAM2 / depth-foundation-model integration for sim-to-real visual gap reduction | Optional, evaluate |
| 2.3 **Voxel-fusion v2**: 3D voxel grid from real RGB-D, encoded via 3D-DA-style architecture (NOT bolt-on; voxel features routed directly to action expert) | New architecture |
| 2.4 Comparison: ACT-RGB-D (Phase 1) vs PI0-Voxel-v2 (Phase 2.3) on same Tier L task suite | Direct sim-to-real measurement of voxel contribution |
| 2.5 Real-world eval with camera-pose perturbation (move overhead Orbbec by ±5 cm / ±10°): test robustness | Sensor-robustness claim measured on real hardware |

**Critical design decision in 2.3**: don't repeat the LIBERO-Plus mistake. The voxel module must satisfy at least ONE of:
- **Architectural constraint**: 3D-action-head (PerAct-style) — actions are voxel-grid coordinates, geometry is structurally required
- **Auxiliary supervision**: occupancy / depth-prediction loss forces voxel features to be geometrically meaningful (our Road B-5 idea applied to real data)
- **Dual-system routing**: voxel features route into action expert directly (Road B-4 / PointACT)

Without one of these, voxel-fusion will fail the same way it did on LIBERO-Plus.

**Publishable artifact:** CoRL/RSS-class paper — *"3D voxel-fusion for sim-to-real VLA: when does geometry help?"* This is the strongest paper-shaped contribution in the plan because it directly contrasts our negative LIBERO-Plus findings with positive real-hardware findings (if they materialize) and provides mechanistic explanation.

### Phase 3 — Tier H scale-up + multi-task VLA (weeks 26–40)

**Goal:** scale to Panthera-HT (or OpenArm-X) with full π0-scale models on Jetson Thor. Multi-task evaluation.

| Subtask | Outcome |
|---|---|
| 3.1 Port Phase 2 architecture to Panthera-HT | Tier H deployment |
| 3.2 Collect 1000+ demos across 20+ tasks on Tier H | Diverse real-world dataset |
| 3.3 Fine-tune π0 or pi0_5 with our voxel-v2 architecture | Tier H VLA |
| 3.4 Deploy to Jetson Thor with TensorRT optimization | Edge inference at scale |
| 3.5 Multi-task eval: known tasks + unseen tasks (language generalization) | VLA paper-quality numbers |

**Publishable artifact:** Top-tier paper — *"On-arm π0-scale VLA inference: deployment lessons from a multi-camera factory testbed."* Cite GR1, RDT, Aloha-Unleashed as comparable deployment works.

### Phase 4 — Factory POC (weeks 40+)

**Goal:** demonstrable factory task with measurable throughput and reliability.

| Subtask | Outcome |
|---|---|
| 4.1 Pick a real factory partner (or simulated factory scenario) — e.g., electronic component sorting, kit assembly | Concrete deliverable |
| 4.2 Long-horizon task chaining via LLM-based task planning | Hierarchical control |
| 4.3 Failure recovery + human-in-the-loop intervention pipeline | Production-grade reliability |
| 4.4 8-hour autonomous operation test with throughput + failure logging | Deployment metrics |

**Publishable artifact:** Industry-track paper / preprint — *"From sim to factory: a 6-month case study deploying VLA-based manipulation."* Strong for ICRA industry track or robotics journal.

---

## 6. Where voxel-fusion fits (the revival case)

Our LIBERO-Plus experience said: **bolt-on voxel-fusion on a frozen VLM doesn't help.**

Our deployment-context hypothesis says: **voxel-fusion CAN help when (a) the depth is real and not encoded in the VLM's pretraining distribution, AND (b) the voxel features are architecturally load-bearing, not bolt-on.**

The two conditions match Phase 2 above. Concrete architecture for Phase 2.3:

```
                    ┌─────────────────────────┐
                    │ Orbbec multi-cam RGB-D  │
                    │ (Femto Bolt overhead +  │
                    │  2× Gemini 336 lateral) │
                    └────────────┬────────────┘
                                 │
        ┌────────────────────────┼────────────────────────┐
        │                        │                        │
        ▼                        ▼                        ▼
┌──────────────┐          ┌─────────────┐          ┌──────────────┐
│ RGB → PaliGm │          │ Depth + cam │          │ Aux occupancy│
│ (frozen VLM) │          │ extrinsics  │          │ head (B-5)   │
└──────┬───────┘          └──────┬──────┘          └──────┬───────┘
       │                         │                        │
       │                  ┌──────▼──────┐                 │
       │                  │ 3D voxel    │                 │
       │                  │ unproject + │◀────────────────┘
       │                  │ encoder     │
       │                  └──────┬──────┘
       │                         │
       └───────────┐    ┌────────┘
                   ▼    ▼
              ┌──────────────────┐
              │ Cross-attention  │  ← voxel tokens enter HERE
              │ Action Expert    │     (NOT bolted to LLM prefix)
              │ (gemma_300m)     │     This is Road B-4 (PointACT).
              └────────┬─────────┘
                       │
                       ▼
              ┌──────────────────┐
              │ Diffusion head   │
              │ → action (7-d)   │
              └──────────────────┘
```

Three differences from the failed monolithic-prefix version:
1. **Voxel input is REAL depth** from Orbbec cameras, not synthetic projections.
2. **Voxel features route to action expert directly** (dual-system, B-4 style).
3. **Auxiliary occupancy / state-prediction loss** (B-5 style) forces voxel features to be geometric.

This composes the architectural lessons from our LIBERO-Plus failure into a setup where they should pay off.

---

## 7. Sim vs real data — what works and what doesn't

This section captures the painful lesson from `IsaacSim/isaac_auto_collector_v6.py`:
scripted-IK sim trajectories failed every model we trained on them. This is a
real phenomenon, not a bug, and it determines how to allocate effort going forward.

### 7.1 Why scripted-IK sim data fails for action policies

What happens when you collect demos by running an IK-solver script:

- The IK solver picks the *minimum-energy* path: straight lines, smooth Bézier
  interpolation, no overshoots
- Velocity profiles are trapezoidal (accel → cruise → decel) — humans never do this
- No regrasps, no hesitations, no recovery from near-misses
- **Every trajectory for the same start/goal looks essentially identical**

Behavioral cloning on this dataset learns a **degenerate δ-distribution policy**:
- It memorizes "from state X, output action Y" with near-zero variance
- At deployment, any perturbation (camera shift, object 5 mm off) puts the
  policy off its training manifold → catastrophic failure
- This is the same mode-collapse problem the GAIL/MaxEntIRL literature has
  documented since 2016

**Every successful real-robot VLA paper (ALOHA, π0, Aloha-Unleashed, RDT,
OpenVLA, RT-2) uses human teleop data, not scripted IK.** Human teleop has
irreplaceable variation: approach from slightly different angles each time,
mid-trajectory corrections, multiple grasp strategies for the same object.

### 7.2 The IsaacSim scope cut

| IsaacSim role | Verdict | Reason |
|---|---|---|
| Action policy training data (scripted IK) | **Stop.** | Doesn't work. Don't keep trying. Three months of negative evidence. |
| Visual encoder SSL pretraining (rendered images, no action loss) | Keep | Works for representation learning; standard recipe |
| Voxel module pretraining (perfect depth + occupancy supervision) | Keep | Aux loss is dense, GT is perfect — exactly what sim is good for |
| Domain randomization at training time | Keep | Augmentations only, no action loss |
| Sim-only ablation experiments (calibration sensitivity, lighting, etc.) | Keep | Useful for paper figures; impossible to control in real |
| Procedural object dataset for visual pretraining | Keep | Random ObjaVerse objects in random poses |
| Sim-to-real *fine-tuning* of action policy | Last-resort only | Investigate only if real data is insufficient |

**Effort budget**: ~15% of total project effort on IsaacSim, not 50%. Spend
80%+ on real teleop data collection + on-arm deployment iteration.

### 7.3 What ARE the proven recipes for real-robot VLA data?

The pattern across recent successful papers:

1. **Pretrain on real data at scale**: Open-X-Embodiment (1M+ trajectories),
   DROID (76k), Bridge (60k). The visual encoder + action expert see internet-scale
   robot variation.
2. **Fine-tune on small task-specific real data**: 50–500 teleop demos per task,
   collected on YOUR hardware with YOUR cameras.
3. **Augment with synthetic perturbation at training time**: not synthetic
   trajectories — perturbations applied to real frames (random crops, color
   jitter, camera-pose jitter, depth dropout, language paraphrasing).

Sim is useful as **(a) representation pretraining** and **(b) the controlled
ablation environment**, NOT as the source of training trajectories.

### 7.4 Data collection ratio for our project

| Source | Role | Volume target |
|---|---|---|
| Open-X-Embodiment / DROID | Pretraining (visual + action) | Use existing pretrained π0 / OpenVLA-OFT weights; don't re-pretrain |
| Tier L real teleop (SO-ARM101) | Task-specific fine-tune | 500–1000 demos across ~5 tasks |
| Tier H real teleop (Panthera-HT) | Precision-task fine-tune | 500 demos across ~10 tasks |
| IsaacSim — visual pretraining only | Voxel encoder pretraining | 100k frames; no action loss |
| IsaacSim — domain randomization | Augmentation at training time | Generated on-the-fly during training |
| Internet video (Bridge / video models) | Backup for language grounding | Use as features, don't fine-tune on |

---

## 8. Depth integration strategy — leveraging Orbbec depth in four tiers

Multiple ways to inject Orbbec depth into a policy. Ranked by effort and risk.

### Tier 1 — RGB-D as 4-channel input (minimum effort, day-1)

Stack depth as the 4th channel on each RGB frame. Normalize depth to [0,1] by
dividing by max workspace depth (e.g. 1 m for tabletop). Mask invalid pixels
(depth == 0) by zeroing or filling with median.

**Cost**: 1 day to wire up. Tells you immediately whether depth as raw input
helps the existing policy class (ACT, π0). Use as Phase 1 baseline.

### Tier 2 — TSDF voxel fusion across multiple cameras (medium effort)

Standard CV recipe:
1. For each frame, take all Orbbec RGB-D + extrinsics
2. Unproject each cam's depth to point cloud in world frame
3. Fuse into a single TSDF (truncated signed distance function) volume on a
   100³ grid (workspace bounds)
4. Run a small 3D conv encoder → voxel tokens
5. Route tokens to action expert via cross-attention (Road B-4 dual-system —
   NOT bolted to LLM prefix; that failed in LIBERO-Plus)

**Cost**: ~2 weeks. This is Phase 2 of the deployment plan and the main
architectural contribution.

### Tier 3 — 3D-action-head (high effort)

Per PerAct/RVT/3D Diffuser Actor. Actions predicted in the voxel grid
(per-voxel translation + rotation + gripper logits). Geometry is structurally
load-bearing because the action target lives in 3D space.

**Cost**: ~4–6 weeks. Risk of re-implementing PerAct, but with the upside
that geometry is forced to matter.

### Tier 4 — Pretrained 3D foundation models (research bet)

Use Depth-Anything-v3, VGGT, or Spatial-VLA's depth modules as depth refiners
or as additional 3D feature extractors. Stack on top of Tier 2 or Tier 3.

**Cost**: open-ended. Save for Phase 3+.

### Recommended sequence

Tier 1 (Phase 1, ACT-RGB-D baseline) → Tier 2 (Phase 2, voxel-fusion revival)
→ Tier 3 (only if Tier 2 insufficient).

### Orbbec-specific gotchas to handle in code

- **Specular surfaces** (metal, glass) → depth holes. Fill via OpenCV
  `cv2.inpaint` or mask.
- **Black or IR-absorbing objects** → depth holes. Same fix.
- **Sunlight** → Gemini 336 IR saturation completely breaks depth. Eliminate
  sunlight in workspace; rely on the diffused LED panels from the parts list.
- **Sync drift across multiple cameras** → use Orbbec SDK hardware sync where
  available; otherwise log timestamps and interpolate to nearest 33 ms.
- **Femto Bolt's RGB is 4K but depth is only 320×288** → downsample RGB to
  depth resolution; don't try to align at RGB resolution.

---

## 9. Bridging 2D-only VLAs (π0, OpenVLA) to 3D inputs

π0 and most pretrained VLAs take RGB + state + language. None take depth or
voxels natively. Four options for injecting our 3D stream:

### Option A — RGB-D as 4th channel
- Change `observation.images.X` from `(3, H, W)` to `(4, H, W)`.
- Replace the SigLIP patch embedding's `Conv2d(3, hidden, ...)` with `Conv2d(4, ...)`.
- Initialize the depth-channel weights either via duplication-from-RGB or zero-init.
- **Pro**: minimal code change, lets pretrained features still flow.
- **Con**: SigLIP was pretrained on 3-channel; 4th channel must be learned from scratch.

### Option B — Separate depth/voxel processing, concat to PaliGemma prefix
- Run depth through a small encoder; concat the resulting tokens to PaliGemma's
  image tokens before the LLM.
- **Pro**: keeps RGB path untouched.
- **Con**: **This is what we just showed fails in LIBERO-Plus.** Don't do this
  unless paired with the architectural fixes from §6 (action-frame or aux loss).

### Option C — Dual-system routing (Road B-4 / PointACT)
- Voxel features go directly to the action expert via cross-attention,
  bypassing the LLM entirely.
- **Pro**: PointACT validated this exact fix for our exact failure mode.
- **Con**: more invasive — requires modifying lerobot's
  `paligemma_with_expert.expert` internals.

### Option D — Use a 3D-native VLA from the start
- 3D Diffuser Actor, RVT-2, or PerAct as the policy class. Skip π0.
- **Pro**: cleanest architecture for our problem.
- **Con**: abandons the pretrained PaliGemma + action-expert advantages of π0.

### Recommended sequence for this project

| Phase | Option |
|---|---|
| Phase 1 (Tier L baseline) | Option A — fastest path to "does depth help at all?" |
| Phase 2 (voxel revival) | Option C (dual-system on π0) — main architectural contribution |
| Phase 2 fallback if C fails | Option D (3D Diffuser Actor) — backup architecture for the same paper |
| Don't bother with | Option B alone — empirically refuted by our own work |

---

## 10. Data collection strategy

A real-robot manipulation paper lives or dies by data. Five-track strategy
(consistent with §7's IsaacSim scope cut):

### Track 1 — Tier L teleop (continuous, all phases)
- SpaceMouse or leader-arm teleop on SO-ARM101
- ~10 demos/day for 100 days = 1000 demos minimum
- Stored in lerobot dataset format with all modalities (RGB, depth, joint, gripper, extrinsics, intrinsics)

### Track 2 — Tier H teleop (Phase 3+)
- Higher-quality demos on Panthera-HT
- Slower rate (~3 demos/day) but precision-task focused
- Target 500 demos by end of Phase 3

### Track 3 — IsaacSim (NARROWED scope per §7)
- **Visual encoder + voxel module pretraining only.** No action policy training.
- 100k+ rendered frames; no scripted-IK trajectories.
- Goal: pretrain the voxel encoder so it sees diverse object/light/material variation before it ever sees real data.

### Track 4 — Internet video (Phase 2+)
- Open-X-Embodiment + DROID + Bridge as auxiliary pretraining data via existing pretrained π0 / OpenVLA-OFT checkpoints — don't re-pretrain from scratch.
- Not for fine-tuning to our specific arm but for visual / language grounding.

### Track 5 — Synthetic perturbation at training time
- Augmentations applied to real-world datasets during training, not separate datasets.
- Camera-pose jitter, depth dropout, lighting jitter, language paraphrasing.
- Goal: instill robustness without per-perturbation real-world data collection.

---

## 11. Risk register

| Risk | Likelihood | Mitigation |
|---|---|---|
| Orbbec depth quality on metallic/reflective objects | High | Fall back to RGB-only voxel for affected tasks; document failure modes |
| Multi-cam calibration drift over time | Medium | Daily ChArUco recalibration; log drift |
| Jetson Thor / Orin Nano inference latency too slow | Medium | Aggressive model distillation; consider Pi0-FAST or smaller VLAs |
| Sim-to-real gap too large despite domain randomization | High | RealNVP / Cycle-GAN domain adaptation; or skip sim-to-real and go teleop-only |
| Panthera-HT integration drags > 1 month | Medium | Pivot to OpenArm-X or UR5e as backup |
| Voxel-fusion in Phase 2 still doesn't help vs RGB-D ACT baseline | Medium | Ablation + negative-result paper; pivot architecture to pure 3D-action-head (PerAct-style) |
| Factory partner doesn't materialize | High | Simulate factory scenario in IsaacSim; demonstrate at industry trade show instead |

---

## 12. Publication cadence

| Month | Venue | Topic |
|---|---|---|
| 2–3 | Workshop (CoRL / NeurIPS / RSS RC) | Negative result: limits of bolt-on 3D fusion in VLA (from LIBERO-Plus work) |
| 4–5 | ICRA / IROS late deadline | Phase 1 — VLA distillation to sub-$1k arms |
| 8–10 | CoRL / RSS | Phase 2 — voxel-fusion with real depth, sim-to-real evaluation |
| 12–14 | NeurIPS / CoRL | Phase 3 — π0-scale on-arm inference, multi-task VLA on Panthera-HT |
| 16+ | Industry track / journal | Phase 4 — factory deployment case study |

**5 publications across 18 months** if executed well. Even if Phase 2 voxel-fusion fails (the highest-risk milestone), we still have 3+ publications from deployment lessons + negative results.

---

## 13. Concrete next 30 days

While Direction A (Road B-4 / B-5) finishes on the H100, **start Phase 0 in parallel** on local hardware:

1. **Week 1** — Order Orbbec Femto Bolt + 2× Gemini 336 (if not already on hand). Assemble SO-ARM101.
2. **Week 2** — Calibrate Orbbec rig with ChArUco. Verify lerobot dataset format extensions for RGB-D + extrinsics save/load.
3. **Week 3** — SO-ARM101 teleop pipeline: SpaceMouse or leader-arm. First 10 pick-and-place demos collected end-to-end.
4. **Week 4** — Single-task ACT training on the 10 demos. Edge deployment to Orin Nano. Smoke test.

This unblocks Phase 1 without waiting for the LIBERO-Plus work to finish.

---

## 14. Open design decisions (decide before Phase 2)

These need a concrete answer before Phase 2 kicks off:

1. **Action representation**: dense 7-d (current π0 style) vs keyframe-action (PerAct style)? Dense is more general; keyframe is more sample-efficient. Probably dense, but worth confirming.
2. **Camera count at inference**: 3 (Femto Bolt + 2× Gemini) vs 5 (add 2 wrist cams)? More cams = better voxel but bigger bandwidth.
3. **VLA backbone**: π0 (lerobot port, proven) vs π0-FAST (faster inference) vs OpenVLA-OFT (more SOTA but heavier) vs custom-distilled?
4. **Sim-to-real strategy**: pure sim training + real fine-tune, vs co-training on sim + real data simultaneously? Recent literature mixed.
5. **How to handle Sim2Real depth gap**: Orbbec depth has different noise model than IsaacSim depth. Sim2real domain randomization on depth (add noise, holes, etc.) probably needed.

These deserve a separate decision memo before committing engineering time to Phase 2.

---

## 15. Related works & references

Curated, organized by section. Each entry should be readable in 1 sentence so
the reader can decide whether to dig in.

### 15.1 Voxel / 3D-aware manipulation policies

- **PerAct** — Shridhar et al., *Perceiver-Actor: A Multi-Task Transformer for Robotic Manipulation*, CoRL 2022. Voxel grid + Perceiver-IO + 3D action head. The original "predict actions in voxel space" architecture. [arXiv 2209.05451](https://arxiv.org/abs/2209.05451), [code](https://github.com/peract/peract).
- **RVT** — Goyal et al., *RVT: Robotic View Transformer for 3D Object Manipulation*, CVPR 2023. Orthographic multi-view projection instead of full 3D voxel — much faster, comparable accuracy. [arXiv 2306.14896](https://arxiv.org/abs/2306.14896), [code](https://github.com/NVlabs/RVT).
- **3D Diffuser Actor** — Ke et al., *3D Diffuser Actor: Policy Diffusion with 3D Scene Representations*, ICRA 2024. SOTA on RLBench-18 (81.3% vs PerAct's 49.4%). [arXiv 2402.10885](https://arxiv.org/abs/2402.10885), [code](https://github.com/nickgkan/3d_diffuser_actor).
- **3D FlowMatch Actor** — Gkanatsios et al. 2024. Flow matching variant of 3D Diffuser Actor; cleaner code. [project page](https://3d-flowmatch-actor.github.io/).
- **PointACT** — Liu et al., *Point-based 3D Visual Representation for Few-shot Manipulation*, 2024. **Documents our exact failure mode** (monolithic 3D-token-on-VLM fails) and the dual-system fix. The reason we chose Road B-4. [arXiv 2605.21414](https://arxiv.org/abs/2605.21414).
- **OC-VLA** — Yang et al., *OC-VLA: Grounding Actions in Camera Space for Viewpoint Generalization*, 2024. Camera-frame action prediction; forces extrinsics to matter. The alternative to dual-system. [arXiv 2508.13103](https://arxiv.org/abs/2508.13103).
- **PoseVLA** — Lin et al., *PoseVLA: 6D Pose Aware Vision-Language-Action Model*. Adds discretized 6D pose tokens to PaliGemma. [project page](https://hetolin.github.io/PoseVLA/).

### 15.2 VLA architectures for real-robot deployment

- **π0 / Pi0** — Black et al., *π₀: A Vision-Language-Action Flow Model for General Robot Control*, Physical Intelligence 2024. The current strongest open VLA. PaliGemma + gemma_300m action expert. [paper](https://www.physicalintelligence.company/blog/pi0).
- **π0-FAST** — Pertsch et al. 2024. Faster inference via action tokenization. [openpi repo](https://github.com/Physical-Intelligence/openpi).
- **π-0.5** — PI 2025. Successor to π0 with longer-horizon planning.
- **OpenVLA** — Kim et al., *OpenVLA: An Open-Source Vision-Language-Action Model*, CoRL 2024. 7B Llama2 backbone, fully open. [arXiv 2406.09246](https://arxiv.org/abs/2406.09246), [code](https://github.com/openvla/openvla).
- **OpenVLA-OFT** — Kim et al. 2025. Optimized fine-tuning recipe; current SOTA on LIBERO-Plus Camera column (56.4%). [paper](https://openvla-oft.github.io/).
- **RDT-1B** — Liu et al., *RDT-1B: a Diffusion Foundation Model for Bimanual Manipulation*, 2024. Diffusion transformer for ALOHA-class bimanual setups. [arXiv 2410.07864](https://arxiv.org/abs/2410.07864), [code](https://github.com/thu-ml/RoboticsDiffusionTransformer).
- **ALOHA / Aloha-Unleashed** — Zhao et al., 2023+. Bimanual teleop + ACT policy class. The "minimum viable real-robot research" reference design. [project page](https://tonyzhaozh.github.io/aloha/).
- **GR00T-N1** — NVIDIA 2025. Humanoid VLA with built-in world-model elements. Targets Jetson Thor edge inference — directly relevant to Tier H. [GR00T site](https://developer.nvidia.com/isaac/gr00t).

### 15.3 Pretraining datasets for VLAs

- **Open-X-Embodiment (OXE)** — *Open X-Embodiment: Robotic Learning Datasets and RT-X Models*, 2023. 1M+ trajectories across 22 embodiments. The de-facto pretraining set for modern VLAs. [paper](https://arxiv.org/abs/2310.08864), [data](https://robotics-transformer-x.github.io/).
- **DROID** — Khazatsky et al., *DROID: A Large-Scale In-The-Wild Robot Manipulation Dataset*, RSS 2024. 76k diverse demos across many labs. [arXiv 2403.12945](https://arxiv.org/abs/2403.12945).
- **Bridge / BridgeV2** — Walke et al., 2023. 60k demos on WidowX; cheap arm = relevant to Tier L. [project page](https://rail-berkeley.github.io/bridgedata/).

### 15.4 Sim-to-real + domain randomization

- **Domain Randomization (origin)** — Tobin et al., *Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World*, IROS 2017. [arXiv 1703.06907](https://arxiv.org/abs/1703.06907).
- **OpenAI Dactyl** — Andrychowicz et al. 2019. Scaled DR to in-hand cube manipulation. The "DR can transfer" existence proof. [arXiv 1808.00177](https://arxiv.org/abs/1808.00177).
- **Why scripted-IK sim data fails** — discussed in many places but most clearly in the GAIL / MaxEntIRL literature. Ho & Ermon 2016 originally; see also *Implicit Behavioral Cloning* (Florence et al., 2021) for the mode-collapse failure mode.

### 15.5 Depth processing + foundation models

- **TSDF fusion** — Curless & Levoy, *A Volumetric Method for Building Complex Models from Range Images*, SIGGRAPH 1996. The classical multi-view depth fusion algorithm; still the default for our Tier 2 voxel-fusion. [paper](https://graphics.stanford.edu/papers/volrange/volrange.pdf).
- **Open3D** — Zhou et al., 2018. Open-source TSDF + point cloud library. [docs](http://www.open3d.org/docs/release/tutorial/pipelines/rgbd_integration.html).
- **Depth Anything v2 / v3** — Yang et al., 2024+. Foundation model for monocular depth. Useful as depth refiner / sim2real bridge. [v2 paper](https://arxiv.org/abs/2406.09414).
- **VGGT** — Wang et al., *Visual Geometry Grounded Transformer*, 2024. Pretrained for multi-view geometry tasks. [arXiv 2503.11651](https://arxiv.org/abs/2503.11651).

### 15.6 Camera calibration

- **ChArUco** — Garrido-Jurado et al., *Generation of fiducial marker dictionaries using mixed integer linear programming*, Pattern Recognition 2016. The standard fiducial-marker board for camera calibration. [paper](https://www.sciencedirect.com/science/article/abs/pii/S0031320315003544).
- **OpenCV calibrateCameraCharuco** — [docs](https://docs.opencv.org/4.x/da/d13/tutorial_aruco_calibration.html).
- **Hand-eye calibration methods** — Tsai & Lenz 1989; Park & Martin 1994; **Daniilidis 1999** (dual-quaternion formulation — our recommended default). Implemented in OpenCV's `calibrateHandEye`.
- **Kalibr** — Furgale et al., *Unified Temporal and Spatial Calibration for Multi-Sensor Systems*, IROS 2013. The standard tool for multi-camera + IMU calibration. [code](https://github.com/ethz-asl/kalibr).

### 15.7 Edge deployment + distillation

- **TinyVLA** — Wen et al., 2024. Distilled small VLA for edge inference. Reference for what's achievable on Orin-class hardware. [arXiv 2409.12514](https://arxiv.org/abs/2409.12514).
- **TensorRT optimization for VLAs** — NVIDIA's [Isaac ROS](https://nvidia-isaac-ros.github.io/) provides reference TensorRT export paths.
- **Jetson AGX Thor benchmark numbers** — see [NVIDIA's official benchmarks](https://developer.nvidia.com/embedded/jetson-modules) for inference latency expectations.

### 15.8 World models for manipulation (longer-term explore-track)

- **Cosmos** — NVIDIA 2025. Large-scale generative video model for robotics. Potential visual encoder for VLAs. [Cosmos repo](https://github.com/NVIDIA/Cosmos).
- **DreamerV3** — Hafner et al. 2023. Model-based RL with latent world models. Reference for "world model + planner" pattern. [paper](https://arxiv.org/abs/2301.04104).
- **GR00T-N1 world-model components** — NVIDIA 2025. Humanoid-focused but shares architectural ideas.
- **DreamVLA** — 2025. VLA with predicted future-frame supervision. Empirical results modest so far.

### 15.9 Negative-result methodology

These should inform how we frame the LIBERO-Plus negative-result paper:
- **"Where are we in the search for an Artificial Visual Cortex for Embodied Intelligence?"** — Majumdar et al. 2023. Showed pretrained visual encoders barely beat random init on manipulation. The methodological template for our LIBERO-Plus null. [arXiv 2303.18240](https://arxiv.org/abs/2303.18240).
- **"Does Pretraining Help Embodied AI?"** — various papers; the answer is "less than you'd think on many benchmarks."

---

## Cross-references

- `PROJECT_STATUS.md` — historical experiments + negative results
- `ROAD_B_PLAN.md` — current H100 work (Road B-4 / B-5)
- `LIBERO_PLUS_GUIDE.md` — sim benchmark reference
- `SETUP.md` — software environment
- IsaacSim parallel track — RTX5090 commits (act_bev, act_cheat_noisy, multicam)
