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

---

## 3. The two-tier strategy (why both?)

Reviewers + funders consistently respond to: *"works on cheap hardware"* + *"scales to research-grade hardware"*. Doing only the high-end version reads as "rich-lab demo"; doing only the low-end reads as "toy". Together they make a defensible deployment story.

**Concretely:**
- Tier L is the **rapid-iteration** track: cheap, low-friction, lets us validate sim-to-real pipelines and edge-deployment quickly.
- Tier H is the **research-quality** track: precision tasks, multi-camera 3D, full-scale VLA models. This is where the architectural contributions (revived voxel-fusion with REAL depth — see §6) live.

---

## 4. Phased plan with publishable milestones

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

## 5. Where voxel-fusion fits (the revival case)

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

## 6. Data collection strategy

A real-robot manipulation paper lives or dies by data. Five-track strategy:

### Track 1 — Tier L teleop (continuous, all phases)
- SpaceMouse or leader-arm teleop on SO-ARM101
- ~10 demos/day for 100 days = 1000 demos minimum
- Stored in lerobot dataset format with all 4 modalities (RGB, depth, joint, gripper)

### Track 2 — Tier H teleop (Phase 3+)
- Higher-quality demos on Panthera-HT
- Slower rate (~3 demos/day) but precision-task focused
- Target 500 demos by end of Phase 3

### Track 3 — IsaacSim auto-collection (continuous, all phases)
- Scripted policy + domain randomization in IsaacSim
- Cheap to scale: 10,000+ demos overnight on RTX5090 / Pro 6000
- Used for: large-scale pretraining, sim-to-real bridge, voxel module pretraining
- Builds on existing IsaacSim multicam pipeline (already in progress on RTX5090)

### Track 4 — Internet video (Phase 2+)
- Open-X-Embodiment + DROID + Bridge as auxiliary pretraining data
- Not for fine-tuning to our specific arm but for visual / language grounding

### Track 5 — Synthetic perturbation (Phase 2+)
- LIBERO-Plus-style perturbations applied to our real-world datasets at training time
- Camera-pose jitter, depth dropout, lighting jitter, language paraphrasing
- Goal: instill robustness without per-perturbation real-world data

---

## 7. Risk register

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

## 8. Publication cadence

| Month | Venue | Topic |
|---|---|---|
| 2–3 | Workshop (CoRL / NeurIPS / RSS RC) | Negative result: limits of bolt-on 3D fusion in VLA (from LIBERO-Plus work) |
| 4–5 | ICRA / IROS late deadline | Phase 1 — VLA distillation to sub-$1k arms |
| 8–10 | CoRL / RSS | Phase 2 — voxel-fusion with real depth, sim-to-real evaluation |
| 12–14 | NeurIPS / CoRL | Phase 3 — π0-scale on-arm inference, multi-task VLA on Panthera-HT |
| 16+ | Industry track / journal | Phase 4 — factory deployment case study |

**5 publications across 18 months** if executed well. Even if Phase 2 voxel-fusion fails (the highest-risk milestone), we still have 3+ publications from deployment lessons + negative results.

---

## 9. Concrete next 30 days

While Direction A (Road B-4 / B-5) finishes on the H100, **start Phase 0 in parallel** on local hardware:

1. **Week 1** — Order Orbbec Femto Bolt + 2× Gemini 336 (if not already on hand). Assemble SO-ARM101.
2. **Week 2** — Calibrate Orbbec rig with ChArUco. Verify lerobot dataset format extensions for RGB-D + extrinsics save/load.
3. **Week 3** — SO-ARM101 teleop pipeline: SpaceMouse or leader-arm. First 10 pick-and-place demos collected end-to-end.
4. **Week 4** — Single-task ACT training on the 10 demos. Edge deployment to Orin Nano. Smoke test.

This unblocks Phase 1 without waiting for the LIBERO-Plus work to finish.

---

## 10. Open design decisions (decide before Phase 2)

These need a concrete answer before Phase 2 kicks off:

1. **Action representation**: dense 7-d (current π0 style) vs keyframe-action (PerAct style)? Dense is more general; keyframe is more sample-efficient. Probably dense, but worth confirming.
2. **Camera count at inference**: 3 (Femto Bolt + 2× Gemini) vs 5 (add 2 wrist cams)? More cams = better voxel but bigger bandwidth.
3. **VLA backbone**: π0 (lerobot port, proven) vs π0-FAST (faster inference) vs OpenVLA-OFT (more SOTA but heavier) vs custom-distilled?
4. **Sim-to-real strategy**: pure sim training + real fine-tune, vs co-training on sim + real data simultaneously? Recent literature mixed.
5. **How to handle Sim2Real depth gap**: Orbbec depth has different noise model than IsaacSim depth. Sim2real domain randomization on depth (add noise, holes, etc.) probably needed.

These deserve a separate decision memo before committing engineering time to Phase 2.

---

## Cross-references

- `PROJECT_STATUS.md` — historical experiments + negative results
- `ROAD_B_PLAN.md` — current H100 work (Road B-4 / B-5)
- `LIBERO_PLUS_GUIDE.md` — sim benchmark reference
- `SETUP.md` — software environment
- IsaacSim parallel track — RTX5090 commits (act_bev, act_cheat_noisy, multicam)
