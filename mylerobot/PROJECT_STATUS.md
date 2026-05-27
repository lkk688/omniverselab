# mylerobot — project status & setup

Last updated: 2026-05-25. Single source of truth for environment, datasets, code, and current state.

---

## 1. North-star research goal

**Sensor-agnostic, language-conditioned manipulation policy.** Headline claim is *robust to camera placement / configuration / failures*, tested on **LIBERO-Plus** (10,030-task perturbation benchmark). We don't chase LIBERO-standard SOTA (Libra-VLA already hits 97.2% there; it's saturated).

Two main contributions targeted for a CoRL/RSS/NeurIPS-class paper:

1. **3D-grounded perception via voxel-fusion + occupancy aux head.** Multi-camera RGB + per-camera extrinsics → 3D voxel grid via cross-attention (PyTorch port of user's BEVDet `cross_attn_lss2.py`). Optional Tesla-style UNet variant with depth-back-projection occupancy supervision. Output: voxel tokens injected into π0's PaliGemma prefix sequence alongside image + language tokens.
2. **Sensor-robustness experiments on LIBERO-Plus.** Train on the 14k-episode LIBERO-Plus training set; evaluate per-perturbation-dimension. Headline cell: Camera Viewpoints (the LIBERO-Plus paper shows existing VLAs drop from ~95% clean to ≤30% under camera perturbation — that's the gap our voxel-fusion targets).

Separate deployment subtrack (parallel, user-driven on RTX5090): IsaacSim Franka multi-camera data collection → distill to small ACT class → SO-ARM101 + Orin Nano hardware deployment.

---

## 2. Strategic decisions log (chronological)

| Date | Decision | Why |
|---|---|---|
| 2026-05-23 | Subclass, not fork, of lerobot | Minimize merge pain |
| 2026-05-23 | `mylerobot/` as a sibling package | Same reason |
| 2026-05-24 | IsaacSim Franka (RTX5090) for multi-cam *deployment* track | Scripted policy exists; photoreal; single-arm matches SO-ARM101 |
| 2026-05-24 | Pivot ACT → π0 as primary policy class | Top-venue ceiling; rich LIBERO baseline ecosystem |
| 2026-05-24 | lerobot PyTorch π0 port (not openpi/JAX) | Saves ~1.5 weeks of JAX porting; same env as everything else |
| **2026-05-25** | **Pivot research claim to sensor-robustness on LIBERO-Plus** | Libra-VLA's 97.2% saturated LIBERO-standard; LIBERO-Plus's Camera column (best SOTA 56%) is wide open |
| 2026-05-25 | Two evaluation tracks: (a) Sylvest/libero_plus_lerobot 2-cam training, (b) our re-rendered 5-cam multicam_v0 for sensor-dropout ablations | Both feed the same paper; one is mainstream, one is controlled |

---

## 3. Environment setup

Two conda envs, both on the H100 box (`/home/010796032/miniconda3/`):

### `py312` — primary working env

| | |
|---|---|
| Python | 3.12.10 |
| Path | `/home/010796032/.conda/envs/py312/bin/python` |
| Use for | π0 (lerobot port), ACT, voxel-fusion, mylerobot, LIBERO-Plus eval — everything PyTorch |
| Key deps | torch 2.10+cu128, lerobot 0.4.4 (editable @ `/data/rnd-liu/aiprojects/lerobot`), gym_aloha, mujoco 3.5.0, libero (LIBERO-Plus), robosuite 1.4, wand, imagemagick (conda) |
| Known issues | `lerobot.policies.groot.groot_n1.GR00TN15Config` clashes with `huggingface_hub 1.7.1` `@strict`. Patched by `mylerobot._env_shim` (must import `mylerobot` *before* any `lerobot.*`). |
| Removed | `flash_attn`, `xformers` (broken ABI; not needed for π0/ACT) |

### `openpi_train` — reference only (JAX, not in critical path)

| | |
|---|---|
| Python | 3.11.15 |
| Path | `/home/010796032/.conda/envs/openpi_train/bin/python` |
| openpi venv | `/data/rnd-liu/aiprojects/openpi/.venv/` (uv-managed) |
| Use for | Reproducing student `vla-pi0-comparison` baseline if needed; not on critical path |
| Status | GCS-blocked (cluster firewall) → norm-stats download failed but unused so far |

### Shell env vars (set in user's session)

```bash
export HF_HOME=/data/rnd-liu/.cache/huggingface     # caches under /data (57 TB free)
export HF_TOKEN=hf_jiSMAsI...                       # ⚠ rotate
export NVIDIA_API_KEY=nvapi-...                     # ⚠ rotate (was leaked in chat)
export MUJOCO_GL=egl                                # set by scripts as needed
export OPENPI_DATA_HOME=/data/rnd-liu/.cache/openpi
```

---

## 4. Repos & datasets — paths and sizes

### Code repos (read/write)

| Path | What | Size |
|---|---|---|
| `/fs/atipa/data/rnd-liu/MyRepo/omniverselab/` | This repo (workspace) | — |
| `/fs/atipa/data/rnd-liu/MyRepo/omniverselab/mylerobot/` | Our research package | < 1 MB |
| `/fs/atipa/data/rnd-liu/MyRepo/omniverselab/IsaacSim/` | IsaacSim collectors (v5, v6, addons) | < 1 MB |
| `/data/rnd-liu/aiprojects/lerobot/` | lerobot editable install (pinned `81948979`) | ~5 GB |
| `/data/rnd-liu/aiprojects/openpi/` | openpi reference (pinned `e4580662`) | ~700 MB |
| `/data/rnd-liu/aiprojects/LIBERO-plus/` | LIBERO-Plus repo (drop-in libero replacement) | 247 MB |
| `/data/rnd-liu/MyRepo/DeepDataMiningLearning/DeepDataMiningLearning/bevdet/` | User's BEVDet code (source for voxel-fusion port) | — |
| `/data/rnd-liu/Others/vla-pi0-comparison/` | SJSU student π0 vs π0-FAST methodology template | — |

### Trained model checkpoints

| Path | Model | Status |
|---|---|---|
| `/data/rnd-liu/aiprojects/lerobot/outputs/train/2026-02-22/21-21-20_aloha_act/checkpoints/020000/` | ACT 20k (original) | 0% on aloha insertion — undertrained |
| `/data/rnd-liu/aiprojects/lerobot/outputs/train/2026-05-24/aloha_act_100k/checkpoints/last/` | **ACT 100k v1** (official Phase 1 baseline) | **8%** on aloha insertion |
| `/data/rnd-liu/aiprojects/lerobot/outputs/train/2026-05-24/aloha_act_100k_v2/` | ACT 100k v2 (failed exploration) | 0% — DO NOT USE |
| `/data/rnd-liu/aiprojects/lerobot/outputs/train/2026-05-24/pi0_libero_smoketest_v2/` | π0 100-step training smoke | loss 0.9→0.6; pipeline validated |
| `/data/rnd-liu/.cache/huggingface/hub/models--lerobot--pi0_libero_finetuned_v044/` | **HF-hosted lerobot π0 LIBERO** (our official baseline) | **66/76/74/36% per LIBERO suite; mean 63%** |

### Datasets (HuggingFace caches and local)

| Dataset | Location | Size | Cameras | Notes |
|---|---|---|---|---|
| `lerobot/aloha_sim_insertion_human` | HF cache | small | 1 (top) | Phase 1 baseline source |
| `lerobot/libero` (combined 4 suites) | HF cache | — | 2 (image, image2) | Standard LIBERO; was used for π0 training smoke |
| **`Sylvest/libero_plus_lerobot`** (v3.0 converted) | `/data/rnd-liu/.cache/huggingface/Sylvest_libero_plus_lerobot/` | **15 GB** | 2 (front, wrist) | **LIBERO-Plus training set: 14,347 eps, 40 tasks, 2.24M frames @ 20fps. Has perturbation variants baked in.** |
| `Sylvest_libero_plus_lerobot_old` (v2.1 backup) | `..._old/` | 15 GB | — | Can delete to free space |
| **`local/libero_multicam_v0`** | `/data/rnd-liu/.cache/lerobot_local/local/libero_multicam_v0/` | **8.6 GB** | 5 (agentview, wrist, front, side, bird) + per-cam extrinsics + intrinsics | **Our re-rendered LIBERO-spatial: 292 successful episodes from pi0_libero_finetuned_v044, 30,422 frames** |
| LIBERO-Plus assets (3D meshes, textures) | `/data/rnd-liu/aiprojects/LIBERO-plus/libero/libero/assets/` (symlinked) | 11 GB | — | Required by perturbation tasks |
| LIBERO-Plus task classification | `/data/rnd-liu/aiprojects/LIBERO-plus/libero/libero/benchmark/task_classification.json` | small | — | task → (category, difficulty) map |

### Cache locations (env vars set above)

```
/data/rnd-liu/.cache/huggingface/       # HF datasets + models
/data/rnd-liu/.cache/openpi/            # openpi pretrained weights (not used yet)
/data/rnd-liu/.cache/uv/                # openpi uv cache
/data/rnd-liu/.cache/libero/datasets/   # LIBERO-Plus dataset symlink target
/data/rnd-liu/.cache/lerobot_local/     # Our locally-generated datasets
```

---

## 5. Install recipes (reproducible)

### A) lerobot editable install (already done)

```bash
conda activate py312
cd /data/rnd-liu/aiprojects/lerobot
git checkout 81948979
pip install -e .
# Note: groot @strict bug needs the mylerobot env_shim
```

### B) openpi reference install (done; not on critical path)

```bash
conda create -n openpi_train -y python=3.11 pip
/home/010796032/.conda/envs/openpi_train/bin/pip install uv
cd /data/rnd-liu/aiprojects && git clone --recurse-submodules https://github.com/Physical-Intelligence/openpi.git
cd openpi && git checkout e4580662 && git submodule update --init --recursive
UV_CACHE_DIR=/data/rnd-liu/.cache/uv .venv/bin/uv sync --no-dev --python /home/010796032/.conda/envs/openpi_train/bin/python
```

### C) LIBERO-Plus install (working recipe — done 2026-05-25)

```bash
# 1. Clone the repo
cd /data/rnd-liu/aiprojects
git clone https://github.com/sylvestf/LIBERO-plus.git

# 2. Uninstall vanilla libero
/home/010796032/.conda/envs/py312/bin/pip uninstall -y libero

# 3. pip install -e (NB: setup.py find_packages() returns empty due to
#    setuptools quirk — install succeeds but `import libero` fails)
/home/010796032/.conda/envs/py312/bin/pip install -e /data/rnd-liu/aiprojects/LIBERO-plus

# 4. Workaround: add path to site-packages via .pth
echo "/data/rnd-liu/aiprojects/LIBERO-plus" > /home/010796032/.conda/envs/py312/lib/python3.12/site-packages/libero_plus_path.pth

# 5. Extra deps
/home/010796032/.conda/envs/py312/bin/pip install wand scikit-image gym
conda install -n py312 -y -c conda-forge imagemagick   # for libwand

# 6. Download assets bundle (~6 GB)
/home/010796032/.conda/envs/py312/bin/python -c "
from huggingface_hub import snapshot_download
snapshot_download(repo_id='sii-research/LIBERO_plus_assets', repo_type='dataset',
                   local_dir='/data/rnd-liu/.cache/huggingface/LIBERO_plus_assets')"
mkdir -p /data/rnd-liu/aiprojects/LIBERO-plus/libero/libero/assets_libero_plus_extracted
cd /data/rnd-liu/aiprojects/LIBERO-plus/libero/libero/assets_libero_plus_extracted
unzip -q /data/rnd-liu/.cache/huggingface/LIBERO_plus_assets/assets.zip
# 7. Symlink to expected path (zip extracts 9 levels deep into inspire/hdd/...)
SRC=/data/rnd-liu/aiprojects/LIBERO-plus/libero/libero/assets_libero_plus_extracted/inspire/hdd/project/embodied-multimodality/public/syfei/libero_new/release/dataset/LIBERO-plus-0/assets
DST=/data/rnd-liu/aiprojects/LIBERO-plus/libero/libero/assets
ln -sfn "$SRC" "$DST"

# 8. Write libero config — must point at LIBERO-plus paths, not the original libero pip install
cat > /home/010796032/.libero/config.yaml <<'EOF'
benchmark_root: /data/rnd-liu/aiprojects/LIBERO-plus/libero/libero
bddl_files: /data/rnd-liu/aiprojects/LIBERO-plus/libero/libero/bddl_files
init_states: /data/rnd-liu/aiprojects/LIBERO-plus/libero/libero/init_files
datasets: /data/rnd-liu/.cache/libero/datasets
assets: /data/rnd-liu/aiprojects/LIBERO-plus/libero/libero/assets
EOF

# 9. Verify
/home/010796032/.conda/envs/py312/bin/python -c "
import mylerobot  # env shim
from libero.libero import benchmark
print(list(benchmark.get_benchmark_dict().keys()))
"
```

**Apt deps the README lists** (`libexpat1`, `libfontconfig1-dev`, `libmagickwand-dev`) — only the last one was needed, handled by conda's `imagemagick`. Skipped others.

### D) LIBERO-Plus training data (Sylvest/libero_plus_lerobot)

```bash
# Download (15 GB)
/home/010796032/.conda/envs/py312/bin/python -c "
from huggingface_hub import snapshot_download
snapshot_download(repo_id='Sylvest/libero_plus_lerobot', repo_type='dataset',
                   local_dir='/data/rnd-liu/.cache/huggingface/Sylvest_libero_plus_lerobot')"

# Convert v2.1 → v3.0 (in place; old version renamed _old)
/home/010796032/.conda/envs/py312/bin/python -m lerobot.scripts.convert_dataset_v21_to_v30 \
  --repo-id Sylvest/libero_plus_lerobot \
  --root /data/rnd-liu/.cache/huggingface/Sylvest_libero_plus_lerobot \
  --push-to-hub false
```

### E) mylerobot package itself

```bash
cd /fs/atipa/data/rnd-liu/MyRepo/omniverselab/mylerobot
/home/010796032/.conda/envs/py312/bin/pip install -e .
/home/010796032/.conda/envs/py312/bin/pytest tests -q   # expect 32 passed
```

---

## 6. Code we've built (`mylerobot/` tree)

```
mylerobot/
├── pyproject.toml
├── README.md
├── PROJECT_STATUS.md                    # this file
├── 2604.24921v1.pdf                     # Libra-VLA paper (informed pivot)
├── mylerobot/
│   ├── __init__.py                      # installs env_shim BEFORE any lerobot import
│   ├── _env_shim.py                     # flash_attn stub + groot @strict workaround
│   ├── policies/
│   │   ├── act_lang/                    # ACTLangPolicy subclass (3 fusion modes)
│   │   ├── pi0_voxel/                   # TODO: voxel-augmented π0 subclass
│   │   └── vision_fusion/
│   │       └── voxel_xattn.py           # VoxelCrossAttnFusion — BEVDet port, 15 tests
│   ├── sim/
│   │   ├── aloha_multicam.py            # 5-cam aloha wrapper (legacy)
│   │   └── libero_multicam.py           # LIBERO multi-cam: standalone LiberoMultiCamEnv
│   │                                     # + make_capture_env() for lerobot LiberoEnv subclass
│   └── ...
├── scripts/
│   ├── eval_act_sim.py                  # gym_aloha rollout eval
│   ├── eval_libero.py                   # shim around lerobot_eval for LIBERO
│   ├── eval_libero_plus_fragility.py    # NEW: per-(suite, category) fragility scan
│   ├── train_act.py                     # shim around lerobot_train
│   ├── rerender_libero_multicam.py      # generates local/libero_multicam_v0
│   └── probe_seed_convention.py         # (one-shot) showed aloha_sim_*_scripted has no clean seed convention
└── tests/
    ├── conftest.py                      # imports _env_shim
    ├── test_policy_forward.py           # 9 tests: ACTLangPolicy
    ├── test_aloha_multicam.py           # 8 tests
    └── test_voxel_xattn.py              # 15 tests: voxel projection + attention + grads
```

**32 unit tests passing** as of 2026-05-25.

### IsaacSim collectors (for the parallel deployment track)

```
IsaacSim/
├── isaac_auto_collector_v5.py           # original 1-cam scripted Franka collector
├── isaac_multicam_addons.py             # NEW: 5-cam scene cfg + LeRobotMultiCamSaver
├── make_v6_from_v5.py                   # patches v5 → v6 surgical edits
├── isaac_auto_collector_v6.py           # generated (1517 lines, AST-parses)
└── test_multicam_addons.py              # 12 tests (geometry + H5 saver, no IsaacLab needed)
```

---

## 7. Master results table

| # | Variant | Task / suite | Result | Notes |
|---|---|---|---|---|
| 0a | ACT 20k (original) | aloha_sim_insertion | 0% (0/50) | undertrained |
| **0** | **ACT 100k v1** | aloha_sim_insertion | **8% (4/50)** | Phase 1 "single-camera perception bottleneck" reference |
| 0_failed | ACT 100k v2 (lr=1e-4 on backbone) | aloha_sim_insertion | 0% — DO NOT cite | failed exploration |
| 1a | pi0_libero_finetuned_v044 | libero_spatial (smoke, n=1/task) | 60% | pipeline validated |
| **1b** | **pi0_libero_finetuned_v044** | **all 4 LIBERO suites, n=10/task** | **mean 63.0%** (s=66, o=76, g=74, l10=36) | **OFFICIAL LIBERO-standard baseline (67 min wall)** |
| 1c | π0 from pi0_base, 100-step LoRA | LIBERO | loss 0.9 → 0.6 | training pipeline validated |
| **1d** | **pi0_libero_finetuned_v044 multicam rerender** | libero_spatial, 50 inits/task × 10 tasks = 500 attempts | **58.4% success (292 saved as `local/libero_multicam_v0`)** | 5-cam dataset for sensor-dropout ablation |
| **1e** | **pi0_libero_finetuned_v044 — LIBERO-Plus fragility scan** | 4 suites × 7 categories × 10 tasks (n=280) | **mean 42.1%** (with libero_10 ms=220); **mean 48.5%** if libero_10 re-eval at ms=520 is folded in | **OFFICIAL LIBERO-Plus baseline** (83 min wall + 41 min libero_10 re-eval) |
| **1e'** | Same with libero_10 re-eval at max_steps=520 | libero_10 only (n=70) | **mean 25.7%** (Camera 20, Robot 0, Layout 40, Light 40, Bg 60, Noise 10, Lang 10) | fixes 0% row caused by undersized max_steps |
| **2a** | **PI0VoxelPolicy from `pi0_base` — 10-step smoke** | Sylvest/libero_plus_lerobot, batch=1 | **loss 0.4–1.6, checkpoint saved (16 GB)** | validates voxel-fusion pipeline E2E |
| 2b_failed | PI0VoxelPolicy from `pi0_base` — 5k LoRA on Sylvest | Sylvest, batch=4 | loss 1.6 → 0.37; fragility 0/10 first 4 cells → undertrained | starting from raw `pi0_base` needs ≫5k steps to recover LIBERO competence |
| **2b** | **PI0VoxelPolicy from `pi0_libero_finetuned_v044` — 10k LoRA** | `lerobot/libero` (matches v044), batch=4, train_expert_only=True | **loss 0.16 final** | competent voxel-augmented checkpoint |
| **2c** | **PI0VoxelPolicy from v044+10k — LIBERO-Plus fragility scan** | 4 suites × 7 cats × 10 tasks (n=280, ms=220) | **mean 38.9%** vs baseline 42.1% — see Δ table below | Camera +15pp, but overall **−3.2pp** → mixed (see notable regressions) |
| **2d** | **+ zero-init out_proj + image augmentations — 10k LoRA + fragility** | same eval config | **mean 46.1% vs baseline 42.1% → +3.9pp overall**; Camera +5, Layout +5, Light +10, Bg +12.5, Noise ±0, Robot −5, Lang ±0 | **the fix worked**: net-positive across ALL columns except Robot Init; biggest regressions of 2c (Noise −60, Bg −60) are gone |
| **2d'** | **+ libero_10 re-eval at ms=520 (fair budget for long-horizon suite)** | libero_10 only (n=70) | **row mean 32.9% vs baseline 25.7% → +7.2pp**; Light +30, Bg +20, Camera +10 | **fair all-suites mean: 54.3% vs baseline 48.5% → +5.7pp**; voxel-fusion advantage holds on long-horizon |
| **2e** | **Augmentations-only control (`pi0_v044` + 10k LoRA + augs, NO voxel)** | full fair eval (ms=220 + libero_10 ms=520) at n=10 | **mean 42.1% (−6.4pp vs baseline)** | augs ALONE *hurt* the model; voxel-fusion appears to be load-bearing component (+12.1pp on top of augs) — **superseded by row 2e' at n=50** |
| **2e'** | **n=50 expansion: 3-way 12-cell comparison (Camera + Bg + Light, all 4 suites)** | n=50 fair budget | baseline **60.8%**, augs-only **58.0%** (−2.8), voxel+augs **56.0%** (**−4.8**); per-column voxel-vs-baseline: Camera +8.5, Bg −10.5, Light −12.5 | **AT N=50, MOST GAINS COLLAPSE.** n=10 results were sampling noise. Only Camera column shows positive voxel effect. The +5.7pp 2d' headline was illusory. |
| **2f** | **Extrinsics-corruption ablation (identity, shuffle) at n=50 on Camera headline cells** | libero_spatial Camera + libero_10 Camera | identity: libero_spatial 68% (vs correct 56%, **+12pp BETTER**); libero_10 38% (vs 40%, tied). shuffle: libero_spatial 58% (tied); libero_10 28% (vs 40%, **−12pp**) | **Voxel module is NOT using 3D geometry**: identity extrinsics ≈ correct or slightly better. Shuffle hurts libero_10 only (weak per-cam identity binding). Module functions as extra-capacity tokens + camera-ID embedding. |
| **2g** | **Camera-dropout ablation (drop image / image2) on libero_spatial × Camera n=50** | 3 variants × 3 conditions | Full / drop image / drop image2: baseline 52/36/56; augs-only 38/54/48; voxel+augs 56/52/72 | **Voxel beats baseline on agentview-drop by 12pp** but **augs-only beats voxel by 20pp on same** (augs-only +16 vs voxel −4). Augs alone are the most cam-dropout robust. Voxel ≠ "sensor-agnostic." |
| **2h** | **EXTRINSICS BUG DISCOVERED (2026-05-26)** | env probe vs hardcoded constants | `AGENTVIEW_XMAT_WORLD` had a SIGN ERROR — camera was looking AWAY from workspace. With broken xmat, EEF projects to uv=(142, **−2180**) outside 256×256 image. With env-probed xmat, EEF projects to uv=(128, 61) near image center. State[:3] is correctly in world frame (no offset needed). | **Every prior voxel experiment used wrong projections.** Suspected to explain row 2f null result. |
| **2h.1** | Fixed-xmat eval on broken-trained ckpt | n=50 libero_spatial Camera + libero_10 Camera | libero_spatial: fixed-correct 70% vs broken-correct 56% vs identity 68% (all within noise). libero_10: **fixed-correct 16% vs broken-correct 40% vs identity 38%** (fixed-correct catastrophically worse) | Model learned to depend on BROKEN projections; switching to correct at eval is out-of-distribution on long-horizon. **Architecture HAS projection-dependence — just not the right kind.** |
| **2h.2** | **Retrain pi0_voxel from v044 with FIXED xmat** (10k LoRA, augs on) | `lerobot/libero` | loss 0.171 (identical to broken-xmat run) | training landscape unchanged — action loss can't see extrinsics correctness |
| **2h.3** | **DEFINITIVE: Extrinsics-corruption on fixed-xmat-trained model** | n=50 libero_spatial Camera | **correct (fixed) 52% vs identity 60%** → identity **+8pp HIGHER** than correct | **Architecture does NOT learn to use 3D geometry even when fed correct extrinsics during training.** Row 2f's null result was real, not a constants bug. |
| **(KILLED)** | Voxel-fusion as monolithic-prefix 3D-grounding | LIBERO-Plus | ABANDONED 2026-05-26 | The bolt-on voxel-cross-attention architecture cannot learn to use geometric structure from action loss alone. Same failure mode as PointACT's monolithic VLA. |
| (Road B-3 OC-VLA) | Camera-frame action prediction (forces extrinsics structurally load-bearing) | LIBERO-Plus | TBD | The strongest precedent for "policy uses geometry" but doesn't address architecture-level capacity |
| (Road B-4 PointACT) | **Dual-system routing: voxel → action expert directly, bypass PaliGemma** | LIBERO-Plus | TBD | **DIRECT FIX for our exact failure mode** (PointACT showed 18.6→73.2% recovery with this) |
| (Road B-5) | Auxiliary geometric supervision (state/depth prediction head on voxel features) | LIBERO-Plus | TBD | Cheap (1-2 days); forces voxel features to encode 3D directly. Compose with B-4. |

### Row 1e — official baseline fragility table

| Suite | Camera | Robot Init | Layout | Light | Background | Noise | Language | row mean |
|---|---|---|---|---|---|---|---|---|
| libero_spatial | 60% | 50% | 80% | 80% | 30% | 70% | 40% | 58.6 |
| libero_object | 20% | 10% | 50% | 30% | 40% | 80% | 20% | 35.7 |
| libero_goal | 80% | 60% | 70% | 90% | 100% | 100% | 20% | 74.3 |
| libero_10 \* | 20% | 0% | 40% | 40% | 60% | 10% | 10% | 25.7 |
| **col mean** | **45%** | **30%** | **60%** | **60%** | **57.5%** | **65%** | **22.5%** | **48.5%** |

\* libero_10 re-evaluated at `max_steps=520` (default for long-horizon suite). Original scan used 220 steps for all suites; updated cell-values now reflect proper budget. Camera column is our voxel-fusion target — drops to 45% vs clean LIBERO (66%).

### Row 2c — voxel-fusion vs baseline delta (Δ in percentage points)

`pi0voxel_from_v044_10k` vs `pi0_libero_finetuned_v044`, both at ms=220, n=10/cell. Cell format: `variant% (baseline%) Δ`.

| Suite | Camera | Robot Init | Layout | Light | Background | Noise | Language | row Δ |
|---|---|---|---|---|---|---|---|---|
| libero_spatial | 70 (60) **+10** | 20 (50) -30 | 60 (80) -20 | 80 (80) ±0 | 20 (30) -10 | 70 (70) ±0 | 50 (40) **+10** | −5.7 |
| libero_object | 60 (20) **+40** | 20 (10) **+10** | 80 (50) **+30** | 40 (30) **+10** | 50 (40) **+10** | 20 (80) -60 | 30 (20) **+10** | **+7.1** |
| libero_goal | 90 (80) **+10** | 60 (60) ±0 | 60 (70) -10 | 80 (90) -10 | 40 (100) -60 | 80 (100) -20 | 10 (20) -10 | −14.3 |
| libero_10 \* | 0 (0) ±0 | 0 (0) ±0 | 0 (0) ±0 | 0 (0) ±0 | 0 (0) ±0 | 0 (0) ±0 | 0 (0) ±0 | ±0 |
| **col Δ** | **+15** | **−5** | **±0** | **±0** | **−15** | **−20** | **+2.5** | **−3.2** |

\* libero_10 uninformative at ms=220 for both variants; needs ms=520 re-eval.

**Headline:** Camera Viewpoints **+15 pp** (40 → 55). Voxel-fusion trades **~3 pp** mean for **+15 pp** on the perturbation dim that drops hardest from clean LIBERO. Robust-vision claim supported.

**Notable regressions:**
- libero_object × Sensor Noise: 80 → 20 (-60). Voxel module may amplify ImageMagick blob noise.
- libero_goal × Background Textures: 100 → 40 (-60). Background change confuses the workspace-bounds voxel grid (uses hard-coded Panda tabletop bounds).
- libero_goal × Sensor Noise: 100 → 80 (-20). Similar to above.

These regressions suggest the voxel module overfit to clean visual features during 10k LoRA. Future ablations: (a) longer training with random augmentations, (b) voxel attention dropout, (c) zero-init the voxel→prefix projection so initial behavior matches baseline.

### Row 2d — voxel-fusion + zero-init + augmentations (the fix)

`pi0voxel_v044+10k_zeroinit+augs` vs `pi0_libero_finetuned_v044`. Same eval config as 2c. Cell format: `variant% (baseline%) Δ`.

| Suite | Camera | Robot Init | Layout | Light | Background | Noise | Language | row Δ |
|---|---|---|---|---|---|---|---|---|
| libero_spatial | 40 (60) -20 | 50 (50) ±0 | 80 (80) ±0 | 70 (80) -10 | 50 (30) **+20** | 80 (70) **+10** | 40 (40) ±0 | ±0 |
| libero_object | 50 (20) **+30** | 0 (10) -10 | 70 (50) **+20** | 80 (30) **+50** | 90 (40) **+50** | 80 (80) ±0 | 10 (20) -10 | **+18.6** |
| libero_goal | 90 (80) **+10** | 50 (60) -10 | 70 (70) ±0 | 90 (90) ±0 | 80 (100) -20 | 90 (100) -10 | 30 (20) **+10** | −2.9 |
| libero_10 \* | 0 (0) ±0 | 0 (0) ±0 | 0 (0) ±0 | 0 (0) ±0 | 0 (0) ±0 | 0 (0) ±0 | 0 (0) ±0 | ±0 |
| **col Δ** | **+5** | **−5** | **+5** | **+10** | **+12.5** | **±0** | **±0** | **+3.9** |

\* libero_10 uninformative at ms=220 for both.

**Headline:** Overall **+3.9 pp** vs baseline (vs row 2c's −3.2 pp). The two architecture fixes:
1. **Zero-init `VoxelCrossAttnFusion.out_proj`** (weight + bias) — voxel-token contribution starts at zero, so the action expert isn't poisoned by random voxel garbage early in training. Standard ControlNet/LoRA/adapter trick.
2. **`--dataset.image_transforms.enable=true`** — lerobot's default brightness/contrast/saturation/hue/sharpness jitter during training. Directly addresses Sensor Noise + Background Textures regressions.

**What recovered (2d vs 2c):**
- libero_object × Sensor Noise: 20 → 80 (+60pp recovery)
- libero_goal × Background Textures: 40 → 80 (+40pp recovery)
- libero_object × Light: 40 → 80 (+40pp)
- libero_object × Background: 50 → 90 (+40pp)
- libero_spatial × Robot Init: 20 → 50 (+30pp recovery)
- libero_spatial × Layout: 60 → 80 (+20pp recovery)

**Trade-off vs 2c:** Camera column dropped from +15 to +5 (lost 10pp of peak). This is consistent with the zero-init mechanic: with `out_proj.weight = 0`, gradients into the voxel attention block only start flowing after `out_proj` itself trains up, so the voxel module learns more slowly. 10k steps may not be enough to fully match 2c's Camera peak — see row 2e (TODO: 30k retrain).

**Paper framing:** the architecture-correctness story now stands on its own: voxel-fusion + zero-init + augmentations gives a **net-positive** improvement across all columns except Robot Init (-5). The Camera gain (+5) is no longer offset by other regressions.

### Row 2d' — fair-budget delta (libero_10 at ms=520, others at ms=220)

This is the table to report in the paper. Both runs use the same per-suite step budget; libero_10 row no longer all-zero.

| Suite | Camera | Robot Init | Layout | Light | Background | Noise | Language | row mean | row Δ |
|---|---|---|---|---|---|---|---|---|---|
| libero_spatial | 40 (60) -20 | 50 (50) ±0 | 80 (80) ±0 | 70 (80) -10 | 50 (30) **+20** | 80 (70) **+10** | 40 (40) ±0 | 58.6 (58.6) | ±0 |
| libero_object | 50 (20) **+30** | 0 (10) -10 | 70 (50) **+20** | 80 (30) **+50** | 90 (40) **+50** | 80 (80) ±0 | 10 (20) -10 | 54.3 (35.7) | **+18.6** |
| libero_goal | 90 (80) **+10** | 50 (60) -10 | 70 (70) ±0 | 90 (90) ±0 | 80 (100) -20 | 90 (100) -10 | 30 (20) **+10** | 71.4 (74.3) | −2.9 |
| libero_10 | 30 (20) **+10** | 0 (0) ±0 | 40 (40) ±0 | 70 (40) **+30** | 80 (60) **+20** | 10 (10) ±0 | 0 (10) -10 | 32.9 (25.7) | **+7.2** |
| **col Δ** | **+7.5** | **−5** | **+5** | **+17.5** | **+17.5** | **±0** | **−2.5** | **54.3 (48.5)** | **+5.7** |

**Headline (paper-ready):** voxel-fusion + zero-init + augmentations gives **+5.7 pp overall** on LIBERO-Plus across all 4 suites. Largest column gains: **Light +17.5**, **Background +17.5**, **Camera +7.5**. Only negative column is Robot Initial States (−5). No regression > 10 pp on any column.

**Per-suite picture:** libero_object gets the biggest lift (+18.6 row mean), libero_10 long-horizon improves (+7.2), libero_spatial holds baseline (±0), libero_goal slips slightly (−2.9 — driven by Bg/Noise where baseline was already at 100%).

### Row 2e — augmentations-only ablation (isolates voxel-fusion's contribution)

3-way comparison, all at fair budget (libero_10 ms=520, others ms=220, n=10/cell):

| Variant | All-suites mean | Δ vs baseline |
|---|---|---|
| Baseline `pi0_v044` (no LoRA, no augs, no voxel) | **48.6%** | — |
| Augs-only (`pi0_v044` + 10k LoRA + image augs, NO voxel) | **42.1%** | **−6.4** |
| Voxel + augs (`pi0voxel_v044` + 10k LoRA + zero-init + augs) | **54.3%** | **+5.7** |

**Per-column decomposition (column means across 4 suites):**

| Column | Baseline | Augs-only | Voxel+augs | augs effect | **voxel effect** | total |
|---|---|---|---|---|---|---|
| Camera | 45.0 | 40.0 | 52.5 | −5.0 | **+12.5** | +7.5 |
| Robot Init | 30.0 | 32.5 | 25.0 | +2.5 | −7.5 | −5.0 |
| Layout | 60.0 | 45.0 | 65.0 | −15.0 | **+20.0** | +5.0 |
| Light | 60.0 | 60.0 | 77.5 | ±0 | **+17.5** | +17.5 |
| Background | 57.5 | 42.5 | 75.0 | −15.0 | **+32.5** | +17.5 |
| Sensor Noise | 65.0 | 60.0 | 65.0 | −5.0 | +5.0 | ±0 |
| Language | 22.5 | 15.0 | 20.0 | −7.5 | +5.0 | −2.5 |
| **mean** | **48.6** | **42.1** | **54.3** | **−6.4** | **+12.1** | **+5.7** |

**Per-suite row means (fair budget):**

| Suite | Baseline | Augs-only | Voxel+augs | augs effect | voxel effect |
|---|---|---|---|---|---|
| libero_spatial | 58.6 | 47.1 | 58.6 | −11.5 | **+11.5** |
| libero_object | 35.7 | 35.7 | 54.3 | ±0 | **+18.6** |
| libero_goal | 74.3 | 68.6 | 71.4 | −5.7 | +2.9 |
| libero_10 | 25.7 | 17.1 | 32.9 | −8.6 | **+15.8** |

**Key takeaways for the paper:**

1. **Image augmentations alone HURT the baseline π0** (−6.4 pp overall, −11.5 to −5.7 on three of four suites). Most likely cause: fine-tuning the action expert on augmented images drifts it off the v044 manifold without giving it useful new visual structure.

2. **Voxel-fusion is the load-bearing component.** It adds **+12.1 pp on top of augs-only** (i.e. the row 2d gain of +5.7 vs baseline is *despite* the augs hurting, not because augs help). Largest voxel contributions: **Background +32.5**, **Layout +20**, **Light +17.5**, **Camera +12.5**.

3. **Background Textures (+32.5pp from voxel) is the cleanest signal.** This perturbation changes the wall/table texture without touching the manipulated objects — exactly where a 3D-geometric prior should shine. Voxel-fusion grounds objects in workspace coordinates regardless of background appearance.

4. **The one weakness is Robot Initial States** (voxel −7.5 vs augs-only, total −5 vs baseline). The hard-coded Panda tabletop workspace bounds don't follow the robot when its starting pose changes — projecting voxel queries to "where the workspace should be" gives wrong feature samples. **Fix candidate:** per-batch workspace bounds derived from `observation.state[:3]`.

**Caveat (n=10):** Wilson 95% CI for a column mean at this n is roughly ±10 pp. The Background and Light +17.5/+32.5 effects are well outside noise; Camera +7.5 and Robot −5 are inside. Needs n=50 expansion before CoRL submission.

### Row 2e' — n=50 full 3-way table (supersedes 2e at n=10)

12-cell headline scan (Camera + Background + Light, all 4 suites) at n=50/cell. libero_10 at ms=520, others at ms=220.

| Cell | baseline | augs-only | voxel+augs | voxel−augs | voxel−baseline |
|---|---|---|---|---|---|
| libero_spatial × Camera | 52 | 38 | 56 | **+18** | +4 |
| libero_spatial × Background | 66 | 52 | 58 | +6 | −8 |
| libero_spatial × Light | 66 | 66 | 62 | −4 | −4 |
| libero_object × Camera | 76 | 70 | 62 | −8 | −14 |
| libero_object × Background | 60 | 64 | 50 | −14 | −10 |
| libero_object × Light | 54 | 50 | 62 | **+12** | +8 |
| libero_goal × Camera | 56 | 66 | 60 | −6 | +4 |
| libero_goal × Background | 70 | 76 | 68 | −8 | −2 |
| libero_goal × Light | 86 | 64 | 56 | −8 | −30 |
| libero_10 × Camera (ms=520) | 0 | 22 | 40 | **+18** | **+40** |
| libero_10 × Background (ms=520) | 68 | 62 | 46 | −16 | −22 |
| libero_10 × Light (ms=520) | 76 | 66 | 52 | −14 | −24 |
| **Col mean (4 suites)** | | | | | |
| Camera | **46.0** | 49.0 | **54.5** | **+5.5** | **+8.5** |
| Background | **66.0** | 63.5 | 55.5 | −8.0 | −10.5 |
| Light | **70.5** | 61.5 | 58.0 | −3.5 | −12.5 |
| **GRAND MEAN (12 cells)** | **60.8** | **58.0** | **56.0** | **−2.0** | **−4.8** |

**Sharpest result:** libero_10 × Camera — baseline **0%**, voxel+augs **40%** (**+40pp** absolute gain). This is the strongest single piece of evidence we have for voxel helping.

**Most painful losses at n=50:** libero_goal × Light (−30 vs baseline), libero_10 × Background and Light (−22, −24).

### Row 2f — extrinsics-corruption ablation

Test: replace extrinsics with identity matrices or shuffle them per-batch. If voxel module uses 3D geometry, corruption should hurt.

| | Correct | Identity | Shuffle |
|---|---|---|---|
| libero_spatial × Camera (n=50, ms=220) | 56% | **68%** (+12) | 58% (+2) |
| libero_10 × Camera (n=50, ms=520) | 40% | 38% (−2) | **28%** (−12) |
| Mean | 48 | 53 | 43 |

**The voxel module does NOT function as a 3D-grounding mechanism.** Identity extrinsics (zero geometric information) match or beat correct extrinsics. Only shuffle hurts on libero_10, and only by 12pp — consistent with the module having learned a weak "this camera looks like this" identity binding, not actual 3D projection-based feature sampling.

### Row 2g — camera-dropout ablation

Test: zero out one of the two cameras at every timestep. Measures "sensor-agnostic" claim.

`libero_spatial × Camera Viewpoints, n=50, ms=220:`

| | Full | drop agentview | drop wrist |
|---|---|---|---|
| baseline | 52 | **36 (−16)** | 56 (+4) |
| augs-only | 38 | **54 (+16)** | 48 (+10) |
| voxel+augs | 56 | 52 (−4) | **72 (+16)** |

**Voxel+augs is 12pp more robust to agentview-drop than baseline (−4 vs −16).** But **augs-only is 20pp more robust still** (+16 vs voxel's −4). The "sensor-agnostic" property comes from augmentations, not from voxel-fusion.

**Both augs and voxel+augs perversely benefit when wrist is dropped (+10, +16).** Likely cause: on perturbed Camera tasks, the agentview is the perturbed view; the rigid-to-eef wrist view is unperturbed. The trained models had learned to over-weight the wrist features in a way that hurts on perturbed agentview. Removing the wrist forces sole reliance on agentview, which apparently works better.

---

## NEGATIVE FINDINGS SUMMARY (added 2026-05-26)

After running n=50 ablations + extrinsics-corruption + camera-dropout (rows 2e', 2f, 2g), the architecture as built does not deliver its intended mechanism:

1. **Voxel module does not use 3D geometry.** Replacing extrinsics with identity matrices doesn't hurt — sometimes helps. The cross-attention learns to use voxel tokens as extra capacity, with weak per-cam identity binding (shuffle hurts +12pp on libero_10 only). This is the same null result PointACT reported when bolting 3D tokens onto a VLM.

2. **Most n=10 "wins" were sampling noise.** n=50 expansion changed sign or magnitude on 7 of 12 headline cells. The row 2e "+12.1pp voxel contribution on top of augs" claim shrinks to −2.0pp at n=50. Wilson 95% CI at n=10 is ±28pp for 50% scores — too wide to trust.

3. **Augmentations alone match or beat voxel+augs on most cells at n=50.** Net augs-only effect: −2.8pp vs baseline (vs the row 2e claim of −6.4pp). Net voxel-vs-augs effect: −2.0pp (vs +12.1pp claim).

4. **The one survivable signal: libero_10 × Camera Viewpoints.** Baseline 0% → voxel+augs 40%. This is a long-horizon × geometric-perturbation cell where extra capacity (even if not properly geometric) may genuinely help. Worth keeping in any paper.

**Implication for paper direction:** the architecture-as-built is not publishable as a 3D-grounding method. Three options:
- **Workshop paper / negative result**: honest report of "we built voxel-cross-attention into π0, found it doesn't use the geometry it's given, and characterize why." Defensible at workshop venues.
- **Road B: redesign so voxel module is forced to use geometry.** Specific candidates: (1) OC-VLA-style camera-frame action prediction (makes extrinsics structurally necessary), (2) PointACT-style dual-system routing (voxel → action expert directly, not via PaliGemma prefix), (3) auxiliary geometric supervision (pose/depth aux head).
- **Pivot to a different claim** about π0's existing capabilities (e.g., language conditioning robustness, multi-task fine-tuning, etc.).

### Image augmentations recipe (used in row 2d)

```bash
--dataset.image_transforms.enable=true
```

This uses lerobot's default `ImageTransformsConfig`:
- `brightness` (ColorJitter 0.8-1.2)
- `contrast` (ColorJitter 0.8-1.2)
- `saturation` (ColorJitter 0.5-1.5)
- `hue` (ColorJitter -0.05 to 0.05)
- `sharpness` (SharpnessJitter 0.5-1.5)

Applied with `max_num_transforms=3, random_order=False` per default.

---

## 8. LIBERO-Plus benchmark structure (10,030 tasks)

Per-suite × per-category task counts (from `task_classification.json`):

| Suite | total | Camera | Robot Init | Layout | Light | Background | Noise | Language |
|---|---|---|---|---|---|---|---|---|
| libero_spatial | 2402 | 376 | 350 | 385 | 292 | 258 | 351 | 390 |
| libero_object  | 2518 | 396 | 398 | 403 | 297 | 248 | 422 | 354 |
| libero_goal    | 2591 | 408 | 409 | 425 | 279 | 281 | 379 | 410 |
| libero_10      | 2519 | 419 | 393 | 312 | 274 | 289 | 449 | 383 |
| **TOTAL**      | **10,030** | **1,599** | 1550 | 1525 | 1142 | 1076 | 1601 | 1537 |

Difficulty levels 1-5 per task. Camera Viewpoint task names encode the perturbation params in the suffix (e.g. `..._view_0_0_100_2_352_initstate_0`).

**LIBERO-Plus paper leaderboard (Camera column):**
| Model | Camera | Total |
|---|---|---|
| OpenVLA | 0.8% | 15.6% |
| OpenVLA-OFT | **56.4%** | 69.6% |
| OpenVLA-OFT_w | 10.4% | 55.8% |
| NORA | 2.2% | 39.0% |
| WorldVLA | 0.1% | 25.0% |

OpenVLA-OFT at 56.4% is the bar for Camera; matching/beating it would be a strong claim.

---

## 9. What's running / pending

**Just launched:**
- `bw7myfb8i` — fragility scan smoke (libero_spatial × Camera Viewpoints × 2 tasks), ETA ~5 min

**Tomorrow's queue (after smoke validates):**
1. Full fragility scan: 4 suites × 7 categories × 10 tasks/cell = 280 tasks (~3h)
2. Write `PI0VoxelCrossAttn` subclass — wire `VoxelCrossAttnFusion` into `Pi0Pytorch.embed_prefix` (the `tokens.append(voxel_tokens)` insertion point we found in lerobot's pi0 code)
3. Extrinsics path for Sylvest/libero_plus_lerobot training data — likely pre-augment with extrinsics derived from `observation.state` (8d eef_pos + axis_angle + gripper) + workspace-frame correction
4. Tesla-style PI0VoxelUNet variant: UNet 2D→3D + depth-back-projection occupancy aux head
5. LoRA fine-tune both variants on Sylvest/libero_plus_lerobot, re-run fragility scan, compute per-cell delta

**User parallel track:**
- IsaacSim multicam Franka data collection on RTX5090 (deployment subtrack)

---

## 10. Known open questions / risks

1. **Extrinsics for the LIBERO-Plus training data**: dataset only has 8d state (eef pose + gripper), not direct camera extrinsics. For voxel-fusion we need extrinsics at every frame. Two options when we get to fine-tuning:
   - (a) Recover EEF pose from state, compose with known wrist-cam-to-EEF rigid offset + hard-coded agentview pose. Frame-correction TBD.
   - (b) Pre-augment dataset with extrinsics by replaying through robosuite (one-time preprocess ~3-4 h).
2. **PaliGemma context budget**: with ~256 image tokens × 2 cams + voxel tokens + language, we're approaching limits. Pool voxel tokens via 3D conv to ~128.
3. **Depth-back-projection occupancy GT** quality: cheap but approximate (misses occluded interior). If results are weak, upgrade to mesh ray-cast.
4. **HF / Nvidia API keys exposed in shell env** — should be rotated.

---

## 11. How to resume work

```bash
conda activate py312
cd /fs/atipa/data/rnd-liu/MyRepo/omniverselab/mylerobot
pytest tests -q                          # expect 32 passed

# Reproduce a LIBERO-Plus baseline number quickly:
MUJOCO_GL=egl python scripts/eval_libero_plus_fragility.py \
  --suites libero_spatial \
  --categories "Camera Viewpoints" \
  --max_tasks_per_category 5 \
  --output_json /tmp/quick_fragility.json
```

Memory entries: `/home/010796032/.claude/projects/-fs-atipa-data-rnd-liu-MyRepo-omniverselab/memory/MEMORY.md` indexes 15+ project/feedback/reference notes from prior sessions.
