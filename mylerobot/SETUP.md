# mylerobot — full setup recipe

Reproducible install of the entire stack on a fresh HPC user account (no sudo). Last verified 2026-05-25 on `/home/010796032/` (H100 NVL box, NFS-mounted `/data/rnd-liu` and `/fs/atipa/data`).

For *project status / strategy / results*, see [PROJECT_STATUS.md](PROJECT_STATUS.md). This file is **install-only**: every command, every path, every gotcha.

---

## 0. Prerequisites

- Miniconda already installed at `/home/010796032/miniconda3/`
- Read-only sudo (no `apt install`). All system libs go through conda-forge.
- Two filesystems: `/home/010796032/` (small) and `/data/rnd-liu/` (57 TB, slow NFS). Put *all* big caches under `/data`.
- HuggingFace + Nvidia API keys already in shell env (⚠ both leaked at some point — rotate when convenient):
  ```bash
  export HF_HOME=/data/rnd-liu/.cache/huggingface
  export HF_TOKEN=hf_jiSMAsI...
  export NVIDIA_API_KEY=nvapi-...
  export MUJOCO_GL=egl
  export OPENPI_DATA_HOME=/data/rnd-liu/.cache/openpi
  ```

---

## 1. Conda envs

### `py312` (PRIMARY working env — π0, ACT, voxel, LIBERO-Plus eval)

```bash
conda create -n py312 -y python=3.12 pip
conda activate py312
```

System libs that need to be in this env (no sudo):

```bash
conda install -n py312 -y -c conda-forge cmake imagemagick
```

CMake is needed by `egl_probe` (libero dep). ImageMagick provides libwand for `wand` (libero env_wrapper.py imports it).

### `openpi_train` (REFERENCE only — JAX path, not on critical path)

```bash
conda create -n openpi_train -y python=3.11 pip
/home/010796032/.conda/envs/openpi_train/bin/pip install uv
```

---

## 2. Repos to clone (all under `/data/rnd-liu/aiprojects/`)

```bash
mkdir -p /data/rnd-liu/aiprojects
cd /data/rnd-liu/aiprojects

# lerobot at pinned commit — our subclass target
git clone https://github.com/huggingface/lerobot.git
cd lerobot && git checkout 81948979 && cd ..

# openpi reference (JAX, π0 source-of-truth)
git clone --recurse-submodules https://github.com/Physical-Intelligence/openpi.git
cd openpi && git checkout e4580662 && git submodule update --init --recursive && cd ..

# LIBERO-Plus — drop-in replacement for libero pip package
git clone https://github.com/sylvestf/LIBERO-plus.git
```

---

## 3. Install `lerobot` editable + dependencies (py312)

```bash
conda activate py312
cd /data/rnd-liu/aiprojects/lerobot
pip install -e .
```

Two upstream issues to work around:

1. **`flash_attn` + `xformers` are broken** (CUDA-ABI mismatch on this box). Uninstall — neither is needed for π0/ACT:
   ```bash
   pip uninstall -y flash_attn xformers
   ```

2. **`lerobot.policies.groot.groot_n1` uses `@strict` from `huggingface_hub.dataclasses`** in a way that crashes against the installed huggingface_hub 1.7.1. Workaround = `mylerobot._env_shim` (installed below) which monkey-patches the broken module at import time. The shim **must run before any `lerobot.*` import**.

Verify:
```bash
/home/010796032/.conda/envs/py312/bin/python -c "
import torch, lerobot
print('torch', torch.__version__, 'cuda', torch.cuda.is_available())
print('lerobot', lerobot.__version__, 'at', lerobot.__file__)
"
# expect torch 2.10.x cu128, lerobot 0.4.4
```

---

## 4. Install `mylerobot` editable (py312)

```bash
cd /fs/atipa/data/rnd-liu/MyRepo/omniverselab/mylerobot
pip install -e .
pytest tests -q
# expect: 32 passed
```

This package contains `mylerobot._env_shim` which patches the groot bug. Always `import mylerobot` (or any submodule) *first* in scripts that touch `lerobot.*`.

---

## 5. openpi reference install (optional — for student-baseline reproduction)

```bash
cd /data/rnd-liu/aiprojects/openpi
UV_CACHE_DIR=/data/rnd-liu/.cache/uv \
  /home/010796032/.conda/envs/openpi_train/bin/uv sync \
    --no-dev \
    --python /home/010796032/.conda/envs/openpi_train/bin/python
```

Verify (under openpi's own .venv):
```bash
.venv/bin/python -c "
import jax, openpi
print('jax', jax.__version__, 'devices', jax.devices())
"
```

Known issue: `openpi-assets/*` checkpoints live on `gs://` and the cluster blocks GCS — norm-stats download fails. Not used in our critical path; π0 weights instead come from HF (`lerobot/pi0_libero_finetuned_v044`).

---

## 6. LIBERO-Plus install (py312) — replaces vanilla `libero`

This is the trickiest part. Multiple workarounds.

```bash
conda activate py312

# 6.1 Uninstall vanilla libero (which got pulled in transitively by lerobot deps)
pip uninstall -y libero

# 6.2 Editable install. WARNING: succeeds with empty MAPPING due to a
#     setuptools quirk with the LIBERO-Plus setup.py — `find_packages()`
#     returns []. Install reports success but `import libero` fails.
pip install -e /data/rnd-liu/aiprojects/LIBERO-plus

# 6.3 Workaround: add the repo to sys.path via a .pth file. Now `import libero`
#     resolves to LIBERO-plus/libero/.
echo "/data/rnd-liu/aiprojects/LIBERO-plus" > \
  /home/010796032/.conda/envs/py312/lib/python3.12/site-packages/libero_plus_path.pth

# 6.4 Extra pip deps (from LIBERO-plus/extra_requirements.txt)
pip install wand scikit-image gym
```

### LIBERO-Plus assets (~6 GB download + 11 GB extracted)

```bash
/home/010796032/.conda/envs/py312/bin/python -c "
from huggingface_hub import snapshot_download
snapshot_download(repo_id='sii-research/LIBERO_plus_assets', repo_type='dataset',
                  local_dir='/data/rnd-liu/.cache/huggingface/LIBERO_plus_assets')"

mkdir -p /data/rnd-liu/aiprojects/LIBERO-plus/libero/libero/assets_libero_plus_extracted
cd /data/rnd-liu/aiprojects/LIBERO-plus/libero/libero/assets_libero_plus_extracted
unzip -q /data/rnd-liu/.cache/huggingface/LIBERO_plus_assets/assets.zip
```

⚠ The zip has 9 levels of nesting (author's filesystem layout got pickled). Symlink to the expected path:

```bash
SRC=/data/rnd-liu/aiprojects/LIBERO-plus/libero/libero/assets_libero_plus_extracted/inspire/hdd/project/embodied-multimodality/public/syfei/libero_new/release/dataset/LIBERO-plus-0/assets
DST=/data/rnd-liu/aiprojects/LIBERO-plus/libero/libero/assets
ln -sfn "$SRC" "$DST"
```

### LIBERO config file (`~/.libero/config.yaml`)

Repoint all paths to LIBERO-Plus:

```bash
mkdir -p /home/010796032/.libero
cat > /home/010796032/.libero/config.yaml <<'EOF'
benchmark_root: /data/rnd-liu/aiprojects/LIBERO-plus/libero/libero
bddl_files: /data/rnd-liu/aiprojects/LIBERO-plus/libero/libero/bddl_files
init_states: /data/rnd-liu/aiprojects/LIBERO-plus/libero/libero/init_files
datasets: /data/rnd-liu/.cache/libero/datasets
assets: /data/rnd-liu/aiprojects/LIBERO-plus/libero/libero/assets
EOF
```

### Verify the install end-to-end

```bash
MUJOCO_GL=egl /home/010796032/.conda/envs/py312/bin/python -c "
import mylerobot  # MUST come before any lerobot/libero import (env shim)
from libero.libero import benchmark
suites = list(benchmark.get_benchmark_dict().keys())
print('suites:', suites)
print('libero_spatial task count:', len(benchmark.get_benchmark_dict()['libero_spatial']().tasks))
# expect 2402 (vanilla LIBERO had 10)
"
```

### Apt deps the README mentions that we DID NOT need

The README says to `apt install libexpat1 libfontconfig1-dev libmagickwand-dev libpython3-stdlib`. On our box only `libmagickwand-dev` was actually required — covered by conda's `imagemagick` package. The others are either already installed system-wide or unused.

---

## 7. LIBERO-Plus training dataset (Sylvest/libero_plus_lerobot, ~15 GB)

```bash
/home/010796032/.conda/envs/py312/bin/python -c "
from huggingface_hub import snapshot_download
snapshot_download(repo_id='Sylvest/libero_plus_lerobot', repo_type='dataset',
                  local_dir='/data/rnd-liu/.cache/huggingface/Sylvest_libero_plus_lerobot')"
```

The dataset ships in lerobot format **v2.1** but our lerobot (v3.0+) requires v3.0 — convert in place (atomic rename: original → `*_old/`, v3.0 → original path):

```bash
/home/010796032/.conda/envs/py312/bin/python -m lerobot.scripts.convert_dataset_v21_to_v30 \
  --repo-id Sylvest/libero_plus_lerobot \
  --root /data/rnd-liu/.cache/huggingface/Sylvest_libero_plus_lerobot \
  --push-to-hub false
```

After conversion you can delete `~/.cache/huggingface/Sylvest_libero_plus_lerobot_old/` to free 15 GB.

Verify:
```bash
/home/010796032/.conda/envs/py312/bin/python -c "
import mylerobot
from lerobot.datasets.lerobot_dataset import LeRobotDatasetMetadata
m = LeRobotDatasetMetadata('Sylvest/libero_plus_lerobot',
                            root='/data/rnd-liu/.cache/huggingface/Sylvest_libero_plus_lerobot')
print(f'eps={m.total_episodes} frames={m.total_frames} fps={m.fps}')
print(f'cameras: {m.camera_keys}')
# expect: eps=14347 frames=2238036 fps=20, cameras=['observation.images.front', 'observation.images.wrist']
"
```

---

## 8. Other datasets used / cached

| Dataset | HF repo | Local path | Use |
|---|---|---|---|
| `lerobot/aloha_sim_insertion_human` | HF | `~/.cache/huggingface/` | Phase 1 ACT baseline |
| `lerobot/libero` | HF | `~/.cache/huggingface/` | Standard LIBERO (lerobot format) — used by π0 LIBERO baseline eval |
| `lerobot/pi0_libero_finetuned_v044` | HF | `~/.cache/huggingface/hub/models--lerobot--pi0_libero_finetuned_v044/` | The official π0 LIBERO checkpoint we use as the scripted policy + baseline |
| `Sylvest/libero_plus_lerobot` | HF | `/data/rnd-liu/.cache/huggingface/Sylvest_libero_plus_lerobot/` (v3.0) | LIBERO-Plus training set, 14k eps |
| `local/libero_multicam_v0` | (generated) | `/data/rnd-liu/.cache/lerobot_local/local/libero_multicam_v0/` (8.6 GB) | Our 292-ep 5-cam re-render with extrinsics |
| `sii-research/LIBERO_plus_assets` | HF | `/data/rnd-liu/.cache/huggingface/LIBERO_plus_assets/` (zip) → symlinked into LIBERO-plus repo | scene meshes + textures |

Don't put these under `/home` — that fs is much smaller.

---

## 9. Generating `local/libero_multicam_v0` (optional, ~3 hours)

If you need to re-create the 5-camera LIBERO-spatial dataset (already done once):

```bash
cd /fs/atipa/data/rnd-liu/MyRepo/omniverselab/mylerobot
MUJOCO_GL=egl /home/010796032/.conda/envs/py312/bin/python scripts/rerender_libero_multicam.py \
  --suites libero_spatial \
  --max_init_states 50 \
  --max_steps 220 \
  --output_repo_id local/libero_multicam_v0
# expects ~3h, ~50-60% success rate, output at:
# /data/rnd-liu/.cache/lerobot_local/local/libero_multicam_v0/
```

---

## 10. Common gotchas (read before debugging)

1. **`mylerobot` MUST be imported before any `lerobot.*`** — otherwise the groot @strict bug crashes the whole import chain. Every script in `mylerobot/scripts/` starts with `import mylerobot`.

2. **`MUJOCO_GL=egl`** must be set when running any rollout/render — defaults to `glfw` which needs an X display. Add to shell rc or pass per command.

3. **GPU memory contention**: the user (yourself) runs BEVDet experiments on the same H100. π0 inference needs ~10 GB. Always check first:
   ```bash
   nvidia-smi --query-gpu=memory.free --format=csv,noheader
   # require ≥ 12 GB before launching pi0 jobs
   ```
   When OOM hits, the error surface inside lerobot's normalize_processor at `denom = std + self.eps`.

4. **`libero` import is order-sensitive in subprocesses**: the lerobot `LiberoEnv` uses `AsyncVectorEnv` by default. The subprocess workers don't inherit our `mylerobot._env_shim` install. Workaround: pass `--eval.use_async_envs=false` to lerobot_eval, OR build a single-process env directly (which is what `eval_libero_plus_fragility.py` does).

5. **lerobot/libero dataset v2.1 vs v3.0**: any Sylvest/* dataset on HF ships v2.1. Run `convert_dataset_v21_to_v30` *before* trying to load. Old version preserved at `*_old/` after conversion.

6. **LIBERO-Plus init-state files**: perturbation-variant tasks (e.g. `..._view_X_Y_Z_W_V_initstate_N`) don't have their own `.pruned_init` file — the loader strips the perturbation suffix to find the base file. **You must pass `is_libero_plus=True` to `LiberoEnv` constructor** for this stripping to happen.

7. **lerobot dataset `add_frame`**: the `task` is a key INSIDE the frame dict, not a kwarg of `add_frame()`. Spent an iteration fixing this.

8. **`observation.images.X` flips per `LiberoProcessorStep`**: the env-side preprocessor rotates LIBERO images 180° (`torch.flip(img, dims=[2, 3])`) — this matches the HuggingFaceVLA camera-mount convention. Don't manually flip elsewhere.

9. **camera_name strings**: lerobot's `LiberoEnv.camera_name` uses the `_image` suffix (`"agentview_image"`); the underlying robosuite `camera_names` argument wants the mujoco short name (`"agentview"`). Strip suffixes when going from one to the other.

10. **`@editable__libero_0_1_0_finder.py` MAPPING is empty**: LIBERO-Plus's setup.py + setuptools combination produces an empty package map. The `.pth` workaround in step 6.3 is required.

11. **Custom policy classes — file-naming convention**: lerobot's `get_policy_class(name)` factory does NOT use the `PreTrainedConfig.register_subclass` registry to find the policy class. It looks at the **config module path** and string-replaces `configuration_` → `modeling_`. So a config at `mylerobot/policies/pi0_voxel/configuration_pi0_voxel.py` REQUIRES a sibling `modeling_pi0_voxel.py`. Files named `configuration.py` + `modeling.py` (no `_<name>` suffix) make the factory raise `AttributeError: module ... has no attribute PI0VoxelPolicy`.

12. **Custom policy classes must be eager subclasses, not lazy placeholders**: `make_policy()` calls `policy_cls.from_pretrained(**kwargs)` as a classmethod. A placeholder class that builds the real class in `__new__` does NOT expose `from_pretrained`, so loading from pretrained fails. Define `class PI0VoxelPolicy(PI0Policy):` directly at module top-level.

13. **Sylvest/libero_plus_lerobot videos are AV1**: torchcodec hits `ValueError: best video stream is unknown` on these files. Pass `--dataset.video_backend=pyav` to lerobot_train (pyav decodes them fine via libdav1d).

---

## 11. Smoke test (run after any setup change)

```bash
cd /fs/atipa/data/rnd-liu/MyRepo/omniverselab/mylerobot

# A. Unit tests
pytest tests -q   # expect 54 passed (32 base + 22 pi0_voxel)

# B. LIBERO-Plus env loads + renders
MUJOCO_GL=egl python -c "
import mylerobot, os, json
from libero.libero import benchmark, get_libero_path
from libero.libero.envs import OffScreenRenderEnv
suite = benchmark.get_benchmark_dict()['libero_spatial']()
print(f'libero_spatial has {len(suite.tasks)} tasks')
task = suite.tasks[0]
bddl = os.path.join(get_libero_path('bddl_files'), task.problem_folder, task.bddl_file)
env = OffScreenRenderEnv(bddl_file_name=bddl, camera_heights=64, camera_widths=64)
obs = env.reset()
print('env reset OK, agentview pixel sum =', int(obs['agentview_image'].sum()))
env.close()
"

# C. π0 LIBERO baseline (1 task × 1 episode, ~3 min)
MUJOCO_GL=egl python scripts/eval_libero_plus_fragility.py \
  --suites libero_spatial \
  --categories 'Camera Viewpoints' \
  --max_tasks_per_category 1 \
  --max_steps 220 \
  --output_json /tmp/setup_smoke.json
```

If all three pass, the install is healthy.
