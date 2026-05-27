# LIBERO-Plus on H100 — environment, datasets, perturbation mechanics, fragility eval

Focused companion to `SETUP.md`. That doc has the broad-strokes install recipe
for the whole `mylerobot` stack (conda envs, lerobot editable install, openpi,
mylerobot itself). This doc dives into the LIBERO / LIBERO-Plus side specifically:
how the benchmark is laid out on disk, how the perturbation variants reuse base
init-states, what the `is_libero_plus=True` flag actually does, and the exact
eval commands we use day-to-day.

Last updated 2026-05-27.

---

## 1. TL;DR — what you need to know in 30 seconds

LIBERO-Plus is a drop-in replacement for the `libero` pip package that adds
**10,030 perturbation-augmented tasks** spanning 7 perturbation categories
(Camera Viewpoints, Robot Initial States, Objects Layout, Light Conditions,
Background Textures, Sensor Noise, Language Instructions). It ships:
- **16,330** `.bddl` task files (the perturbed scene definitions)
- **4,170** `.pruned_init` init-state files (BASE tasks only; perturbation
  variants reuse them)

The 4:1 BDDL-to-init ratio is the central mechanism: perturbation variants
encode the perturbation *only* in the BDDL filename suffix (e.g.
`pick_up_the_milk_..._light_10.bddl`). On disk only the base init-state file
exists; the loader has to **strip the suffix** to find it. lerobot's
`LiberoEnv` supports this via the `is_libero_plus=True` constructor flag.

If you forget that flag, every perturbation task will raise `FileNotFoundError`
on `env.reset()`.

---

## 2. Setup recipe (LIBERO-Plus only — broader env in SETUP.md)

### 2.1 Prereqs

You already have:
- `py312` conda env with `lerobot==0.4.4` editable + `mylerobot` editable
  (see `SETUP.md` §1–4 if not)
- `imagemagick` from conda-forge (for `libwand`)
- `HF_HOME=/data/rnd-liu/.cache/huggingface` exported

### 2.2 Clone + install LIBERO-Plus (replaces the vanilla `libero` pip package)

```bash
conda activate py312

# (1) Clone — independent of vanilla LIBERO; do not also have libero installed
cd /data/rnd-liu/aiprojects
git clone https://github.com/sylvestf/LIBERO-plus.git

# (2) Remove vanilla libero if it came in as a transitive dep
pip uninstall -y libero

# (3) Editable install. NB: succeeds with empty MAPPING due to a setuptools
#     quirk — `find_packages()` returns []. Install reports success but
#     `import libero` fails.
pip install -e /data/rnd-liu/aiprojects/LIBERO-plus

# (4) Workaround: add the repo to sys.path via a .pth file so `import libero`
#     resolves to LIBERO-plus/libero/
echo "/data/rnd-liu/aiprojects/LIBERO-plus" > \
  /home/010796032/.conda/envs/py312/lib/python3.12/site-packages/libero_plus_path.pth

# (5) Extra pip deps from LIBERO-plus/extra_requirements.txt
pip install wand scikit-image gym
```

### 2.3 Download asset bundle (~6 GB download, ~11 GB extracted)

```bash
/home/010796032/.conda/envs/py312/bin/python -c "
from huggingface_hub import snapshot_download
snapshot_download(repo_id='sii-research/LIBERO_plus_assets', repo_type='dataset',
                  local_dir='/data/rnd-liu/.cache/huggingface/LIBERO_plus_assets')"

mkdir -p /data/rnd-liu/aiprojects/LIBERO-plus/libero/libero/assets_libero_plus_extracted
cd /data/rnd-liu/aiprojects/LIBERO-plus/libero/libero/assets_libero_plus_extracted
unzip -q /data/rnd-liu/.cache/huggingface/LIBERO_plus_assets/assets.zip
```

The zip extracts 9 directory levels deep (the author's filesystem layout got
pickled into the zip metadata). Symlink to the path the loader actually wants:

```bash
SRC=/data/rnd-liu/aiprojects/LIBERO-plus/libero/libero/assets_libero_plus_extracted/inspire/hdd/project/embodied-multimodality/public/syfei/libero_new/release/dataset/LIBERO-plus-0/assets
DST=/data/rnd-liu/aiprojects/LIBERO-plus/libero/libero/assets
ln -sfn "$SRC" "$DST"
```

### 2.4 Point `~/.libero/config.yaml` at LIBERO-Plus

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

### 2.5 Smoke-verify

```bash
MUJOCO_GL=egl /home/010796032/.conda/envs/py312/bin/python -c "
import mylerobot  # MUST come before libero import
from libero.libero import benchmark
suites = list(benchmark.get_benchmark_dict().keys())
print('suites:', suites)
n = len(benchmark.get_benchmark_dict()['libero_spatial']().tasks)
print(f'libero_spatial task count: {n}  (expect 2402 — vanilla LIBERO had 10)')
"
```

If you get 2402, install succeeded.

---

## 3. The perturbation init-state mechanic

This is the part that bit us first. Read carefully.

### 3.1 Task counts per suite × category

From `LIBERO-plus/libero/libero/benchmark/task_classification.json`:

| Suite          | total | Camera | Robot Init | Layout | Light | Background | Noise | Language |
|----------------|-------|--------|------------|--------|-------|------------|-------|----------|
| libero_spatial | 2,402 | 376    | 350        | 385    | 292   | 258        | 351   | 390      |
| libero_object  | 2,518 | 396    | 398        | 403    | 297   | 248        | 422   | 354      |
| libero_goal    | 2,591 | 408    | 409        | 425    | 279   | 281        | 379   | 410      |
| libero_10      | 2,519 | 419    | 393        | 312    | 274   | 289        | 449   | 383      |
| **TOTAL**      | **10,030** | **1,599** | 1,550 | 1,525 | 1,142 | 1,076 | 1,601 | 1,537 |

10,030 perturbed tasks + the 40 base tasks (10 per suite × 4 suites) = the
training distribution.

### 3.2 Filename anatomy

A typical LIBERO-Plus BDDL filename has the form:

```
pick_up_the_milk_and_place_it_in_the_basket_view_0_0_100_2_352_initstate_0.bddl
\___________________ base task _____________________/\__ perturbation suffix __/
```

Perturbation suffix patterns observed (regex in
[lerobot/envs/libero.py:64](../../aiprojects/lerobot/src/lerobot/envs/libero.py#L64)):

```python
_LIBERO_PERTURBATION_SUFFIX_RE = re.compile(
    r"_(?:language|view|light)_[^.]*|_(?:table|tb)_\d+"
)
```

So the recognised suffix prefixes are: `_language_*`, `_view_*` (camera viewpoint),
`_light_*`, `_table_<n>` (background texture), `_tb_<n>` (sensor noise via
imagewand blob). Additional variants live under `libero_newobj/` keyed by
`_add_*` or `_level*` and need a separate code path.

### 3.3 On-disk asymmetry

```
bddl_files/   16,330 .bddl     ← every perturbation variant has its own BDDL
init_files/    4,170 .pruned_init   ← only BASE tasks have init-state files
```

When the env constructs task #N, it reads `task.init_states_file`, which for a
perturbation variant *will not exist on disk under that name*. The loader must
strip the suffix to find the parent base task's init file.

### 3.4 lerobot's `get_task_init_states` — what `is_libero_plus=True` actually does

From [lerobot/envs/libero.py:67-87](../../aiprojects/lerobot/src/lerobot/envs/libero.py#L67-L87):

```python
def get_task_init_states(task_suite, i, is_libero_plus=False):
    task = task_suite.tasks[i]
    filename = Path(task.init_states_file)
    root = Path(get_libero_path("init_states"))

    if not is_libero_plus:
        # Vanilla LIBERO: filename matches a real file on disk
        init_states_path = root / task.problem_folder / filename.name
        return torch.load(init_states_path, weights_only=False)

    # LIBERO-plus: two special-cased branches.

    # (a) New-object / level-difficulty variants — flat array, special directory
    if "_add_" in filename.name or "_level" in filename.name:
        init_states_path = root / "libero_newobj" / task.problem_folder / filename.name
        init_states = torch.load(init_states_path, weights_only=False)
        return init_states.reshape(1, -1)

    # (b) The common case: strip perturbation suffix, fall back to base task's init
    stripped = _LIBERO_PERTURBATION_SUFFIX_RE.sub("", filename.stem) + filename.suffix
    init_states_path = root / task.problem_folder / stripped
    return torch.load(init_states_path, weights_only=False)
```

This function lives in lerobot upstream — *we did not patch it*; we discovered
its existence and made sure to pass `is_libero_plus=True` everywhere we
construct `LiberoEnv`. Sister flag must be set on:

- `LiberoEnv(..., is_libero_plus=True)` (the env constructor) — sets
  `self.is_libero_plus`, which is read on every `reset()`.
- Top-level helper factories `make_libero_envs(..., is_libero_plus=True)` if
  you use them.

### 3.5 Where we set it

[`scripts/eval_libero_plus_fragility.py:252`](scripts/eval_libero_plus_fragility.py#L252)
is the canonical example:

```python
env = LiberoEnv(
    task_suite=suite,
    task_id=task_id,
    task_suite_name=suite_name,
    episode_length=args.max_steps,
    camera_name="agentview_image,robot0_eye_in_hand_image",
    observation_height=256,
    observation_width=256,
    obs_type="pixels_agent_pos",
    episode_index=args.seed + ep_idx,
    is_libero_plus=True,  # use the perturbation-suffix-aware init-state loader
)
```

**If you forget this flag**, the first reset on a perturbation task fails
with `FileNotFoundError: ..._view_0_0_100_2_352_initstate_0.pruned_init`.

### 3.6 What perturbations actually do

A perturbation variant differs from its base task in exactly one of these ways
(everything else — initial scene layout, target objects, base goal predicate —
is identical):

| Suffix prefix         | What changes                                                                 |
|-----------------------|------------------------------------------------------------------------------|
| `_view_<...>_initstate_<n>` | `agentview` camera xpos + xmat (camera viewpoint perturbation) |
| `_light_<n>`          | Lamp colour / position / intensity                                           |
| `_table_<n>`          | Background table texture                                                      |
| `_tb_<n>`             | ImageMagick blob applied to images (sensor noise; "imagewand" pipeline)      |
| `_language_<n>`       | Task instruction phrasing                                                    |
| `_add_<n>` (newobj)   | Extra distractor objects added                                                |
| `_level<n>` (newobj)  | Difficulty tier                                                              |

The init-state file the loader resolves to is the **base task's** layout, so
the robot starts in the same pose with the same object positions; the
perturbation is overlaid on top by the BDDL file.

---

## 4. Datasets

### 4.1 `lerobot/libero` — standard LIBERO training trajectories

```bash
# Already auto-downloaded by lerobot when first referenced.
# Stored under HF_HOME at:
ls /data/rnd-liu/.cache/huggingface/lerobot/hub/datasets--lerobot--libero/
# 1,693 episodes × ~150 frames each, 2 cams (image + image2), fps 20
```

Used as the base training dataset for `pi0_libero_finetuned_v044` and for all
our voxel-fusion fine-tunes that start from v044 (rows 2b / 2d / 2h.2).

⚠ **Gotchas:**
- `meta/stats.json` is missing image stats. Pass `--dataset.use_imagenet_stats=false`
  to lerobot_train, or it crashes in `make_dataset`.
- Videos are AV1-encoded. Pass `--dataset.video_backend=pyav` (torchcodec
  hits "best video stream is unknown" on AV1).

### 4.2 `Sylvest/libero_plus_lerobot` — LIBERO-Plus training trajectories

```bash
# Download (15 GB)
/home/010796032/.conda/envs/py312/bin/python -c "
from huggingface_hub import snapshot_download
snapshot_download(repo_id='Sylvest/libero_plus_lerobot', repo_type='dataset',
                  local_dir='/data/rnd-liu/.cache/huggingface/Sylvest_libero_plus_lerobot')"

# Convert v2.1 → v3.0 (in place; old version renamed _old/)
/home/010796032/.conda/envs/py312/bin/python -m lerobot.scripts.convert_dataset_v21_to_v30 \
  --repo-id Sylvest/libero_plus_lerobot \
  --root /data/rnd-liu/.cache/huggingface/Sylvest_libero_plus_lerobot \
  --push-to-hub false
```

- 14,347 episodes, 2.24M frames @ 20fps
- Camera keys are `observation.images.front` + `observation.images.wrist`
  (different from `lerobot/libero`'s `.image`/`.image2`)
- State is the same 8-d eef_pos+axis_angle+gripper

### 4.3 `local/libero_multicam_v0` — our 5-cam re-render

If you need ground-truth extrinsics in the dataset (for proper voxel-fusion
training), the standard datasets don't have them. We re-rendered libero_spatial
through `pi0_libero_finetuned_v044` with 5 cameras and saved per-cam extrinsics:

```bash
cd /fs/atipa/data/rnd-liu/MyRepo/omniverselab/mylerobot
MUJOCO_GL=egl python scripts/rerender_libero_multicam.py \
    --suite libero_spatial \
    --tasks_per_suite 10 \
    --episodes_per_task 50 \
    --output_repo_id local/libero_multicam_v0
# ETA ~3h. Output at /data/rnd-liu/.cache/lerobot_local/local/libero_multicam_v0/
# 292 successful episodes (~58% success rate from teacher), 30,422 frames.
```

Cameras saved: `agentview`, `wrist` (= `robot0_eye_in_hand`), `front`, `side`,
`bird`. Plus `observation.cam_extrinsics_world2cam_cv.<cam>` and
`observation.cam_intrinsics_K.<cam>` keys for each.

---

## 5. Running the fragility eval

The script `scripts/eval_libero_plus_fragility.py` is the workhorse for every
benchmark number in `PROJECT_STATUS.md` §7.

### 5.1 Baseline: pi0_libero_finetuned_v044 at n=10/cell

```bash
cd /fs/atipa/data/rnd-liu/MyRepo/omniverselab/mylerobot
MUJOCO_GL=egl /home/010796032/.conda/envs/py312/bin/python scripts/eval_libero_plus_fragility.py \
    --policy_path lerobot/pi0_libero_finetuned_v044 \
    --suites libero_spatial libero_object libero_goal libero_10 \
    --max_tasks_per_category 10 \
    --max_steps 220 \
    --output_json /data/rnd-liu/aiprojects/lerobot/outputs/eval/pi0_baseline_n10_fragility.json
```

ETA ~83 min wall on H100. Produces a 4×7 success-rate table.

### 5.2 Long-horizon at the correct step budget

`libero_10` needs more rollout steps than the other suites (default max in
`TASK_SUITE_MAX_STEPS` is 520, not 220). Run it separately:

```bash
MUJOCO_GL=egl python scripts/eval_libero_plus_fragility.py \
    --policy_path lerobot/pi0_libero_finetuned_v044 \
    --suites libero_10 \
    --max_tasks_per_category 10 \
    --max_steps 520 \
    --output_json /data/rnd-liu/aiprojects/lerobot/outputs/eval/pi0_baseline_libero10_ms520.json
```

ETA ~41 min. Fold the libero_10 row from this scan into the all-suites table
for the "fair budget" comparison (PROJECT_STATUS row 1e').

### 5.3 n=50 expansion for CoRL-grade statistics

Wilson 95% CI at n=10 is ±28pp for a 50% success rate — too wide for paper
claims. Expand to n=50 on the headline columns (Camera, Background, Light):

```bash
# Short-horizon suites
MUJOCO_GL=egl python scripts/eval_libero_plus_fragility.py \
    --policy_path <CKPT> \
    --suites libero_spatial libero_object libero_goal \
    --categories "Camera Viewpoints" "Background Textures" "Light Conditions" \
    --max_tasks_per_category 50 \
    --max_steps 220 \
    --output_json <OUT>.json

# libero_10 long-horizon (separate run, ms=520)
MUJOCO_GL=egl python scripts/eval_libero_plus_fragility.py \
    --policy_path <CKPT> \
    --suites libero_10 \
    --categories "Camera Viewpoints" "Background Textures" "Light Conditions" \
    --max_tasks_per_category 50 \
    --max_steps 520 \
    --output_json <OUT>_libero10_ms520.json
```

ETA ~80 min for ms=220 portion + ~50 min for libero_10 ms=520. Total ~2 hours per checkpoint.

### 5.4 Ablation hooks (PI0VoxelPolicy only)

The eval script has two ablation flags for testing the voxel architecture:

```bash
# Extrinsics corruption: replace extrinsics with identity matrices or shuffle
--corrupt_extrinsics identity   # zero geometric info; tests whether voxel uses geometry
--corrupt_extrinsics shuffle    # keeps geometry shape but breaks cam-image pairing

# Camera dropout: zero out one camera's image at every timestep
--drop_cam image      # drops agentview
--drop_cam image2     # drops wrist
```

These were used to produce PROJECT_STATUS rows 2f (corruption) and 2g (dropout).
The corruption ablation revealed that the bolt-on voxel architecture doesn't
use 3D geometry (identity ≈ correct extrinsics at n=50).

### 5.5 Common args reference

| Flag                            | Default                        | Purpose                                     |
|--------------------------------|--------------------------------|---------------------------------------------|
| `--policy_path`                 | `lerobot/pi0_libero_finetuned_v044` | HF repo or local checkpoint            |
| `--suites`                      | all four                       | which LIBERO-Plus suites                    |
| `--categories`                  | all seven                      | which perturbation columns                  |
| `--max_tasks_per_category`      | 10                             | n per cell                                  |
| `--max_steps`                   | 220                            | per-episode step cap                        |
| `--corrupt_extrinsics`          | `none`                         | `identity`/`shuffle` (PI0VoxelPolicy only)  |
| `--drop_cam`                    | `none`                         | `image`/`image2` (any policy)               |
| `--output_json`                 | default path                   | cell-level results json                     |

### 5.6 Comparing two fragility runs

```bash
python scripts/compare_fragility.py \
    --baseline /data/.../pi0_baseline.json \
    --variant  /data/.../pi0voxel_variant.json \
    --label_baseline "pi0_v044" \
    --label_variant  "pi0voxel_v044+10k_b5"
```

Prints a markdown delta table with row Δ and overall mean Δ.

---

## 6. Gotchas (LIBERO-Plus specific — broader ones in SETUP.md §10)

1. **`is_libero_plus=True` flag on `LiberoEnv`** — see §3.5. Without it,
   perturbation-variant tasks raise `FileNotFoundError` on `reset()`.
2. **Vanilla LIBERO must be uninstalled** before installing LIBERO-Plus, or
   imports resolve to the wrong package.
3. **The `.pth` workaround for `import libero`** — LIBERO-Plus's setup.py
   has a `find_packages()` bug that ships an empty MAPPING; the pip install
   succeeds but the import fails. Drop a `.pth` file in site-packages to
   route `import libero` to `LIBERO-plus/libero/`.
4. **9-level-deep asset zip** — the asset bundle extracts into
   `inspire/hdd/project/embodied-multimodality/.../assets/` because the
   author's filesystem layout got pickled into the zip. Symlink to the
   expected path or the env will complain about missing meshes.
5. **AV1 video codec on `lerobot/libero`** — pass `--dataset.video_backend=pyav`.
   torchcodec hits "best video stream is unknown" on AV1.
6. **Missing image stats in `lerobot/libero`** — pass
   `--dataset.use_imagenet_stats=false`. Otherwise lerobot's `make_dataset`
   crashes in `factory.py` because `dataset.meta.stats[key]` doesn't have
   image-feature entries.
7. **MuJoCo headless render** — every script that uses the env needs
   `MUJOCO_GL=egl`; defaults to `glfw` which requires X11.
8. **GPU memory** — pi0 inference needs ~10 GB. Check
   `nvidia-smi --query-gpu=memory.free --format=csv,noheader` before launching;
   parallel BEVDet jobs on the same H100 will OOM the fragility eval at load
   time.

---

## 7. Cross-references

- `SETUP.md` — broader setup (conda envs, lerobot install, openpi reference)
- `PROJECT_STATUS.md` — master results table (rows 1b/1e/1e' baseline,
  2b/2c/2d/2d'/2e'/2f/2g/2h.* voxel experiments)
- `ROAD_B_PLAN.md` — current architectural direction (B-4 + B-5)
- [`scripts/eval_libero_plus_fragility.py`](scripts/eval_libero_plus_fragility.py) — eval driver
- [`scripts/compare_fragility.py`](scripts/compare_fragility.py) — delta-table generator
- [`scripts/rerender_libero_multicam.py`](scripts/rerender_libero_multicam.py) — generates `local/libero_multicam_v0`
