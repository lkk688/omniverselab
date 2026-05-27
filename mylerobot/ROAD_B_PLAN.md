# Road B — Architectural Fixes for Voxel Module That Doesn't Use Geometry

Created 2026-05-26 after empirical evidence (PROJECT_STATUS rows 2e' through 2h.3)
that the monolithic-prefix voxel-fusion architecture cannot learn 3D-grounded
representations from action loss alone.

This document supersedes `ROAD_B_DESIGN.md` (written before the n=50 ablations
and extrinsics-corruption results were in). All option rankings reflect what
we now know about our specific failure mode.

---

## What we know (after killing the monolithic-prefix approach)

1. **Voxel cross-attention output concatenated onto PaliGemma's prefix is functionally inert as 3D grounding.** Identity extrinsics ≈ correct extrinsics, even after retraining with geometrically correct projections (rows 2h.2 + 2h.3).

2. **The action loss alone provides no gradient signal that rewards geometric correctness in voxel features.** The action expert adapts to whatever the voxel module outputs, including random projections.

3. **The architecture HAS projection-dependence at the input level** (libero_10 broken-trained drops 24pp when switched to fixed-correct at eval), but learns it as "use the projections you saw during training" rather than "use 3D geometry."

4. **This is the same null result PointACT documented** in their paper (RLBench 18.6% monolithic prefix-fusion of 3D tokens vs 73.2% 2D-only), and the same one OC-VLA implicitly addresses with a different mechanism (camera-frame action prediction).

---

## Three candidate fixes (B-3, B-4, B-5)

### B-4: PointACT-style dual-system routing — ROUTE VOXEL → ACTION EXPERT, BYPASS LLM

**Reference:** arxiv 2605.21414. Their fix for our exact failure mode.

**Architecture change:** voxel tokens enter the action expert (`gemma_300m`) via a new cross-attention layer, rather than being concatenated to PaliGemma's prefix. The image + language tokens stay in the PaliGemma prefix as before; voxel is a separate stream consumed only by the action head.

**Why it might work for us:**
- Direct empirical precedent for our exact failure mode
- PaliGemma is pretrained on 2D+language and likely actively under-weights voxel-prefix tokens
- Action expert is trained from scratch on action prediction → would learn to use voxel as voxel

**Expected effort:** 4–5 days. Most invasive option — requires modifying lerobot's `paligemma_with_expert.expert` internals.

**Risks:**
- lerobot's action expert may not have clean injection point; might require partial fork
- Cross-attention to voxel adds parameters that need to learn from scratch

---

### B-5: Auxiliary geometric supervision — STATE PREDICTION HEAD ON VOXEL FEATURES — STARTING NOW

**Reference:** OC-VLA and PointACT both have aux 3D losses; specific form here is state-prediction (cheapest).

**Architecture change:** add a small head on pooled voxel features that predicts `observation.state[:3]` (EEF position in world frame). Aux loss = MSE between predicted and true state. The action loss + aux loss are added with a tunable weight.

**Why it might work for us:**
- Provides the inductive bias we proved is missing — voxel features MUST encode 3D structure to satisfy the aux loss
- State[:3] is in the dataset for free, no new GT needed
- Cheap: 1–2 days
- Composes with B-4 (can use both simultaneously)

**Expected effort:** 1–2 days.

**Risks:**
- Aux loss may converge to some "fake" representation that satisfies the head without actually being usable by the action expert
- Pooling strategy matters (mean vs max vs attention pool)
- Loss weight needs tuning

**Diagnostic value:** if aux loss converges → architecture has 3D capacity, just lacked the gradient signal. If aux loss won't converge → architecture is fundamentally incapable. Either result is informative.

---

### B-3: OC-VLA-style camera-frame action prediction — SKIPPED FOR NOW

**Reference:** arxiv 2508.13103. Camera-frame action heads on novel viewpoints.

**Architecture change:** instead of predicting base-frame eef deltas, predict deltas expressed in the chosen camera's frame. At inference, transform back to base-frame using extrinsics.

**Why we're skipping it:**

OC-VLA fixes a different problem than ours. OC-VLA addresses "policy ignores extrinsics during action decoding"; our problem is "voxel module never produces geometric features in the first place." OC-VLA forces the action expert to use camera-frame, but the voxel module's lack of inductive bias is upstream of that.

Also: LIBERO actions are normalized to [−1, +1] (Check 1 result), so applying OC-VLA requires unnormalize → frame transform → renormalize. This complicates a fix that targets the wrong layer.

**When we might revisit B-3:** if B-4 + B-5 both fail, or if B-4 + B-5 succeed but the model still has a separable "action expert ignores geometry" issue.

---

## Recommended execution order

### Week 1 (now)
- **Day 1–2 (in progress):** B-5 — implement state-prediction aux head. Train one variant 10k from v044 with `aux_state_pred_weight=0.1`. Verify (a) aux loss converges, (b) action performance ≥ baseline.
- **Day 3–5:** B-4 — implement dual-system routing. Modify PI0VoxelPytorch to route voxel tokens into the action expert via cross-attention instead of the PaliGemma prefix.

### Week 2
- **Day 6–8:** Train the B-4 + B-5 combined model (dual-system + aux supervision). 10k from v044.
- **Day 9–10:** Re-run extrinsics-corruption + n=50 fragility on the B-4 + B-5 model. Compare to the dead monolithic-prefix baseline.

### Stop condition for the whole Road B effort

If the B-4 + B-5 model's extrinsics-corruption test STILL shows identity ≈ correct on libero_spatial × Camera (n=50), the 3D-grounding hypothesis on this benchmark is conclusively dead. Pivot the paper to a different claim:
- "Voxel-augmented VLA: a negative-result study on LIBERO-Plus" (workshop)
- Or pivot to a different mechanism (language-conditioning, multi-task fine-tuning, etc.)

---

## Composability

B-4 and B-5 are designed to compose:
- B-4 changes WHERE voxel tokens enter the network (action expert instead of LLM prefix)
- B-5 changes WHAT signal trains the voxel module (action loss + aux geometric loss)

The full architecture after both: voxel features computed by VoxelCrossAttnFusion → (a) state-prediction head with aux loss + (b) cross-attention into action expert. Either by itself may be insufficient; together they directly attack both the routing problem (B-4) and the gradient-signal problem (B-5).

---

## B-5 implementation (2026-05-26)

### Code changes already landed in commit 38eb93a

1. **`configuration_pi0_voxel.py`** — added two fields:
   - `aux_state_pred_weight: float = 0.0` (default off; set > 0 to enable)
   - `aux_state_pool: str = "mean"` (pooling strategy; only "mean" implemented)

2. **`modeling_pi0_voxel.py`**:
   - `PI0VoxelPytorch.__init__`: builds `aux_state_head = Linear(C → C/4) → ReLU → Linear(C/4 → 3)` when `aux_state_pred_weight > 0`. Built lazily so disabled runs have unchanged state_dict.
   - `PI0VoxelPytorch.embed_prefix`: stashes computed voxel_tokens on `self._last_voxel_tokens` when aux head exists.
   - `PI0VoxelPolicy.forward`: mean-pools the stashed voxel tokens, runs the aux head, computes MSE vs `observation.state[:3]`, adds `aux_w * aux_loss` to total loss. Logs `aux_state_loss` and `aux_state_l1` to the output dict.

3. **`tests/test_pi0_voxel.py`** — 1 new test (default 0.0 + accepts custom weight). 57/57 tests pass.

### Smoke train recipe (run before full 10k to verify wiring)

```bash
cd /fs/atipa/data/rnd-liu/MyRepo/omniverselab/mylerobot
nvidia-smi --query-gpu=memory.free --format=csv,noheader   # require ≥12 GB

PYTORCH_ALLOC_CONF=expandable_segments:True \
/home/010796032/.conda/envs/py312/bin/python scripts/train_act.py \
    --dataset.repo_id=lerobot/libero \
    --dataset.video_backend=pyav \
    --dataset.use_imagenet_stats=false \
    --dataset.image_transforms.enable=true \
    --policy.type=pi0_voxel \
    --policy.pretrained_path=lerobot/pi0_libero_finetuned_v044 \
    --policy.train_expert_only=true \
    --policy.push_to_hub=false \
    --policy.voxel_camera_keys='["observation.images.image","observation.images.image2"]' \
    --policy.aux_state_pred_weight=0.1 \
    --output_dir=/data/rnd-liu/aiprojects/lerobot/outputs/train/2026-05-27/pi0voxel_b5_smoke \
    --job_name=pi0voxel_b5_smoke \
    --steps=10 \
    --batch_size=2 \
    --num_workers=2 \
    --save_freq=100 \
    --log_freq=1 \
    --wandb.enable=false \
    2>&1 | tee /tmp/pi0voxel_b5_smoke.log
```

**Pass criteria for smoke:**
- No NaN / OOM / crash
- `aux_state_loss` and `aux_state_l1` appear in log output (confirms aux head running)
- `aux_state_l1` starts ≲ 0.5 m (workspace radius) — sanity check that head outputs are in the right ballpark

### Full 10k retrain recipe (after smoke passes)

```bash
PYTORCH_ALLOC_CONF=expandable_segments:True \
/home/010796032/.conda/envs/py312/bin/python scripts/train_act.py \
    --dataset.repo_id=lerobot/libero \
    --dataset.video_backend=pyav \
    --dataset.use_imagenet_stats=false \
    --dataset.image_transforms.enable=true \
    --policy.type=pi0_voxel \
    --policy.pretrained_path=lerobot/pi0_libero_finetuned_v044 \
    --policy.train_expert_only=true \
    --policy.push_to_hub=false \
    --policy.voxel_camera_keys='["observation.images.image","observation.images.image2"]' \
    --policy.aux_state_pred_weight=0.1 \
    --output_dir=/data/rnd-liu/aiprojects/lerobot/outputs/train/2026-05-27/pi0voxel_b5_10k \
    --job_name=pi0voxel_b5_10k \
    --steps=10000 \
    --batch_size=4 \
    --num_workers=4 \
    --save_freq=2000 \
    --log_freq=100 \
    --wandb.enable=false \
    > /tmp/pi0voxel_b5_10k.log 2>&1 &
```

ETA ~50 min on H100 with ≥ 12 GB free.

### Extrinsics-corruption test on B-5 checkpoint (the dispositive experiment)

After the 10k retrain finishes, run the 4-way corruption test on the new checkpoint. **The answer to "does B-5 work" is:** does `correct` now beat `identity` by more than the n=50 Wilson CI (~14 pp)?

```bash
CKPT=/data/rnd-liu/aiprojects/lerobot/outputs/train/2026-05-27/pi0voxel_b5_10k/checkpoints/last/pretrained_model

# 1. Correct extrinsics, libero_spatial Camera
MUJOCO_GL=egl /home/010796032/.conda/envs/py312/bin/python scripts/eval_libero_plus_fragility.py \
    --policy_path $CKPT \
    --suites libero_spatial --categories "Camera Viewpoints" \
    --max_tasks_per_category 50 --max_steps 220 \
    --output_json /data/rnd-liu/aiprojects/lerobot/outputs/eval/extrinsics_corruption/pi0voxel_B5_n50_spatial_camera_correct.json

# 2. Identity extrinsics
MUJOCO_GL=egl /home/010796032/.conda/envs/py312/bin/python scripts/eval_libero_plus_fragility.py \
    --policy_path $CKPT \
    --suites libero_spatial --categories "Camera Viewpoints" \
    --max_tasks_per_category 50 --max_steps 220 \
    --corrupt_extrinsics identity \
    --output_json /data/rnd-liu/aiprojects/lerobot/outputs/eval/extrinsics_corruption/pi0voxel_B5_n50_spatial_camera_identity.json
```

**Decision based on outcome:**
- If `correct − identity > +14 pp` (outside CI) → B-5 worked. Architecture *can* use geometry given the right supervision. Move to B-4 (dual-system routing) for further gains.
- If `correct ≈ identity` (within ±14 pp) → B-5 didn't move the needle. The aux loss converges but the action expert ignores the now-geometric voxel features anyway. Skip directly to B-4 (re-route to action expert is required, not just better supervision).
- If `correct < identity` (significantly) → architecture actively hurt by geometry. Investigate aux head training (probably a bug or the wrong supervision).
