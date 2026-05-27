# Road B Design Sketch — Making Voxel-Fusion Actually Use Geometry

Created 2026-05-26 after the n=50 + extrinsics-corruption + camera-dropout
ablations (PROJECT_STATUS §7 rows 2e', 2f, 2g) showed the current
PI0VoxelPolicy is functionally inert as a 3D-grounding mechanism. This
document is a concrete design plan for the three candidate fixes, ranked
by expected leverage.

## Core diagnosis

The action loss has no signal that rewards geometric correctness in the
voxel projection. The action expert can predict perfectly correct actions
whether the voxel tokens were computed with correct extrinsics, identity
extrinsics, or random extrinsics — because the model learns to use voxel
tokens as bonus capacity regardless of their geometric content.

The fix must make extrinsics structurally necessary for the action
expert to be correct.

---

## Candidate 1 — OC-VLA-style camera-frame action prediction (HIGHEST LEVERAGE)

**Reference:** arxiv 2508.13103. Ablation table: novel-viewpoint
zero-shot 54.0% (camera-frame) vs 41.3% (base-frame); camera perturbation
73.8% vs 61.3%.

**Mechanism:** instead of predicting base-frame eef deltas, the policy
predicts deltas expressed in the chosen camera's frame. At inference
time, the predicted delta must be transformed back to base frame using
the camera extrinsic. This makes extrinsics structurally load-bearing:
wrong extrinsic → wrong base-frame command → task failure.

**Concrete implementation for our pipeline:**

1. **Pick a reference camera.** Use `observation.images.image` (agentview)
   since it's the only stable world-frame camera in LIBERO. Wrist (image2)
   is rigid to eef and would create circular dependency.

2. **Training data transform.** In `mylerobot/policies/pi0_voxel/`,
   add a wrapper preprocessor that for each batch:
   - Reads `extrinsics["agentview"]` (R_w2c, t_w2c) — the world→cam transform
   - Original action: `a_world = [dx, dy, dz, drx, dry, drz, gripper]` in base frame
   - Translation part: `[dx, dy, dz]_cam = R_w2c @ [dx, dy, dz]_world`
   - Rotation part: convert axis-angle `[drx, dry, drz]_world` to rotation matrix
     `dR_world`, then `dR_cam = R_w2c @ dR_world @ R_w2c.T`, then back to axis-angle
   - Gripper: unchanged
   - Replace `batch["action"]` with the cam-frame version

3. **Inference inverse transform.** In the rollout loop (eval_libero_plus_fragility.py),
   after policy outputs cam-frame action:
   - Read extrinsics from the env's current state
   - Invert the transformation: `[dx, dy, dz]_world = R_cam2world @ [dx, dy, dz]_cam`
   - Same for rotation
   - Send the world-frame action to env

4. **Where to put the transform.**
   - Cleanest: a new `PI0VoxelOCPolicy` class that wraps PI0VoxelPolicy and
     overrides `forward` (training) and `predict_action_chunk` (inference)
     to apply the transforms.
   - Or: keep it inside PI0VoxelPolicy with a config flag
     `action_frame: "world" | "agentview_cam"`.

5. **Sanity tests for the transform:**
   - Round-trip: cam→world→cam reproduces input action.
   - At zero extrinsic: cam-frame = world-frame (identity transform).
   - Synthetic 90° camera rotation: action [1, 0, 0]_world → [0, 0, -1]_cam
     (or similar; verify orientation conventions).

6. **Extrinsics-corruption re-test.** After retrain, re-run the row 2f
   identity/shuffle ablation. If voxel is using geometry, identity
   extrinsics will now catastrophically fail (model trained to predict
   cam-frame actions, but using identity extrinsic → wrong reference frame
   → wrong actions).

**Estimated effort:**
- Transform helper functions + tests: 1 day
- Training-side wrapper: 1 day
- Inference-side wrapper: 0.5 day
- Smoke train + debug: 1 day
- Full 10k retrain: ~50 min compute
- Full fragility + corruption ablation: ~3 hours compute
- **Total: 3-4 days of dev + ~4h compute**

**Risk:** action distribution in cam-frame may have higher variance
(different cameras see different action distributions), making training
harder. If so, may need to predict actions in eef-frame instead
(eef pose is in `observation.state[:3]` and is "camera-independent" in a
different way).

---

## Candidate 2 — PointACT-style dual-system routing (MEDIUM LEVERAGE, REPRODUCES PRIOR FIX)

**Reference:** arxiv 2605.21414. Showed monolithic 3D-token VLA fusion
fails (18.6% vs 73.2% 2D-only), dual-system architecture fixes it.

**Mechanism:** route voxel tokens directly into the action expert
(gemma_300m) via cross-attention, bypassing PaliGemma entirely. The
PaliGemma prefix stays at [image tokens, language tokens]; voxel tokens
join the action expert's input as a separate stream.

**Why this might help:** PaliGemma is pretrained on 2D+language and may
actively under-weight voxel-prefix tokens that don't look like image or
language tokens. The action expert is trained from scratch and would
learn to use voxel tokens *as voxel tokens*.

**Concrete implementation:**

1. Remove voxel token concatenation from `PI0VoxelPytorch.embed_prefix`.
2. Modify the action expert forward to accept additional voxel context:
   look at `gemma_300m.forward` and add a cross-attention layer between
   action token positions and voxel tokens.
3. Action expert in lerobot's pi0 is `self.paligemma_with_expert.expert`;
   inspect to find the insertion point.

**Estimated effort:** 4-5 days. More invasive than Candidate 1 because
it touches the action expert internals.

**Risk:** lerobot's `paligemma_with_expert` may not have a clean hook
point. May require monkey-patching or partial fork of the action expert.

---

## Candidate 3 — Auxiliary geometric supervision (LOWEST LEVERAGE, EASIEST)

**Mechanism:** add a head on voxel features that predicts geometric
quantities. The aux loss provides direct geometric supervision.

**Concrete heads (in order of supervision availability):**

- **State prediction**: predict `observation.state[:3]` (eef position) from
  voxel features. GT comes from the dataset for free.
- **Image-coordinates back-projection**: given a voxel position, predict
  where it should project on each camera. Loss = L1 between predicted
  pixel and the actual extrinsic-computed pixel. No external GT needed.
- **Depth from each cam**: predict depth-image from voxel features per cam.
  Requires depth GT — not available from current LIBERO replays. Would
  need re-rendering with depth.

**Estimated effort:** 1-2 days for state prediction or back-projection
heads.

**Risk:** aux loss may not be high enough weight to dominate the action
loss's "use as capacity" gradient. May not move the needle.

---

## Recommended sequence

1. **Week 1**: Implement Candidate 1 (OC-VLA-style camera-frame actions).
   This is the highest-leverage and best-precedented option. Test it on
   the libero_spatial × Camera + libero_10 × Camera cells with n=50, plus
   extrinsics-corruption re-test. If identity extrinsics now hurt
   significantly, the architecture is fixed and we have a paper.

2. **Week 2 (only if Week 1 succeeds)**: Add Candidate 3 (back-projection
   aux loss) on top. Cheap, may strengthen the geometric grounding.

3. **Week 3 (only if Weeks 1+2 don't yield enough)**: Consider Candidate 2
   (dual-system routing). High effort, but addresses the same null result
   PointACT identified.

**Stop conditions (kill the line of work):**
- After Candidate 1: if extrinsics-corruption still shows identity ≈ correct,
  the action-frame fix didn't work and Candidates 2 + 3 are unlikely to.
- If voxel+augs at n=50 is still worse than augs-only on the headline
  cells, the entire 3D-grounding hypothesis on this benchmark is wrong;
  pivot to a different claim.

## Decision points before starting

These need a quick check before sinking days into implementation:

1. **Are LIBERO actions truly base-frame eef deltas?** Verify by reading
   one episode of `lerobot/libero` and confirming `action[:3]` are
   small per-step world-frame translations.
2. **What's the camera-frame distribution variance vs base-frame?** A quick
   sanity check: load 1000 sample actions, transform to cam-frame, compare
   std and range. If much larger, training will be harder.
3. **Can we get the agentview extrinsic at every dataset step?** Currently
   we synthesize it from `observation.state` (via `derive_libero_extrinsics_batch`).
   Need to confirm that synthesis is accurate enough that round-trip
   world→cam→world has < 1mm error.
