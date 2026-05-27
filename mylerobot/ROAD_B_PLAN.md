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
