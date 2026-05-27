"""Voxel-grid cross-attention vision fusion module.

PyTorch port of `BEVDet/cross_attn_lss2.py`'s `CrossAttnLSSTransform` adapted
to a 3D voxel grid (instead of 2D BEV plane + K shallow depth anchors).

What changes vs the BEV version
-------------------------------
BEV (Hy × Hx queries × K depth anchors):
  per query: K*N_cam feature samples → attention pool → 1 feature per query
  output: (B, out_channels, Hy, Hx) BEV map

Voxel (Hz × Hy × Hx queries, K=1):
  per query: N_cam feature samples → attention pool → 1 feature per query
  output: (B, N_tokens, out_channels) token sequence ready for PaliGemma

K=1 because each voxel has a single known 3D position — no shallow-depth
ambiguity to resolve. This is strictly simpler than BEV.

Memory contract preserved from BEVDet:
  - chunked processing of queries (`attn_chunk`)
  - per-camera embedding (learned bias)
  - multi-head + GQA (Grouped-Query Attention)
  - visibility masking for queries with no valid camera projection

Conventions
-----------
- World frame: arbitrary metric origin.
- Camera extrinsics: world→cam, OpenCV convention (cam looks down +Z, +Y down).
  This matches what `mylerobot/sim/aloha_multicam.py` produces and what
  `isaac_multicam_addons.cam_world_pose_to_world2cam_cv` produces.
- Camera intrinsics: 3×3 K matrix in pixel units of the image features
  (NOT the original image — caller must scale K if features are downsampled).
- Voxel grid: axis-aligned in world frame, centred per cell.
"""

from __future__ import annotations

import math
from typing import Sequence

import torch
import torch.nn as nn
import torch.nn.functional as F


class VoxelCrossAttnFusion(nn.Module):
    """3D voxel queries × per-camera image features → token sequence.

    Args:
        feat_channels: per-camera 2D feature channel dim (e.g. SigLIP output dim).
        out_channels:  output token dim (= PaliGemma hidden size, e.g. 2048).
        voxel_resolution: (Hz, Hy, Hx) — voxel grid resolution.
        workspace_bounds: ((xmin, ymin, zmin), (xmax, ymax, zmax)) in world metres.
        num_heads: query heads for cross-attention. 1 = projection-free path
            (cheapest, matches BEVDet's "original" path).
        num_kv_groups: GQA grouping. 1 = standard MHA; >1 = reduce K/V heads.
        attn_chunk: max queries processed per attention pass (memory cap).
        max_cams: upper bound for per-camera learned embedding table.
        out_tokens: final token count after spatial pooling. If equal to
            Hz*Hy*Hx no pooling happens; otherwise we apply a strided 3D conv.
            Default 128 keeps the token budget similar to a SigLIP image.

    Forward inputs:
        image_features: list[Tensor] of length N_cam, each (B, Cf, Hf, Wf).
        intrinsics:     (B, N_cam, 3, 3) feature-space K matrices (caller scales
                        from original-pixel K if features are downsampled).
        extrinsics:     (B, N_cam, 4, 4) world→cam in OpenCV convention.
        cam_mask:       (B, N_cam) bool, True where the camera is valid.
                        Used to ignore wrist cams that are out of view, etc.

    Forward output:
        tokens of shape (B, out_tokens, out_channels).
    """

    def __init__(
        self,
        feat_channels: int,
        out_channels: int,
        voxel_resolution: tuple[int, int, int],
        workspace_bounds: tuple[tuple[float, float, float], tuple[float, float, float]],
        num_heads: int = 4,
        num_kv_groups: int = 1,
        attn_chunk: int = 4096,
        max_cams: int = 12,
        out_tokens: int = 128,
        dropout: float = 0.0,
        zero_init_out_proj: bool = True,
    ):
        super().__init__()
        self.feat_channels = feat_channels
        self.out_channels = out_channels
        self.voxel_resolution = tuple(voxel_resolution)
        Hz, Hy, Hx = self.voxel_resolution
        assert all(d > 0 for d in self.voxel_resolution)
        (xmin, ymin, zmin), (xmax, ymax, zmax) = workspace_bounds
        assert xmax > xmin and ymax > ymin and zmax > zmin
        self.workspace_bounds = workspace_bounds

        self.num_heads = int(num_heads)
        self.num_kv_groups = int(num_kv_groups)
        self.use_multihead = self.num_heads > 1
        if self.use_multihead:
            assert feat_channels % self.num_heads == 0, (
                f"feat_channels={feat_channels} must be divisible by num_heads={self.num_heads}"
            )
            assert self.num_heads % self.num_kv_groups == 0, (
                f"num_heads={self.num_heads} must be divisible by num_kv_groups={self.num_kv_groups}"
            )
            self.head_dim = feat_channels // self.num_heads
            self.num_kv_heads = self.num_heads // self.num_kv_groups
            kv_proj_dim = self.num_kv_heads * self.head_dim
            self.wq = nn.Linear(feat_channels, feat_channels, bias=False)
            self.wk = nn.Linear(feat_channels, kv_proj_dim, bias=False)
            self.wv = nn.Linear(feat_channels, kv_proj_dim, bias=False)
            self.wo = nn.Linear(feat_channels, feat_channels, bias=False)
            self.attn_scale_mh = 1.0 / math.sqrt(self.head_dim)
        else:
            self.head_dim = feat_channels
            self.num_kv_heads = 1
            self.wq = self.wk = self.wv = self.wo = None
            self.attn_scale_mh = 1.0 / math.sqrt(feat_channels)

        self.attn_chunk = int(attn_chunk)
        self.max_cams = int(max_cams)
        self.cam_embed = nn.Embedding(self.max_cams, feat_channels)

        # Learnable positional embedding for voxel queries: maps voxel XYZ (normalized
        # to [-1, 1]) to feat_channels.
        self.pos_embed = nn.Sequential(
            nn.Linear(3, feat_channels),
            nn.ReLU(inplace=True),
            nn.Linear(feat_channels, feat_channels),
        )

        self.dropout = nn.Dropout(dropout)

        # --- Output projection / pooling ---
        self.n_voxels = Hz * Hy * Hx
        self.out_tokens = int(out_tokens)
        if self.out_tokens == self.n_voxels:
            # No spatial pooling; just channel projection.
            self.spatial_pool = nn.Identity()
            self.out_proj = nn.Linear(feat_channels, out_channels)
        else:
            # Pool via a single 3D conv with stride; then flatten + project.
            # Choose stride per axis so that product of output sizes = out_tokens.
            self.spatial_pool, pooled_shape = _make_pool_conv(
                feat_channels, self.voxel_resolution, self.out_tokens
            )
            self._pooled_shape = pooled_shape
            self.out_proj = nn.Linear(feat_channels, out_channels)

        if zero_init_out_proj:
            # Voxel-token contribution starts at zero so the prefix matches
            # baseline π0 at step 0. Standard ControlNet/LoRA/adapter trick:
            # the action expert can't be poisoned by random voxel garbage
            # early in training; it learns to use voxel tokens only when they
            # help. Tests that need a "live" module pass False.
            nn.init.zeros_(self.out_proj.weight)
            nn.init.zeros_(self.out_proj.bias)

        # Cached voxel centres (registered as buffer so it moves with .to(device))
        self.register_buffer(
            "_voxel_centres",
            _build_voxel_grid(self.voxel_resolution, self.workspace_bounds),
            persistent=False,
        )

    # ------------------------------------------------------------------
    # core forward
    # ------------------------------------------------------------------
    def forward(
        self,
        image_features: Sequence[torch.Tensor],
        intrinsics: torch.Tensor,
        extrinsics: torch.Tensor,
        cam_mask: torch.Tensor | None = None,
        workspace_translate: torch.Tensor | None = None,
    ) -> torch.Tensor:
        """
        Args (additions to original):
            workspace_translate: (B, 3) world-frame offset applied to every
                voxel position before projection. Use this to make the
                workspace follow the robot (e.g. centred on observation.state
                eef position). When None, voxel grid stays at the canonical
                bounds passed at construction.
        """
        if len(image_features) == 0:
            raise ValueError("image_features list is empty")
        B = image_features[0].shape[0]
        N_cam = len(image_features)
        device = image_features[0].device
        dtype = image_features[0].dtype

        # Sanity-check shapes
        for i, f in enumerate(image_features):
            assert f.dim() == 4 and f.shape[1] == self.feat_channels, (
                f"image_features[{i}] expected (B, {self.feat_channels}, Hf, Wf), got {f.shape}"
            )
        assert intrinsics.shape == (B, N_cam, 3, 3), intrinsics.shape
        assert extrinsics.shape == (B, N_cam, 4, 4), extrinsics.shape
        if cam_mask is not None:
            assert cam_mask.shape == (B, N_cam), cam_mask.shape

        Hz, Hy, Hx = self.voxel_resolution
        Q = self.n_voxels

        # Voxel XYZ centres in world frame, (Q, 3)
        voxel_xyz = self._voxel_centres.view(Q, 3).to(device=device, dtype=dtype)

        # Pre-compute pos-embed for ALL voxel queries (cheap; can be cached).
        (xmin, ymin, zmin), (xmax, ymax, zmax) = self.workspace_bounds
        x_norm = (voxel_xyz[:, 0] - xmin) / max(xmax - xmin, 1e-6) * 2 - 1
        y_norm = (voxel_xyz[:, 1] - ymin) / max(ymax - ymin, 1e-6) * 2 - 1
        z_norm = (voxel_xyz[:, 2] - zmin) / max(zmax - zmin, 1e-6) * 2 - 1
        norm_xyz = torch.stack([x_norm, y_norm, z_norm], dim=-1)  # (Q, 3)
        q_embed_all = self.pos_embed(norm_xyz).unsqueeze(0).expand(B, -1, -1)  # (B, Q, C)

        # Output buffer
        out_voxel_feats = torch.empty((B, Q, self.feat_channels), device=device, dtype=dtype)

        if workspace_translate is not None:
            assert workspace_translate.shape == (B, 3), workspace_translate.shape

        # Process queries in chunks to cap memory.
        for i in range(0, Q, self.attn_chunk):
            j = min(Q, i + self.attn_chunk)
            q_len = j - i

            voxel_xyz_chunk = voxel_xyz[i:j]  # (q_len, 3) — canonical (LOCAL) frame
            q_embed = q_embed_all[:, i:j, :]  # (B, q_len, C)

            # Apply per-batch workspace translation if provided. Position
            # embedding (q_embed) intentionally stays in canonical frame so
            # voxel queries have consistent identity across batches; only
            # the projection geometry shifts to "follow the robot".
            if workspace_translate is not None:
                voxel_xyz_chunk_proj = (
                    voxel_xyz_chunk.unsqueeze(0)
                    + workspace_translate.to(voxel_xyz_chunk.dtype).unsqueeze(1)
                )  # (B, q_len, 3)
            else:
                voxel_xyz_chunk_proj = voxel_xyz_chunk

            # Project voxel centres into each camera.
            sampled, valid_mask = _project_and_sample(
                voxel_xyz_chunk_proj, image_features, intrinsics, extrinsics
            )
            # sampled:    (B, q_len, N_cam, C)
            # valid_mask: (B, q_len, N_cam) bool

            # Apply optional cam_mask (e.g. wrist cams that are out of view).
            if cam_mask is not None:
                # cam_mask: (B, N_cam) -> broadcast over q_len
                valid_mask = valid_mask & cam_mask.unsqueeze(1)

            # Add per-camera bias.
            cam_ids = torch.arange(N_cam, device=device) % self.max_cams
            cam_bias = self.cam_embed(cam_ids).view(1, 1, N_cam, self.feat_channels).to(sampled.dtype)
            sampled = sampled + cam_bias

            # Cross-attention: voxel-query × camera-tokens
            fused = self._attend(q_embed, sampled, valid_mask)  # (B, q_len, C)
            out_voxel_feats[:, i:j, :] = fused

        # Reshape to (B, C, Hz, Hy, Hx) for spatial pooling.
        feat_3d = out_voxel_feats.transpose(1, 2).reshape(B, self.feat_channels, Hz, Hy, Hx)

        if isinstance(self.spatial_pool, nn.Identity):
            pooled = feat_3d
            pooled_flat = out_voxel_feats  # (B, Q, C)
        else:
            pooled = self.spatial_pool(feat_3d)  # (B, C, Hz', Hy', Hx')
            pooled_flat = pooled.flatten(2).transpose(1, 2)  # (B, N_tokens, C)

        tokens = self.out_proj(pooled_flat)  # (B, N_tokens, out_channels)
        return tokens

    # ------------------------------------------------------------------
    # attention helpers
    # ------------------------------------------------------------------
    def _attend(
        self,
        q_embed: torch.Tensor,   # (B, q_len, C)
        sampled: torch.Tensor,   # (B, q_len, N_cam, C)
        valid_mask: torch.Tensor,  # (B, q_len, N_cam) bool
    ) -> torch.Tensor:
        B, q_len, N_cam, C = sampled.shape
        if self.use_multihead:
            H_q = self.num_heads
            G = self.num_kv_groups
            H_kv = self.num_kv_heads
            d_h = self.head_dim

            q_proj = self.wq(q_embed).view(B, q_len, H_q, d_h)
            kv_in = sampled  # (B, q_len, N_cam, C)
            k_proj = self.wk(kv_in).view(B, q_len, N_cam, H_kv, d_h)
            v_proj = self.wv(kv_in).view(B, q_len, N_cam, H_kv, d_h)

            if G > 1:
                k_proj = k_proj.repeat_interleave(G, dim=3)
                v_proj = v_proj.repeat_interleave(G, dim=3)

            scores = torch.einsum("bqhd,bqnhd->bqhn", q_proj, k_proj) * self.attn_scale_mh
            mask_b = valid_mask.unsqueeze(2)  # (B, q_len, 1, N_cam)
            scores = scores.masked_fill(~mask_b, -1e9)
            attn = torch.softmax(scores, dim=-1)
            attn = self.dropout(attn)
            out = torch.einsum("bqhn,bqnhd->bqhd", attn, v_proj)
            out = out.reshape(B, q_len, C)
            fused = self.wo(out)
        else:
            scores = (q_embed.unsqueeze(2) * sampled).sum(dim=-1) * self.attn_scale_mh
            scores = scores.masked_fill(~valid_mask, -1e9)
            attn = torch.softmax(scores, dim=-1)  # (B, q_len, N_cam)
            attn = self.dropout(attn)
            fused = (attn.unsqueeze(-1) * sampled).sum(dim=2)  # (B, q_len, C)

        # Zero out queries with no valid camera (avoid garbage from softmax over all -inf).
        none_valid = (~valid_mask).all(dim=2, keepdim=True)  # (B, q_len, 1)
        fused = fused.masked_fill(none_valid, 0.0)
        return fused


# ============================================================================
# Helpers
# ============================================================================
def _build_voxel_grid(
    res: tuple[int, int, int],
    bounds: tuple[tuple[float, float, float], tuple[float, float, float]],
) -> torch.Tensor:
    """Return (Hz, Hy, Hx, 3) of voxel-centre XYZ coordinates in world frame."""
    Hz, Hy, Hx = res
    (xmin, ymin, zmin), (xmax, ymax, zmax) = bounds
    dx = (xmax - xmin) / Hx
    dy = (ymax - ymin) / Hy
    dz = (zmax - zmin) / Hz
    xs = torch.linspace(xmin + dx / 2, xmax - dx / 2, Hx)
    ys = torch.linspace(ymin + dy / 2, ymax - dy / 2, Hy)
    zs = torch.linspace(zmin + dz / 2, zmax - dz / 2, Hz)
    zz, yy, xx = torch.meshgrid(zs, ys, xs, indexing="ij")
    return torch.stack([xx, yy, zz], dim=-1)  # (Hz, Hy, Hx, 3)


def _project_and_sample(
    voxel_xyz: torch.Tensor,           # (q_len, 3) OR (B, q_len, 3)
    image_features: Sequence[torch.Tensor],  # list of (B, C, Hf, Wf)
    intrinsics: torch.Tensor,          # (B, N_cam, 3, 3) — in feature pixel units
    extrinsics: torch.Tensor,          # (B, N_cam, 4, 4) — world->cam OpenCV
) -> tuple[torch.Tensor, torch.Tensor]:
    """Project voxel centres into each camera and bilinear-sample features.

    Args:
        voxel_xyz: voxel positions in world frame. Shape (q_len, 3) for the
            shared-across-batch case (default), or (B, q_len, 3) for the
            per-batch case used when the workspace grid is translated by a
            batch-dependent offset (e.g. eef-centred bounds).

    Returns:
        sampled: (B, q_len, N_cam, C)
        valid:   (B, q_len, N_cam) bool — True if projection is in front of cam
                 and lies within the feature map.
    """
    B, N_cam = intrinsics.shape[:2]
    C = image_features[0].shape[1]
    device = voxel_xyz.device
    dtype_x = image_features[0].dtype

    if voxel_xyz.dim() == 2:
        # Broadcast (q_len, 3) -> (B, q_len, 3) for uniform downstream code.
        voxel_xyz_b = voxel_xyz.unsqueeze(0).expand(B, -1, -1)
    elif voxel_xyz.dim() == 3:
        voxel_xyz_b = voxel_xyz
        assert voxel_xyz_b.shape[0] == B, (
            f"voxel_xyz batch dim {voxel_xyz_b.shape[0]} != intrinsics batch {B}"
        )
    else:
        raise ValueError(f"voxel_xyz must be (q_len, 3) or (B, q_len, 3), got {voxel_xyz.shape}")
    q_len = voxel_xyz_b.shape[1]

    # Homogenize: (B, q_len, 4)
    ones = torch.ones(B, q_len, 1, device=device, dtype=extrinsics.dtype)
    pts_homo = torch.cat([voxel_xyz_b.to(extrinsics.dtype), ones], dim=-1)  # (B, q_len, 4)
    # We want world->cam transform on each batch's voxel set: (B, 1, 4, q_len)
    pts_t = pts_homo.transpose(1, 2).unsqueeze(1)  # (B, 1, 4, q_len)
    # Broadcast over N_cam: (B, N_cam, 4, 4) @ (B, 1, 4, q_len) → (B, N_cam, 4, q_len)
    pts_cam = torch.matmul(extrinsics, pts_t)
    # Apply intrinsics: (B, N_cam, 3, 3) @ (3, q_len) — extract first 3 rows of pts_cam
    pts_pix = torch.matmul(intrinsics, pts_cam[..., :3, :])  # (B, N_cam, 3, q_len)
    z = pts_pix[..., 2, :]  # (B, N_cam, q_len)
    valid_z = z > 1e-3  # in front of camera
    uv = pts_pix[..., :2, :] / z.unsqueeze(-2).clamp_min(1e-6)  # (B, N_cam, 2, q_len)

    # Sample each camera independently (varying Hf/Wf per cam is allowed).
    sampled_per_cam: list[torch.Tensor] = []
    valid_per_cam: list[torch.Tensor] = []
    for c in range(N_cam):
        feat_c = image_features[c]  # (B, C, Hf, Wf)
        Hf, Wf = feat_c.shape[-2:]
        uv_c = uv[:, c]  # (B, 2, q_len)
        u_norm = uv_c[:, 0, :] / max(Wf - 1, 1) * 2 - 1
        v_norm = uv_c[:, 1, :] / max(Hf - 1, 1) * 2 - 1
        # bounds check
        inside = (u_norm >= -1) & (u_norm <= 1) & (v_norm >= -1) & (v_norm <= 1)
        valid_c = valid_z[:, c] & inside  # (B, q_len)
        grid = torch.stack([u_norm, v_norm], dim=-1).view(B, q_len, 1, 2).to(dtype_x)
        # F.grid_sample: input (B, C, Hf, Wf), grid (B, q_len, 1, 2) -> (B, C, q_len, 1)
        s = F.grid_sample(
            feat_c, grid, mode="bilinear", align_corners=False, padding_mode="zeros"
        ).squeeze(-1)  # (B, C, q_len)
        sampled_per_cam.append(s.transpose(1, 2))  # (B, q_len, C)
        valid_per_cam.append(valid_c)

    sampled = torch.stack(sampled_per_cam, dim=2)  # (B, q_len, N_cam, C)
    valid = torch.stack(valid_per_cam, dim=2)      # (B, q_len, N_cam)
    return sampled, valid


def _make_pool_conv(
    channels: int,
    in_res: tuple[int, int, int],
    target_tokens: int,
) -> tuple[nn.Module, tuple[int, int, int]]:
    """Make a 3D conv that pools (Hz, Hy, Hx) → some (Hz', Hy', Hx') with
    product close to target_tokens. Picks integer strides per axis."""
    Hz, Hy, Hx = in_res
    # Find strides (sz, sy, sx) so that ceil(Hz/sz)*... ≈ target_tokens.
    # Greedy: start with all strides=1, increase the largest axis until we hit budget.
    sz = sy = sx = 1
    def out_size(h, s): return (h + s - 1) // s
    def total_tokens(): return out_size(Hz, sz) * out_size(Hy, sy) * out_size(Hx, sx)

    while total_tokens() > target_tokens:
        # increase stride on the dim whose CURRENT output size is largest
        candidates = [
            (out_size(Hz, sz), 'z'),
            (out_size(Hy, sy), 'y'),
            (out_size(Hx, sx), 'x'),
        ]
        candidates.sort(reverse=True)
        ax = candidates[0][1]
        if ax == 'z': sz += 1
        elif ax == 'y': sy += 1
        else: sx += 1
        if sz > Hz and sy > Hy and sx > Hx:
            break

    out_z = out_size(Hz, sz)
    out_y = out_size(Hy, sy)
    out_x = out_size(Hx, sx)
    conv = nn.Sequential(
        nn.Conv3d(channels, channels, kernel_size=(min(sz+1,Hz), min(sy+1,Hy), min(sx+1,Hx)),
                  stride=(sz, sy, sx), padding=0, bias=True),
        nn.GELU(),
    )
    return conv, (out_z, out_y, out_x)
