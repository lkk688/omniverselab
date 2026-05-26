"""Auxiliary heads on top of the BEV feature map.

Phase 1: cube-pose heatmap.
  - The encoder produces a (B, C, X, Y) BEV feature.
  - A small 2D conv predicts a 1-channel logit map.
  - The supervision target is a Gaussian centred at the projected cube XY,
    rendered on the same (X, Y) grid as the BEV. Loss is per-pixel BCE
    (focal-style weighting optional).
  - This forces the BEV encoder to *attend to the target object*, which is the
    aux gradient signal that breaks the proprio shortcut.

Phase 2 (TODO): per-voxel occupancy with Isaac-derived ground truth.
"""

from __future__ import annotations

import torch
import torch.nn as nn
import torch.nn.functional as F


class CubeHeatmapHead(nn.Module):
    """Predict a cube-presence heatmap on the BEV grid."""

    def __init__(self, in_channels: int, hidden: int = 64):
        super().__init__()
        self.head = nn.Sequential(
            nn.Conv2d(in_channels, hidden, kernel_size=3, padding=1),
            nn.GroupNorm(8, hidden),
            nn.GELU(),
            nn.Conv2d(hidden, 1, kernel_size=1),
        )

    def forward(self, bev_feat: torch.Tensor) -> torch.Tensor:
        return self.head(bev_feat).squeeze(1)  # (B, X, Y) logits


def build_target_heatmap(
    cube_pose_w: torch.Tensor,
    voxel_x_range: tuple[float, float],
    voxel_y_range: tuple[float, float],
    voxel_size: float,
    sigma_m: float,
) -> torch.Tensor:
    """Render a 2D Gaussian heatmap centred at the cube XY.

    Args:
        cube_pose_w: (B, 3) world-frame cube position. Only x,y are used.
        voxel_x_range, voxel_y_range: workspace bounds matching the BEV grid.
        voxel_size: voxel edge length (metres).
        sigma_m: Gaussian standard deviation in metres on the BEV plane.

    Returns:
        (B, X, Y) target heatmap in [0, 1], with 1.0 at the cube centre.
    """
    device = cube_pose_w.device
    dtype = cube_pose_w.dtype

    x_lo, x_hi = voxel_x_range
    y_lo, y_hi = voxel_y_range
    nx = max(1, int(round((x_hi - x_lo) / voxel_size)))
    ny = max(1, int(round((y_hi - y_lo) / voxel_size)))

    xs = torch.linspace(x_lo + voxel_size / 2, x_hi - voxel_size / 2, nx, device=device, dtype=dtype)
    ys = torch.linspace(y_lo + voxel_size / 2, y_hi - voxel_size / 2, ny, device=device, dtype=dtype)
    gx, gy = torch.meshgrid(xs, ys, indexing="ij")            # (X, Y)

    cube_x = cube_pose_w[:, 0].view(-1, 1, 1)
    cube_y = cube_pose_w[:, 1].view(-1, 1, 1)
    sq = (gx.unsqueeze(0) - cube_x) ** 2 + (gy.unsqueeze(0) - cube_y) ** 2
    return torch.exp(-0.5 * sq / (sigma_m ** 2))


def cube_heatmap_loss(
    pred_logits: torch.Tensor,
    target: torch.Tensor,
    visibility_xy: torch.Tensor | None = None,
) -> torch.Tensor:
    """BCE-with-logits between predicted heatmap and a soft Gaussian target.

    Args:
        pred_logits: (B, X, Y)
        target:      (B, X, Y)
        visibility_xy: optional (B, X, Y) ≥0 mask; pixels with 0 visibility
            (no camera saw that voxel column) are excluded from the loss.

    Returns scalar loss.
    """
    loss_map = F.binary_cross_entropy_with_logits(pred_logits, target, reduction="none")
    if visibility_xy is not None:
        m = (visibility_xy > 0).to(loss_map.dtype)
        denom = m.sum().clamp_min(1.0)
        return (loss_map * m).sum() / denom
    return loss_map.mean()
