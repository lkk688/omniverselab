"""LiftSplat-style BEV encoder for calibrated multi-view RGB.

Given N RGB images + N (K, world->cam) calibrations and a fixed world-frame
voxel grid, this module:
  1. Encodes each image with a shared CNN (ResNet18 layers 1-3, ImageNet-pretrained).
  2. Projects every voxel center into every camera and samples its feature
     via bilinear `grid_sample`. Voxels behind a camera or outside its FOV
     receive zero features and a 0 visibility weight.
  3. Aggregates features across cameras (visibility-weighted mean).
  4. Applies a 1x1 conv + (optional) 3D conv refinement.
  5. Reduces the Z axis to a 2D BEV feature map (z-mean).

Returns the (B, C, X, Y) BEV map plus the (B, C, X, Y, Z) voxel feature
volume (for the auxiliary occupancy/heatmap head).

Conventions
-----------
* World axes: x forward (toward table edge), y left, z up. Workspace at
  roughly x=0.5, y=0, z=0..0.1.
* Camera frame: OpenCV (+x right, +y down, +z forward).
* `K` is (3,3) pixel intrinsics matching the *input image resolution*.
* `extrinsics_w2c` is (4,4) world->cam (rotation top-left, translation top-right).
"""

from __future__ import annotations

from typing import Tuple

import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision.models import resnet18, ResNet18_Weights


class ImgBackbone(nn.Module):
    """ResNet18 trunk up through layer3 → (B, 256, H/16, W/16). ImageNet init."""

    def __init__(self, out_channels: int = 256):
        super().__init__()
        net = resnet18(weights=ResNet18_Weights.IMAGENET1K_V1)
        # conv1..layer3 give 256 channels at stride 16.
        self.stem = nn.Sequential(
            net.conv1, net.bn1, net.relu, net.maxpool,
            net.layer1, net.layer2, net.layer3,
        )
        self._out_channels = out_channels

    @property
    def out_channels(self) -> int:
        return self._out_channels

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.stem(x)


class LiftSplatBEV(nn.Module):
    """Project per-camera image features into a world-frame voxel grid, then
    collapse to BEV.

    Args:
        n_cam: number of input cameras (must match call-site).
        voxel_x_range, voxel_y_range, voxel_z_range: workspace bounds (metres).
        voxel_size: edge length of one voxel cube (metres).
        feature_dim: output channels for the BEV (and voxel) feature.
        backbone_out_channels: channels emitted by `ImgBackbone`.
        n_3d_refine_layers: how many 3D conv blocks to stack on the voxel grid.
        pool_mode: "mean" | "max" across cameras.
        image_hw: (H, W) at which the images come in. Used to set the feature-
            map sample grid and to early-reject out-of-frame voxel projections.
    """

    def __init__(
        self,
        n_cam: int,
        voxel_x_range: Tuple[float, float],
        voxel_y_range: Tuple[float, float],
        voxel_z_range: Tuple[float, float],
        voxel_size: float,
        feature_dim: int = 128,
        backbone_out_channels: int = 256,
        n_3d_refine_layers: int = 1,
        pool_mode: str = "mean",
        image_hw: Tuple[int, int] = (240, 320),
    ):
        super().__init__()
        self.n_cam = n_cam
        self.feature_dim = feature_dim
        self.pool_mode = pool_mode
        self.image_hw = image_hw

        # Backbone (shared across cameras) and channel-reducing 1x1.
        self.backbone = ImgBackbone(out_channels=backbone_out_channels)
        self.feat_proj = nn.Conv2d(backbone_out_channels, feature_dim, kernel_size=1)

        # Voxel grid (world-frame, fixed; registered as buffer so it follows .to()).
        x_lo, x_hi = voxel_x_range
        y_lo, y_hi = voxel_y_range
        z_lo, z_hi = voxel_z_range
        nx = max(1, int(round((x_hi - x_lo) / voxel_size)))
        ny = max(1, int(round((y_hi - y_lo) / voxel_size)))
        nz = max(1, int(round((z_hi - z_lo) / voxel_size)))
        xs = torch.linspace(x_lo + voxel_size / 2, x_hi - voxel_size / 2, nx)
        ys = torch.linspace(y_lo + voxel_size / 2, y_hi - voxel_size / 2, ny)
        zs = torch.linspace(z_lo + voxel_size / 2, z_hi - voxel_size / 2, nz)
        # (X, Y, Z, 3) of voxel-centre world coords.
        gx, gy, gz = torch.meshgrid(xs, ys, zs, indexing="ij")
        voxel_grid = torch.stack([gx, gy, gz], dim=-1)
        self.register_buffer("voxel_grid", voxel_grid, persistent=False)
        self.nx, self.ny, self.nz = nx, ny, nz
        self.voxel_x_range = voxel_x_range
        self.voxel_y_range = voxel_y_range
        self.voxel_z_range = voxel_z_range
        self.voxel_size = voxel_size

        # 3D refinement (small — kept cheap).
        refine_layers = []
        for _ in range(n_3d_refine_layers):
            refine_layers += [
                nn.Conv3d(feature_dim, feature_dim, kernel_size=3, padding=1),
                nn.GroupNorm(num_groups=8, num_channels=feature_dim),
                nn.GELU(),
            ]
        self.refine_3d = nn.Sequential(*refine_layers) if refine_layers else nn.Identity()

    @property
    def bev_hw(self) -> Tuple[int, int]:
        return (self.nx, self.ny)

    def _project_voxels(
        self, K: torch.Tensor, extrinsics_w2c: torch.Tensor
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        """Project every voxel into every camera.

        Args:
            K: (B, N, 3, 3) intrinsics.
            extrinsics_w2c: (B, N, 4, 4) world->cam.
        Returns:
            uv_norm: (B, N, X, Y, Z, 2) — normalized pixel coords in [-1,1]
                suitable for grid_sample. NaN-filled where invalid.
            visibility: (B, N, X, Y, Z) — 1 where projection is inside the
                image and in front of camera, else 0.
        """
        B, N = K.shape[:2]
        X, Y, Z, _ = self.voxel_grid.shape
        H, W = self.image_hw
        device = K.device

        # (B, N, X*Y*Z, 4)
        pts = self.voxel_grid.reshape(-1, 3)
        ones = torch.ones((pts.shape[0], 1), device=device, dtype=pts.dtype)
        pts_h = torch.cat([pts, ones], dim=-1)        # (V, 4)
        pts_h = pts_h.unsqueeze(0).unsqueeze(0)        # (1, 1, V, 4)
        pts_h = pts_h.expand(B, N, -1, -1)

        # Transform to camera frame: (B, N, V, 4)
        # extrinsics_w2c: (B, N, 4, 4); apply to (B, N, V, 4)
        cam = torch.einsum("bnij,bnvj->bnvi", extrinsics_w2c.to(pts_h.dtype), pts_h)
        x_c, y_c, z_c = cam[..., 0], cam[..., 1], cam[..., 2]

        valid_z = z_c > 1e-3
        z_safe = torch.where(valid_z, z_c, torch.ones_like(z_c))

        # Pixel coords: (u, v) = K @ (x/z, y/z, 1)
        # Using (B,N,3,3) @ (B,N,3,V) → (B,N,3,V). Build the homogeneous (x/z, y/z, 1).
        norm_cam = torch.stack([x_c / z_safe, y_c / z_safe, torch.ones_like(z_safe)], dim=-1)
        # (B,N,V,3) → (B,N,3,V) for matmul, then back.
        uvw = torch.einsum("bnij,bnvj->bnvi", K.to(pts_h.dtype), norm_cam)
        u = uvw[..., 0]
        v = uvw[..., 1]

        # grid_sample expects coords in [-1, 1] with x=-1 left, y=-1 top.
        u_norm = (u / (W - 1)) * 2.0 - 1.0
        v_norm = (v / (H - 1)) * 2.0 - 1.0
        in_image = (u_norm.abs() <= 1.0) & (v_norm.abs() <= 1.0)
        visibility = (valid_z & in_image).to(K.dtype)

        # Replace invalid coords with a value that grid_sample will pad to zero.
        uv = torch.stack([u_norm, v_norm], dim=-1)  # (B, N, V, 2)
        bad = ~(valid_z & in_image)
        uv = torch.where(bad.unsqueeze(-1), torch.full_like(uv, 2.0), uv)

        uv = uv.reshape(B, N, X, Y, Z, 2)
        visibility = visibility.reshape(B, N, X, Y, Z)
        return uv, visibility

    def forward(
        self,
        images: torch.Tensor,
        intrinsics: torch.Tensor,
        extrinsics_w2c: torch.Tensor,
    ) -> dict[str, torch.Tensor]:
        """
        Args:
            images: (B, N, 3, H, W), float in [0,1] after the upstream normalizer.
            intrinsics: (B, N, 3, 3)
            extrinsics_w2c: (B, N, 4, 4)
        Returns dict with:
            - bev: (B, C, X, Y)
            - voxel: (B, C, X, Y, Z) — for aux/occupancy heads
            - visibility: (B, X, Y, Z) — total visibility across cameras
        """
        B, N, C, H, W = images.shape
        assert N == self.n_cam, f"n_cam={self.n_cam} != images.shape[1]={N}"
        assert (H, W) == self.image_hw, f"image_hw={self.image_hw} != {(H, W)}"

        # 1) encode each image
        feat = self.backbone(images.reshape(B * N, C, H, W))           # (B*N, Cb, hf, wf)
        feat = self.feat_proj(feat)                                    # (B*N, Cv, hf, wf)
        _, Cv, hf, wf = feat.shape

        # 2) project voxels to pixel grids in normalized space (grid_sample expects)
        uv, vis = self._project_voxels(intrinsics, extrinsics_w2c)     # (B,N,X,Y,Z,2), (B,N,X,Y,Z)

        # 3) sample features per cam at every voxel
        # grid_sample input  : (B*N, Cv, hf, wf)
        # grid_sample sample : (B*N, X, Y*Z, 2) — squeeze (X,Y,Z) into a 2D grid
        # We flatten (Y,Z) into a "width" axis for the 2D grid_sample call.
        uv_flat = uv.reshape(B * N, self.nx, self.ny * self.nz, 2)
        sampled = F.grid_sample(
            feat, uv_flat, mode="bilinear", padding_mode="zeros", align_corners=True
        )  # (B*N, Cv, X, Y*Z)
        sampled = sampled.reshape(B, N, Cv, self.nx, self.ny, self.nz)

        # 4) aggregate across cameras (visibility-weighted)
        vis_b = vis.unsqueeze(2)                                       # (B, N, 1, X, Y, Z)
        if self.pool_mode == "max":
            sampled = sampled.masked_fill(vis_b == 0, float("-inf"))
            voxel = sampled.max(dim=1).values                          # (B, Cv, X, Y, Z)
            voxel = torch.where(torch.isinf(voxel), torch.zeros_like(voxel), voxel)
        else:
            # weighted mean across cameras; denom with eps
            sampled_w = sampled * vis_b
            denom = vis_b.sum(dim=1).clamp_min(1e-6)                   # (B, 1, X, Y, Z)
            voxel = sampled_w.sum(dim=1) / denom                       # (B, Cv, X, Y, Z)

        # 5) 3D refinement
        voxel = self.refine_3d(voxel)

        # 6) BEV slice — mean along Z axis (last dim, after channel)
        bev = voxel.mean(dim=-1)                                       # (B, Cv, X, Y)

        return {
            "bev": bev,
            "voxel": voxel,
            "visibility": vis.sum(dim=1),                              # (B, X, Y, Z)
        }
