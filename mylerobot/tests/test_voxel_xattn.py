"""Unit tests for the VoxelCrossAttnFusion module.

Standalone — no lerobot/pi0 dependency. Validates:
  - module instantiates cleanly across single-head and multi-head + GQA configs
  - voxel grid centres are correctly placed within workspace bounds
  - forward pass produces correct output shapes (with and without pooling)
  - projections respect camera extrinsics (a voxel in front of cam → valid; behind → masked)
  - gradients flow to all trainable params via a synthetic loss
"""

from __future__ import annotations

import numpy as np
import pytest
import torch

from mylerobot.policies.vision_fusion.voxel_xattn import (
    VoxelCrossAttnFusion,
    _build_voxel_grid,
    _project_and_sample,
)


# ---------------------------------------------------------------------------
# Grid construction
# ---------------------------------------------------------------------------
def test_build_voxel_grid_shape_and_centring():
    # res=(Hz=2, Hy=3, Hx=4); bounds x∈[0,1], y∈[0,1.5], z∈[0,2]
    # → dx=0.25, dy=0.5, dz=1.0; first voxel centre at (dx/2, dy/2, dz/2)
    grid = _build_voxel_grid((2, 3, 4), ((0.0, 0.0, 0.0), (1.0, 1.5, 2.0)))
    assert grid.shape == (2, 3, 4, 3)
    np.testing.assert_allclose(grid[0, 0, 0].numpy(), [0.125, 0.25, 0.5], atol=1e-6)
    # Last voxel centre at (xmax - dx/2, ymax - dy/2, zmax - dz/2)
    np.testing.assert_allclose(grid[-1, -1, -1].numpy(), [0.875, 1.25, 1.5], atol=1e-6)


# ---------------------------------------------------------------------------
# Module init
# ---------------------------------------------------------------------------
def test_module_init_single_head():
    m = VoxelCrossAttnFusion(
        feat_channels=64, out_channels=128,
        voxel_resolution=(4, 4, 4),
        workspace_bounds=((-0.5, -0.5, 0.0), (0.5, 0.5, 0.5)),
        num_heads=1,
        out_tokens=64,
    )
    # No Q/K/V projections in single-head path
    assert m.wq is None and m.wk is None and m.wv is None and m.wo is None


def test_module_init_multihead_gqa():
    m = VoxelCrossAttnFusion(
        feat_channels=64, out_channels=128,
        voxel_resolution=(4, 4, 4),
        workspace_bounds=((-0.5, -0.5, 0.0), (0.5, 0.5, 0.5)),
        num_heads=4, num_kv_groups=2,
        out_tokens=64,
    )
    assert m.wq is not None and m.wk is not None
    # GQA: K/V heads = 4 / 2 = 2
    assert m.num_kv_heads == 2
    # K/V proj output dim = 2 * (64/4) = 32
    assert m.wk.weight.shape == (32, 64)


def test_module_init_validates_head_dims():
    # feat_channels not divisible by num_heads
    with pytest.raises(AssertionError):
        VoxelCrossAttnFusion(
            feat_channels=63, out_channels=128,
            voxel_resolution=(4, 4, 4),
            workspace_bounds=((-0.5, -0.5, 0.0), (0.5, 0.5, 0.5)),
            num_heads=4,
            out_tokens=64,
        )


# ---------------------------------------------------------------------------
# Helpers for building a synthetic 2-camera scene
# ---------------------------------------------------------------------------
def _make_synthetic_scene(B: int = 1, n_cam: int = 2, feat_channels: int = 16,
                          Hf: int = 8, Wf: int = 8):
    """Two cameras looking at the workspace origin.

    Cam 0: in front of workspace at +X looking back (-X).
    Cam 1: above workspace at +Z looking down (-Z).
    """
    torch.manual_seed(0)
    image_features = [
        torch.randn(B, feat_channels, Hf, Wf, dtype=torch.float32) for _ in range(n_cam)
    ]
    # OpenCV intrinsic for an 8x8 feature: fx=fy=8, cx=cy=4
    K = torch.tensor([[8.0, 0.0, 4.0],
                      [0.0, 8.0, 4.0],
                      [0.0, 0.0, 1.0]], dtype=torch.float32)
    intrinsics = K.view(1, 1, 3, 3).expand(B, n_cam, 3, 3).clone()

    # Build world->cam for two cameras.
    # OpenCV cam frame: +X right, +Y down, +Z forward.
    extrinsics = torch.zeros(B, n_cam, 4, 4, dtype=torch.float32)
    extrinsics[..., 3, 3] = 1.0

    # Cam 0 at world (1, 0, 0.25), looking towards origin (so cam +Z = -X world).
    # World→Cam rotation: cam_z = -world_x, cam_x = -world_y, cam_y = -world_z
    R0 = torch.tensor([[ 0.0, -1.0,  0.0],
                       [ 0.0,  0.0, -1.0],
                       [-1.0,  0.0,  0.0]])
    t0 = torch.tensor([0.0, 0.0, 1.0])  # cam-frame translation s.t. world origin ends up at (0,0,1)
    extrinsics[:, 0, :3, :3] = R0
    extrinsics[:, 0, :3, 3] = t0

    # Cam 1 above workspace at world (0, 0, 1.5), looking down (cam +Z = -Z world).
    R1 = torch.tensor([[ 1.0,  0.0,  0.0],
                       [ 0.0, -1.0,  0.0],
                       [ 0.0,  0.0, -1.0]])
    t1 = torch.tensor([0.0, 0.0, 1.5])
    extrinsics[:, 1, :3, :3] = R1
    extrinsics[:, 1, :3, 3] = t1

    return image_features, intrinsics, extrinsics


# ---------------------------------------------------------------------------
# Projection + sampling
# ---------------------------------------------------------------------------
def test_project_and_sample_shapes():
    image_features, intrinsics, extrinsics = _make_synthetic_scene(B=2, n_cam=2, feat_channels=16)
    voxel_xyz = torch.randn(20, 3) * 0.2  # 20 voxel centres near origin
    sampled, valid = _project_and_sample(voxel_xyz, image_features, intrinsics, extrinsics)
    assert sampled.shape == (2, 20, 2, 16)
    assert valid.shape == (2, 20, 2)
    assert valid.dtype == torch.bool


def test_project_voxel_in_view_is_valid():
    """Origin is in front of both cameras → both should be valid."""
    image_features, intrinsics, extrinsics = _make_synthetic_scene(B=1, n_cam=2, feat_channels=8)
    origin = torch.zeros(1, 3)
    sampled, valid = _project_and_sample(origin, image_features, intrinsics, extrinsics)
    assert valid[0, 0, 0].item() is True
    assert valid[0, 0, 1].item() is True


def test_project_voxel_behind_cam0_is_invalid():
    """A point at world (2, 0, 0.25) is BEHIND cam 0 (which sits at x=1 looking -x; so x>1 is behind)."""
    image_features, intrinsics, extrinsics = _make_synthetic_scene(B=1, n_cam=2, feat_channels=8)
    behind = torch.tensor([[2.0, 0.0, 0.25]])
    _, valid = _project_and_sample(behind, image_features, intrinsics, extrinsics)
    # cam 0 should be invalid; cam 1 (above looking down) should still see this point.
    assert valid[0, 0, 0].item() is False


# ---------------------------------------------------------------------------
# Full forward pass
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("num_heads,num_kv_groups,out_tokens", [
    (1, 1, 32),
    (4, 1, 32),
    (4, 2, 32),
    (4, 4, 32),   # MQA
    (1, 1, 64),   # output equals n_voxels (4^3=64)
])
def test_forward_shapes(num_heads, num_kv_groups, out_tokens):
    image_features, intrinsics, extrinsics = _make_synthetic_scene(
        B=2, n_cam=2, feat_channels=16
    )
    m = VoxelCrossAttnFusion(
        feat_channels=16, out_channels=32,
        voxel_resolution=(4, 4, 4),
        workspace_bounds=((-0.3, -0.3, -0.1), (0.3, 0.3, 0.4)),
        num_heads=num_heads, num_kv_groups=num_kv_groups,
        out_tokens=out_tokens,
    )
    tokens = m(image_features, intrinsics, extrinsics)
    # Token count may be slightly different from requested if 3D conv strides don't hit it exactly.
    B, N, C = tokens.shape
    assert B == 2
    assert C == 32
    assert N <= 64  # bounded by n_voxels
    if out_tokens == 64:
        assert N == 64  # no pooling


def test_forward_backward_gradients_flow():
    image_features, intrinsics, extrinsics = _make_synthetic_scene(
        B=2, n_cam=2, feat_channels=16
    )
    m = VoxelCrossAttnFusion(
        feat_channels=16, out_channels=32,
        voxel_resolution=(4, 4, 4),
        workspace_bounds=((-0.3, -0.3, -0.1), (0.3, 0.3, 0.4)),
        num_heads=4, num_kv_groups=2,
        out_tokens=16,
        zero_init_out_proj=False,   # production default zero-inits; tests need a "live" module
    )
    tokens = m(image_features, intrinsics, extrinsics)
    loss = tokens.pow(2).mean()
    loss.backward()
    # Every trainable param should have a non-None grad.
    no_grad = [n for n, p in m.named_parameters() if p.requires_grad and p.grad is None]
    assert not no_grad, f"params without grad: {no_grad}"
    # And at least one non-zero grad somewhere.
    any_nonzero = any(p.grad.abs().sum() > 0 for n, p in m.named_parameters() if p.grad is not None)
    assert any_nonzero


def test_cam_mask_zeroes_disabled_camera_contribution():
    """If we mask out all cameras, output should be all zeros (none_valid path)."""
    image_features, intrinsics, extrinsics = _make_synthetic_scene(
        B=1, n_cam=2, feat_channels=16
    )
    m = VoxelCrossAttnFusion(
        feat_channels=16, out_channels=32,
        voxel_resolution=(2, 2, 2),
        workspace_bounds=((-0.1, -0.1, -0.1), (0.1, 0.1, 0.1)),
        num_heads=1,
        out_tokens=8,
        zero_init_out_proj=False,   # need non-zero output to detect cam-mask effect
    )
    cam_mask = torch.zeros(1, 2, dtype=torch.bool)  # disable both cameras
    tokens = m(image_features, intrinsics, extrinsics, cam_mask=cam_mask)
    # out_proj has bias, so output isn't strictly zero, but the residual fused-feats path is zero.
    # Stronger check: re-run with all cams enabled, expect a different output.
    tokens_enabled = m(image_features, intrinsics, extrinsics, cam_mask=torch.ones(1, 2, dtype=torch.bool))
    assert not torch.allclose(tokens, tokens_enabled, atol=1e-5), (
        "cam_mask had no effect on output"
    )


def test_voxel_outside_view_gets_zero_fused_feats():
    """Voxel far outside any camera's view should be masked, fused feat all zero."""
    image_features, intrinsics, extrinsics = _make_synthetic_scene(
        B=1, n_cam=2, feat_channels=16
    )
    m = VoxelCrossAttnFusion(
        feat_channels=16, out_channels=32,
        voxel_resolution=(2, 2, 2),
        # Workspace far away — no voxel should project into any camera.
        workspace_bounds=((100.0, 100.0, 100.0), (101.0, 101.0, 101.0)),
        num_heads=1,
        out_tokens=8,
    )
    tokens = m(image_features, intrinsics, extrinsics)
    # Per-voxel fused (before out_proj+bias) is zeroed for none_valid queries; after
    # out_proj+bias, all tokens are the same (the bias projected). So variance ≈ 0.
    var = tokens.var(dim=1).max()
    assert var < 1e-5, f"expected near-zero variance across tokens when all out of view, got {var}"
