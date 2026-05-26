"""Unit tests for PI0VoxelConfig + extrinsics_libero (no pi0/PaliGemma needed).

Heavy integration tests (loading actual pi0 weights, running embed_prefix
end-to-end) live separately and require a GPU — kept out of the default
pytest run.
"""

from __future__ import annotations

import numpy as np
import pytest
import torch

from mylerobot.policies.pi0_voxel.configuration_pi0_voxel import PI0VoxelConfig
from mylerobot.policies.pi0_voxel.extrinsics_libero import (
    AGENTVIEW_K_256,
    WRIST_K_256,
    agentview_extrinsic_world2cam_cv,
    axis_angle_to_rotmat_np,
    cam_world_pose_to_world2cam_cv_np,
    derive_libero_extrinsics_batch,
    intrinsic_from_fovy,
    quat_wxyz_to_rotmat_np,
    wrist_extrinsic_from_state,
)


# ---------------------------------------------------------------------------
# Geometry primitives
# ---------------------------------------------------------------------------
def test_axis_angle_zero_is_identity():
    R = axis_angle_to_rotmat_np(np.zeros(3))
    np.testing.assert_allclose(R, np.eye(3), atol=1e-12)


def test_axis_angle_pi_around_x():
    # 180° around X-axis: y→-y, z→-z
    R = axis_angle_to_rotmat_np(np.array([np.pi, 0, 0]))
    expected = np.diag([1.0, -1.0, -1.0])
    np.testing.assert_allclose(R, expected, atol=1e-10)


def test_axis_angle_90_around_z():
    R = axis_angle_to_rotmat_np(np.array([0, 0, np.pi / 2]))
    expected = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=np.float64)
    np.testing.assert_allclose(R, expected, atol=1e-10)


def test_quat_wxyz_identity():
    R = quat_wxyz_to_rotmat_np(np.array([1, 0, 0, 0]))
    np.testing.assert_allclose(R, np.eye(3), atol=1e-12)


def test_quat_wxyz_90_around_y():
    R = quat_wxyz_to_rotmat_np(np.array([np.sqrt(0.5), 0, np.sqrt(0.5), 0]))
    expected = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], dtype=np.float64)
    np.testing.assert_allclose(R, expected, atol=1e-10)


def test_world2cam_cv_inverse_roundtrip():
    """Projecting the camera's own world position via world2cam must give the origin."""
    pos = np.array([1.5, -0.3, 0.6])
    R_cam2world = axis_angle_to_rotmat_np(np.array([0, 0.5, 0]))  # arbitrary
    T = cam_world_pose_to_world2cam_cv_np(pos, R_cam2world)
    pos_h = np.array([pos[0], pos[1], pos[2], 1.0])
    proj = T @ pos_h
    np.testing.assert_allclose(proj[:3], [0, 0, 0], atol=1e-9)


# ---------------------------------------------------------------------------
# Per-camera helpers
# ---------------------------------------------------------------------------
def test_agentview_extrinsic_orthonormal():
    T = agentview_extrinsic_world2cam_cv()
    R = T[:3, :3]
    np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-7)
    assert np.linalg.det(R) == pytest.approx(1.0, abs=1e-6)


def test_wrist_extrinsic_shape_and_orthonormal():
    state = np.array([-0.05, 0.007, 0.68, np.pi, 0.002, -0.09, 0.039, -0.039])
    T = wrist_extrinsic_from_state(state)
    assert T.shape == (4, 4)
    R = T[:3, :3]
    np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-7)
    assert np.linalg.det(R) == pytest.approx(1.0, abs=1e-6)


def test_wrist_extrinsic_validates_state_shape():
    with pytest.raises(ValueError):
        wrist_extrinsic_from_state(np.zeros(7))


def test_intrinsic_known_values():
    K = intrinsic_from_fovy(90.0, 100, 100)
    assert K[0, 0] == pytest.approx(50.0)
    assert K[0, 2] == pytest.approx(50.0)
    assert K[1, 2] == pytest.approx(50.0)


def test_default_intrinsics_size():
    assert AGENTVIEW_K_256.shape == (3, 3)
    assert WRIST_K_256.shape == (3, 3)
    # Wrist has wider FOV → smaller focal length
    assert WRIST_K_256[0, 0] < AGENTVIEW_K_256[0, 0]


# ---------------------------------------------------------------------------
# Vectorised derivation
# ---------------------------------------------------------------------------
def test_derive_libero_extrinsics_batch_shapes():
    B = 4
    states = torch.zeros(B, 8)
    states[:, 3] = np.pi  # gripper-down orientation
    extr = derive_libero_extrinsics_batch(states)
    assert set(extr.keys()) == {"agentview", "wrist"}
    assert extr["agentview"].shape == (B, 4, 4)
    assert extr["wrist"].shape == (B, 4, 4)


def test_derive_libero_extrinsics_batch_agentview_constant():
    """Agentview extrinsic should be the same for every batch element."""
    B = 3
    rng = np.random.default_rng(0)
    states = torch.from_numpy(rng.normal(size=(B, 8))).float()
    extr = derive_libero_extrinsics_batch(states)
    for b in range(1, B):
        torch.testing.assert_close(extr["agentview"][0], extr["agentview"][b])


def test_derive_libero_extrinsics_batch_wrist_varies_with_state():
    """Wrist extrinsic must depend on state (different states → different poses)."""
    s1 = torch.tensor([[-0.05, 0.0, 0.68, np.pi, 0, 0, 0.04, -0.04]])
    s2 = torch.tensor([[0.10, 0.0, 0.68, np.pi, 0, 0, 0.04, -0.04]])
    e1 = derive_libero_extrinsics_batch(s1)
    e2 = derive_libero_extrinsics_batch(s2)
    # agentview should be identical
    torch.testing.assert_close(e1["agentview"], e2["agentview"])
    # wrist should differ
    assert not torch.allclose(e1["wrist"], e2["wrist"], atol=1e-3)


# ---------------------------------------------------------------------------
# Config validation
# ---------------------------------------------------------------------------
def test_config_defaults_construct():
    cfg = PI0VoxelConfig()
    assert cfg.voxel_arch == "cross_attn"
    assert cfg.voxel_resolution == (8, 16, 16)
    assert cfg.voxel_out_tokens == 64
    assert cfg.extrinsics_optional is True


def test_config_rejects_bad_arch():
    with pytest.raises(ValueError):
        PI0VoxelConfig(voxel_arch="bogus")


def test_config_rejects_bad_workspace_bounds():
    with pytest.raises(ValueError):
        PI0VoxelConfig(workspace_bounds=((0, 0, 0), (0, 1, 1)))  # x equal


def test_config_rejects_occupancy_aux_without_unet():
    with pytest.raises(ValueError):
        PI0VoxelConfig(voxel_arch="cross_attn", use_occupancy_aux=True)


def test_config_accepts_unet_with_occupancy():
    cfg = PI0VoxelConfig(voxel_arch="unet", use_occupancy_aux=True)
    assert cfg.use_occupancy_aux is True


def test_config_register_subclass():
    """PI0VoxelConfig should be registered with type='pi0_voxel'."""
    from lerobot.configs import PreTrainedConfig
    # subclass registry lookup
    found = PreTrainedConfig.get_choice_class("pi0_voxel")
    assert found is PI0VoxelConfig


# ---------------------------------------------------------------------------
# Lazy-factory wiring (verifies the subclass machinery without loading weights)
# ---------------------------------------------------------------------------
def test_pi0voxel_pytorch_factory_builds_class():
    """The factory in modeling.py should build a class that's a true PI0Pytorch
    subclass and exposes our voxel_fusion module attribute on instances."""
    from mylerobot.policies.pi0_voxel.modeling_pi0_voxel import _make_pi0_voxel_pytorch_class
    from lerobot.policies.pi0.modeling_pi0 import PI0Pytorch

    Klass = _make_pi0_voxel_pytorch_class()
    assert issubclass(Klass, PI0Pytorch)
    # __init__ should accept (config, rtc_processor=None)
    import inspect
    sig = inspect.signature(Klass.__init__)
    params = list(sig.parameters)
    assert "config" in params
    assert "rtc_processor" in params
    # The subclass adds an `embed_prefix` override
    assert Klass.embed_prefix is not PI0Pytorch.embed_prefix
    # And a build_voxel_tokens helper
    assert hasattr(Klass, "build_voxel_tokens")


def test_pi0voxel_policy_factory_builds_class():
    from mylerobot.policies.pi0_voxel.modeling_pi0_voxel import _make_pi0_voxel_policy_class
    from lerobot.policies.pi0.modeling_pi0 import PI0Policy

    Klass = _make_pi0_voxel_policy_class()
    assert issubclass(Klass, PI0Policy)
    assert Klass.name == "pi0_voxel"
    assert Klass.config_class is PI0VoxelConfig
    # Override hooks present
    assert Klass.forward is not PI0Policy.forward
    assert Klass.predict_action_chunk is not PI0Policy.predict_action_chunk
    assert hasattr(Klass, "_prepare_voxel_aux")
