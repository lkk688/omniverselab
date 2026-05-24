"""Unit tests for the multi-camera aloha env wrapper."""

import os
os.environ.setdefault("MUJOCO_GL", "egl")

import numpy as np
import pytest

from mylerobot.sim.aloha_multicam import (
    AlohaMultiCamEnv,
    DEFAULT_SCENE_CAMERAS,
    intrinsic_from_fovy,
    mujoco_cam_pose_to_opencv_extrinsic,
)


@pytest.fixture(scope="module")
def env():
    e = AlohaMultiCamEnv(task="insertion", observation_height=120, observation_width=160)
    e.reset(seed=0)
    yield e
    e.close()


def test_camera_names_match_defaults(env):
    assert tuple(env.cameras) == DEFAULT_SCENE_CAMERAS


def test_render_returns_all_cameras(env):
    imgs = env.render_all_cameras()
    assert set(imgs.keys()) == set(DEFAULT_SCENE_CAMERAS)
    for name, img in imgs.items():
        assert img.shape == (120, 160, 3), f"{name}: {img.shape}"
        assert img.dtype == np.uint8
        # Cameras are looking at the table; render should not be all zero.
        assert img.sum() > 0, f"{name} render is all zero — camera blocked?"


def test_observation_dict_shape(env):
    obs, info = env.reset(seed=1)
    assert "pixels" in obs and "agent_pos" in obs
    assert set(obs["pixels"].keys()) == set(DEFAULT_SCENE_CAMERAS)
    assert obs["agent_pos"].shape == (14,)


def test_step_renders_all_cameras(env):
    env.reset(seed=2)
    action = np.zeros(env.action_space.shape, dtype=np.float32)
    obs, reward, term, trunc, info = env.step(action)
    assert set(obs["pixels"].keys()) == set(DEFAULT_SCENE_CAMERAS)
    for name, img in obs["pixels"].items():
        assert img.shape == (120, 160, 3)


def test_calibration_shapes(env):
    env.reset(seed=3)
    calib = env.get_calibration()
    assert set(calib.keys()) == set(DEFAULT_SCENE_CAMERAS)
    for name, c in calib.items():
        assert c["cam_xpos_world"].shape == (3,)
        assert c["cam_xmat_world"].shape == (3, 3)
        assert c["intrinsic_K"].shape == (3, 3)
        assert c["extrinsic_world2cam_cv"].shape == (4, 4)
        # Sanity: rotation block should be orthonormal up to float error.
        R = c["extrinsic_world2cam_cv"][:3, :3]
        np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-6)
        np.testing.assert_allclose(np.linalg.det(R), 1.0, atol=1e-6)
        # Intrinsics non-negative on diagonal.
        assert c["intrinsic_K"][0, 0] > 0
        assert c["intrinsic_K"][1, 1] > 0


def test_intrinsic_from_fovy_known_values():
    # fovy=90° on 100x100 image → fy = 50 / tan(45°) = 50.0
    K = intrinsic_from_fovy(90.0, 100, 100)
    assert K[0, 0] == pytest.approx(50.0, abs=1e-6)
    assert K[1, 1] == pytest.approx(50.0, abs=1e-6)
    assert K[0, 2] == pytest.approx(50.0)  # cx
    assert K[1, 2] == pytest.approx(50.0)  # cy


def test_extrinsic_round_trip():
    # Identity cam_xmat at origin → extrinsic = OpenCV flip
    T = mujoco_cam_pose_to_opencv_extrinsic(np.zeros(3), np.eye(3))
    expected = np.eye(4)
    expected[:3, :3] = np.diag([1.0, -1.0, -1.0])
    np.testing.assert_allclose(T, expected, atol=1e-9)


def test_calibration_extrinsics_are_finite_and_distinct(env):
    """Different cameras must have different world positions."""
    env.reset(seed=4)
    calib = env.get_calibration()
    positions = {n: c["cam_xpos_world"] for n, c in calib.items()}
    assert all(np.all(np.isfinite(p)) for p in positions.values())
    names = list(positions.keys())
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            diff = np.linalg.norm(positions[names[i]] - positions[names[j]])
            assert diff > 0.01, (
                f"{names[i]} and {names[j]} have nearly identical positions "
                f"({positions[names[i]]} vs {positions[names[j]]})"
            )
