"""Unit tests for `isaac_multicam_addons`.

These tests cover the parts that DON'T require IsaacLab — the geometry
helpers and H5 saver. The IsaacLab-dependent functions
(`add_multicam_to_scene_cfg`, `capture_all_cams`) need a live env and
must be smoke-tested on the RTX5090 box.

Run on the H100 side or anywhere with numpy + h5py + pytest:

    pytest /data/rnd-liu/MyRepo/omniverselab/IsaacSim/test_multicam_addons.py -v
"""

from __future__ import annotations

import sys
from pathlib import Path

import h5py
import numpy as np
import pytest

# Make the sibling module importable regardless of cwd.
sys.path.insert(0, str(Path(__file__).resolve().parent))
import isaac_multicam_addons as mc  # noqa: E402


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def test_quat_wxyz_to_R_identity():
    R = mc._quat_wxyz_to_R(1.0, 0.0, 0.0, 0.0)
    np.testing.assert_allclose(R, np.eye(3), atol=1e-12)


def test_quat_wxyz_to_R_90deg_z():
    # 90° rotation about world +Z: (cos45, 0, 0, sin45)
    R = mc._quat_wxyz_to_R(np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5))
    expected = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=np.float64)
    np.testing.assert_allclose(R, expected, atol=1e-10)


def test_look_at_quat_axis_aligned_z_down():
    # Camera at (0, 0, 2) looking at origin → cam +Z = -world_Z.
    w, x, y, z = mc._look_at_quat_ros((0.0, 0.0, 2.0), (0.0, 0.0, 0.0))
    # rebuild rotation, check fwd axis lines up with -Z world
    R = mc._quat_wxyz_to_R(w, x, y, z)
    cam_fwd_in_world = R[:, 2]
    np.testing.assert_allclose(cam_fwd_in_world, [0.0, 0.0, -1.0], atol=1e-9)


def test_look_at_quat_horizontal():
    # Camera at (1, 0, 0) looking at origin → cam +Z = -X world
    w, x, y, z = mc._look_at_quat_ros((1.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    R = mc._quat_wxyz_to_R(w, x, y, z)
    np.testing.assert_allclose(R[:, 2], [-1.0, 0.0, 0.0], atol=1e-9)
    # cam +Y (down) should align with -world_Z (camera held upright)
    np.testing.assert_allclose(R[:, 1], [0.0, 0.0, -1.0], atol=1e-9)


def test_world2cam_inverse_round_trip():
    """Project the cam's own origin via world2cam: should be (0, 0, 0)."""
    pos = np.array([1.5, -0.3, 0.6])
    # arbitrary rotation: 30° around Y
    c, s = np.cos(np.deg2rad(30)), np.sin(np.deg2rad(30))
    R_w_from_cam = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    # quat (wxyz) for this rotation:
    qw = np.sqrt((R_w_from_cam.trace() + 1.0) / 4.0)
    qx = (R_w_from_cam[2, 1] - R_w_from_cam[1, 2]) / (4 * qw)
    qy = (R_w_from_cam[0, 2] - R_w_from_cam[2, 0]) / (4 * qw)
    qz = (R_w_from_cam[1, 0] - R_w_from_cam[0, 1]) / (4 * qw)

    T = mc.cam_world_pose_to_world2cam_cv(pos, (qw, qx, qy, qz))
    homo = np.array([pos[0], pos[1], pos[2], 1.0])
    projected = T @ homo
    np.testing.assert_allclose(projected[:3], [0, 0, 0], atol=1e-9)
    np.testing.assert_allclose(projected[3], 1.0, atol=1e-12)


def test_world2cam_rotation_orthonormal():
    T = mc.cam_world_pose_to_world2cam_cv((1.0, 2.0, 3.0), (np.sqrt(0.5), 0, 0, np.sqrt(0.5)))
    R = T[:3, :3]
    np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-10)
    assert np.linalg.det(R) == pytest.approx(1.0, abs=1e-10)


def test_intrinsic_from_pinhole_known():
    # focal=24mm, aperture=20.955mm, image 480x640 → fx = 24/20.955 * 640
    K = mc.intrinsic_from_pinhole_cfg(24.0, 20.955, 480, 640)
    expected_fx = 24.0 / 20.955 * 640
    assert K[0, 0] == pytest.approx(expected_fx)
    assert K[1, 1] == pytest.approx(expected_fx)  # square pixels
    assert K[0, 2] == pytest.approx(320.0)
    assert K[1, 2] == pytest.approx(240.0)
    assert K[2, 2] == pytest.approx(1.0)


# ---------------------------------------------------------------------------
# Extra cam defs resolved
# ---------------------------------------------------------------------------
def test_extra_cam_defs_have_rotations():
    for name, d in mc.EXTRA_CAMERA_DEFS.items():
        assert "rot_wxyz_ros" in d, f"{name} missing rot_wxyz_ros"
        w, x, y, z = d["rot_wxyz_ros"]
        # quaternion is unit
        assert (x * x + y * y + z * z + w * w) == pytest.approx(1.0, abs=1e-6)


def test_sensor_name_for():
    assert mc.sensor_name_for("top") == "top_camera"
    assert mc.sensor_name_for("left") == "cam_left"
    assert mc.sensor_name_for("wrist") == "cam_wrist"


# ---------------------------------------------------------------------------
# Saver — synthetic data, no IsaacLab needed
# ---------------------------------------------------------------------------
def test_saver_roundtrip(tmp_path: Path):
    cam_names = ["top", "wrist"]
    intrinsics = {
        c: mc.intrinsic_from_pinhole_cfg(24.0, 20.955, 64, 64) for c in cam_names
    }
    saver = mc.LeRobotMultiCamSaver(
        save_dir=str(tmp_path / "demos"),
        cam_names=cam_names,
        intrinsics_per_cam=intrinsics,
        save_failed=False,
        task_id="Isaac-Lift-Cube-Franka-IK-Rel-v0",
    )
    rng = np.random.default_rng(0)
    T = 15  # > 10, so save() doesn't drop
    for t in range(T):
        cam_records = {
            "top": {
                "rgb": rng.integers(0, 255, (64, 64, 3), dtype=np.uint8),
                "pos_w": np.array([0.5, 0.0, 1.5]),
                "quat_w_wxyz": np.array([1.0, 0.0, 0.0, 0.0]),
            },
            "wrist": {
                "rgb": rng.integers(0, 255, (64, 64, 3), dtype=np.uint8),
                "pos_w": np.array([0.3 + 0.01 * t, 0.0, 0.4]),  # moves over time
                "quat_w_wxyz": np.array([np.sqrt(0.5), 0.0, np.sqrt(0.5), 0.0]),
            },
        }
        joint_pos = np.linspace(0, 0.1, 9, dtype=np.float64)  # 9-d fake state
        action = np.linspace(0, 0.05, 7, dtype=np.float64)
        saver.record_step(joint_pos, action, cam_records)
    fname = saver.save(success=True, episode_info={"test": True})
    assert fname is not None
    assert saver.demo_count == 1

    # Reload and check structure.
    with h5py.File(fname, "r") as f:
        assert f["obs/state"].shape == (T, 9)
        assert f["actions"].shape == (T, 7)
        for c in cam_names:
            assert f[f"obs/images/{c}"].shape == (T, 64, 64, 3)
            assert f[f"obs/images/{c}"].dtype == np.uint8
            assert f[f"obs/cam_extrinsics_world2cam_cv/{c}"].shape == (T, 4, 4)
            assert f[f"obs/cam_pos_world/{c}"].shape == (T, 3)
            assert f[f"obs/cam_quat_world_wxyz/{c}"].shape == (T, 4)
            assert f[f"obs/cam_intrinsics_K/{c}"].shape == (3, 3)
        meta = f.attrs["metadata"]
        import json
        meta_obj = json.loads(meta)
        assert meta_obj["cameras"] == cam_names
        assert meta_obj["success"] is True
        assert meta_obj["num_frames"] == T
        assert meta_obj["format_version"] == "lerobot_multicam_v1"


def test_saver_short_trajectory_dropped(tmp_path: Path):
    saver = mc.LeRobotMultiCamSaver(
        save_dir=str(tmp_path / "demos"), cam_names=["top"],
        intrinsics_per_cam={"top": np.eye(3)},
    )
    for _ in range(5):  # < 10 = dropped
        saver.record_step(
            np.zeros(9), np.zeros(7),
            {"top": {"rgb": np.zeros((4, 4, 3), dtype=np.uint8),
                     "pos_w": np.zeros(3), "quat_w_wxyz": np.array([1.0, 0, 0, 0])}},
        )
    result = saver.save(success=True)
    assert result is None
    assert saver.demo_count == 0


def test_saver_failed_episode_dropped_by_default(tmp_path: Path):
    saver = mc.LeRobotMultiCamSaver(
        save_dir=str(tmp_path / "demos"), cam_names=["top"],
        intrinsics_per_cam={"top": np.eye(3)}, save_failed=False,
    )
    for _ in range(15):
        saver.record_step(
            np.zeros(9), np.zeros(7),
            {"top": {"rgb": np.zeros((4, 4, 3), dtype=np.uint8),
                     "pos_w": np.zeros(3), "quat_w_wxyz": np.array([1.0, 0, 0, 0])}},
        )
    result = saver.save(success=False)
    assert result is None
    assert saver.fail_count == 1
    assert saver.demo_count == 0
