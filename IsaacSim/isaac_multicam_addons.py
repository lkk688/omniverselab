"""Multi-camera + extrinsics addons for isaac_auto_collector_v5.

Standalone module — does NOT import IsaacLab at top level (only when called).
Used by isaac_auto_collector_v6.py, which is generated from v5 by
`make_v6_from_v5.py`.

Provides:
  - EXTRA_CAMERA_DEFS                  static-camera positions/orientations
  - add_multicam_to_scene_cfg(...)     mutate env_cfg.scene to add cameras
  - sensor_name_for(cam_name)          map our CLI name -> scene.sensors key
  - capture_all_cams(env, cam_names)   per-step capture helper
  - cam_world_pose_to_world2cam_cv()   IsaacLab pose -> OpenCV 4x4
  - intrinsic_from_pinhole_cfg()       pinhole params -> 3x3 K
  - LeRobotMultiCamSaver               H5 saver with per-frame extrinsics

H5 layout produced (V5-compatible for `obs/images/top` + `obs/state` + `actions`):

    actions                                       [T, 7]
    obs/state                                     [T, J]
    obs/images/<cam>                              [T, H, W, 3]  uint8
    obs/cam_extrinsics_world2cam_cv/<cam>         [T, 4, 4]     float64
    obs/cam_pos_world/<cam>                       [T, 3]        float64
    obs/cam_quat_world_wxyz/<cam>                 [T, 4]        float64
    obs/cam_intrinsics_K/<cam>                    [3, 3]        float64
    attrs["metadata"] = json with cameras, conventions, success, etc.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from datetime import datetime
from typing import Any, Dict, List, Optional

import h5py
import numpy as np
import torch


# ============================================================================
#  Static camera definitions — positions tuned for IsaacLab Franka tabletop.
#  Franka at origin, table at +X 0.5m, workspace centre ~(0.5, 0.0, 0.05).
#  Adjust if your task uses a different layout.
# ============================================================================
WORKSPACE_CENTRE = (0.5, 0.0, 0.05)

EXTRA_CAMERA_DEFS: Dict[str, Dict[str, Any]] = {
    # name : { "pos": (x,y,z), "rot_wxyz_ros": filled by _resolve_extrinsics }
    "top":   {"pos": (0.5,  0.0, 1.25)},
    "left":  {"pos": (0.4,  0.6, 0.4)},
    "right": {"pos": (0.4, -0.6, 0.4)},
    "front": {"pos": (1.1,  0.0, 0.3)},
}


def _look_at_quat_ros(cam_pos, target):
    """ROS-convention quaternion (wxyz) that aims a camera at `target`.

    Cam frame: +X right, +Y down, +Z forward. World up assumed +Z.
    """
    cam_pos = np.asarray(cam_pos, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    fwd = target - cam_pos
    fwd /= np.linalg.norm(fwd) + 1e-12
    world_up = np.array([0.0, 0.0, 1.0])
    right = np.cross(fwd, world_up)
    rn = np.linalg.norm(right)
    if rn < 1e-6:
        right = np.array([1.0, 0.0, 0.0])
    else:
        right /= rn
    down = np.cross(fwd, right)
    R = np.stack([right, down, fwd], axis=1)  # cam-frame axes expressed in world
    # R -> quaternion (wxyz, ROS/IsaacLab OffsetCfg order)
    t = R.trace()
    if t > 0:
        s = 0.5 / np.sqrt(t + 1.0); w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (float(w), float(x), float(y), float(z))


def _resolve_extrinsics():
    for _name, _def in EXTRA_CAMERA_DEFS.items():
        if "rot_wxyz_ros" not in _def:
            _def["rot_wxyz_ros"] = _look_at_quat_ros(_def["pos"], WORKSPACE_CENTRE)


_resolve_extrinsics()


# ============================================================================
#  Scene-cfg injection — call after parse_env_cfg(), before ManagerBasedRLEnv.
# ============================================================================
def add_multicam_to_scene_cfg(env_cfg, cam_names: List[str], height: int, width: int,
                              robot_path_suffix: str = "Robot/panda_hand"):
    """Mutate env_cfg.scene to add fixed-pose + wrist cameras.

    Cameras get `cam_<name>` attrs on the scene cfg, except `top`, which
    uses the conventional `top_camera` attr/key expected by the v5 recorder.
    After env init, the new sensors appear under `env.scene.sensors`.

    Args:
        env_cfg: An IsaacLab ManagerBasedRLEnvCfg-derived cfg.
        cam_names: subset of {"top", "left", "right", "front", "wrist"}.
        height, width: render resolution for ALL new cameras.
        robot_path_suffix: where the wrist cam attaches. Default
            "Robot/panda_hand" works for the standard Franka task cfgs.
    """
    # Imported here (not at module top) so this module can be unit-tested
    # without IsaacLab installed.
    import isaaclab.sim as sim_utils
    from isaaclab.sensors import CameraCfg

    cam_spawn = sim_utils.PinholeCameraCfg(
        focal_length=24.0,
        focus_distance=400.0,
        horizontal_aperture=20.955,
        clipping_range=(0.01, 1.0e5),
    )
    for name in cam_names:
        if name == "wrist":
            cfg = CameraCfg(
                prim_path="{ENV_REGEX_NS}/" + robot_path_suffix + "/wrist_cam",
                update_period=0.0,
                height=height, width=width,
                data_types=["rgb"],
                spawn=cam_spawn,
                offset=CameraCfg.OffsetCfg(
                    pos=(0.05, 0.0, 0.0),
                    rot=(0.0, 0.0, 0.7071, 0.7071),  # looking +Z along the hand
                    convention="ros",
                ),
            )
        elif name in EXTRA_CAMERA_DEFS:
            d = EXTRA_CAMERA_DEFS[name]
            attr = "top_camera" if name == "top" else f"cam_{name}"
            prim_name = "top_camera" if name == "top" else f"cam_{name}"
            cfg = CameraCfg(
                prim_path="{ENV_REGEX_NS}/" + prim_name,
                update_period=0.0,
                height=height, width=width,
                data_types=["rgb"],
                spawn=cam_spawn,
                offset=CameraCfg.OffsetCfg(
                    pos=tuple(d["pos"]),
                    rot=tuple(d["rot_wxyz_ros"]),
                    convention="ros",
                ),
            )
        else:
            print(f"[multicam] WARNING: unknown camera '{name}', skipping.")
            continue
        if name == "wrist":
            attr = f"cam_{name}"
        setattr(env_cfg.scene, attr, cfg)
        print(f"[multicam] registered scene.{attr} at pos={cfg.offset.pos}")


def sensor_name_for(cam_name: str) -> str:
    """CLI name -> key in env.scene.sensors."""
    return "top_camera" if cam_name == "top" else f"cam_{cam_name}"


# ============================================================================
#  Per-step capture
# ============================================================================
def capture_all_cams(env, cam_names: List[str]) -> Dict[str, Dict[str, Any]]:
    """Return {cam_name: {"rgb", "pos_w", "quat_w_wxyz"}} for the current step.

    Skips cameras whose sensor isn't found (e.g. if top_camera missing from
    the task's default scene cfg). Returned tensors are still GPU-resident;
    saver moves to CPU.
    """
    out: Dict[str, Dict[str, Any]] = {}
    if not hasattr(env.scene, "sensors"):
        return out
    for c in cam_names:
        key = sensor_name_for(c)
        if key not in env.scene.sensors:
            continue
        sensor = env.scene.sensors[key]
        try:
            out[c] = {
                "rgb":         sensor.data.output["rgb"][0],
                "pos_w":       sensor.data.pos_w[0],
                "quat_w_wxyz": sensor.data.quat_w_world[0],
            }
        except (AttributeError, KeyError) as e:
            print(f"[multicam] couldn't read sensor '{key}': {e}")
    return out


# ============================================================================
#  Pose conversion
# ============================================================================
def _quat_wxyz_to_R(qw, qx, qy, qz) -> np.ndarray:
    """Quaternion (wxyz) -> 3x3 rotation matrix."""
    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qw*qz),     2*(qx*qz + qw*qy)],
        [2*(qx*qy + qw*qz),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qw*qx)],
        [2*(qx*qz - qw*qy),     2*(qy*qz + qw*qx),     1 - 2*(qx*qx + qy*qy)],
    ], dtype=np.float64)


def cam_world_pose_to_world2cam_cv(pos_w, quat_w_wxyz) -> np.ndarray:
    """IsaacLab pose (pos_w, quat_w_world) -> 4x4 world-to-cam (OpenCV).

    For ROS-convention cameras (we registered them as `convention="ros"`):
    the cam frame already matches OpenCV image frame (+X right, +Y down,
    +Z forward), so no extra flip needed.
    """
    R_world_from_cam = _quat_wxyz_to_R(*[float(q) for q in quat_w_wxyz])
    R_w2c = R_world_from_cam.T
    t_w2c = -R_w2c @ np.asarray(pos_w, dtype=np.float64).reshape(3)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R_w2c
    T[:3, 3]  = t_w2c
    return T


def intrinsic_from_pinhole_cfg(focal_length_mm: float, horizontal_aperture_mm: float,
                                height: int, width: int) -> np.ndarray:
    """Pinhole K from IsaacLab PinholeCameraCfg-style sensor params."""
    fx = (focal_length_mm / horizontal_aperture_mm) * width
    fy = fx  # square pixel
    cx = width / 2.0
    cy = height / 2.0
    return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)


# ============================================================================
#  H5 saver — extends V5's LeRobotDemoSaver with multi-cam + extrinsics
# ============================================================================
class LeRobotMultiCamSaver:
    def __init__(self, save_dir: str, cam_names: List[str],
                 intrinsics_per_cam: Dict[str, np.ndarray],
                 save_failed: bool = False, task_id: str = ""):
        self.save_dir = save_dir
        self.cam_names = list(cam_names)
        self.intrinsics = dict(intrinsics_per_cam)
        self.save_failed = save_failed
        self.task_id = task_id
        os.makedirs(save_dir, exist_ok=True)
        if save_failed:
            os.makedirs(os.path.join(save_dir, "failed"), exist_ok=True)
        self.states: List[np.ndarray] = []
        self.actions: List[np.ndarray] = []
        self.images: Dict[str, List[np.ndarray]] = {c: [] for c in self.cam_names}
        self.extrinsics: Dict[str, List[np.ndarray]] = {c: [] for c in self.cam_names}
        self.pos_w: Dict[str, List[np.ndarray]] = {c: [] for c in self.cam_names}
        self.quat_w: Dict[str, List[np.ndarray]] = {c: [] for c in self.cam_names}
        self.demo_count = 0
        self.success_count = 0
        self.fail_count = 0

    def record_step(self, joint_pos, action, cam_records: Dict[str, Dict[str, Any]]):
        """cam_records: {cam_name: {"rgb": HxWx3 (uint8 np|tensor),
                                     "pos_w": (3,), "quat_w_wxyz": (4,)}}"""
        self.states.append(joint_pos.cpu().numpy() if torch.is_tensor(joint_pos) else joint_pos)
        self.actions.append(action.cpu().numpy() if torch.is_tensor(action) else action)
        for c in self.cam_names:
            rec = cam_records.get(c)
            if rec is None:
                continue
            rgb = rec["rgb"]
            if torch.is_tensor(rgb):
                rgb = rgb.cpu().numpy()
            self.images[c].append(np.asarray(rgb, dtype=np.uint8))
            pos = rec["pos_w"]
            if torch.is_tensor(pos):
                pos = pos.cpu().numpy()
            self.pos_w[c].append(np.asarray(pos, dtype=np.float64).reshape(3))
            qw = rec["quat_w_wxyz"]
            if torch.is_tensor(qw):
                qw = qw.cpu().numpy()
            self.quat_w[c].append(np.asarray(qw, dtype=np.float64).reshape(4))
            self.extrinsics[c].append(
                cam_world_pose_to_world2cam_cv(self.pos_w[c][-1], self.quat_w[c][-1])
            )

    def save(self, success: bool = True,
             episode_info: Optional[Dict[str, Any]] = None) -> Optional[str]:
        if len(self.actions) < 10:
            print(f"  Trajectory too short ({len(self.actions)} steps), skipping.")
            self.clear()
            return None
        if not success and not self.save_failed:
            print(f"  Discarding failed episode (fail #{self.fail_count + 1})")
            self.fail_count += 1
            self.clear()
            return None

        if success:
            subdir = self.save_dir
            idx = self.demo_count
        else:
            subdir = os.path.join(self.save_dir, "failed")
            self.fail_count += 1
            idx = self.fail_count

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        tag = "demo" if success else "fail"
        filename = os.path.join(subdir, f"{tag}_{idx:04d}_{ts}.hdf5")

        with h5py.File(filename, "w") as f:
            f.create_dataset("obs/state", data=np.array(self.states),
                             compression="gzip", compression_opts=4)
            f.create_dataset("actions", data=np.array(self.actions),
                             compression="gzip", compression_opts=4)
            for c in self.cam_names:
                if not self.images[c]:
                    continue
                f.create_dataset(f"obs/images/{c}", data=np.stack(self.images[c]),
                                 compression="gzip", compression_opts=4)
                f.create_dataset(f"obs/cam_extrinsics_world2cam_cv/{c}",
                                 data=np.stack(self.extrinsics[c]))
                f.create_dataset(f"obs/cam_pos_world/{c}",
                                 data=np.stack(self.pos_w[c]))
                f.create_dataset(f"obs/cam_quat_world_wxyz/{c}",
                                 data=np.stack(self.quat_w[c]))
                if c in self.intrinsics:
                    f.create_dataset(f"obs/cam_intrinsics_K/{c}",
                                     data=self.intrinsics[c])
            meta = {
                "timestamp": ts,
                "episode_index": idx,
                "num_frames": len(self.actions),
                "success": success,
                "format_version": "lerobot_multicam_v1",
                "action_space": "ik_rel_7d",
                "robot": "franka_panda",
                "task": self.task_id,
                "cameras": self.cam_names,
                "extrinsics_convention": "world_to_cam_opencv",
                "intrinsics_convention": "pinhole_K_3x3",
            }
            if episode_info:
                meta.update(episode_info)
            f.attrs["metadata"] = json.dumps(meta)

        if success:
            self.demo_count += 1
            self.success_count += 1
        print(f"  Saved: {filename} ({len(self.actions)} steps, {len([c for c in self.cam_names if self.images[c]])} cams)")
        print(f"  Stats: {self.success_count} ok, {self.fail_count} fail")
        self.clear()
        return filename

    def clear(self):
        self.states.clear()
        self.actions.clear()
        for c in self.cam_names:
            self.images[c].clear()
            self.extrinsics[c].clear()
            self.pos_w[c].clear()
            self.quat_w[c].clear()

    @property
    def total_demos(self) -> int:
        return self.demo_count
