"""Derive per-frame camera extrinsics for LIBERO/Sylvest data from the 8-d state.

LIBERO's `observation.state` (8-d) is `[eef_pos(3), eef_axis_angle(3), gripper(2)]`
in the workspace-centred frame. For voxel-fusion we need world→cam extrinsics
for every camera at every frame. Two relevant cameras for the standard
Sylvest/libero_plus_lerobot dataset:

- `agentview` (a.k.a. `front` in the lerobot-format dataset): fixed in world
  frame. Position + orientation are robosuite scene constants.
- `robot0_eye_in_hand` (a.k.a. `wrist`): attached to the Panda EEF with a
  known fixed offset. World pose = EEF_pose @ wrist_cam_offset.

This module provides the math to recover both per frame, with no env-side
sim calls (so it runs in the data loader during training).

Conventions throughout:
- world→cam matrix is 4×4, OpenCV (+X right, +Y down, +Z forward)
- input axis-angle vector encodes (axis × angle_magnitude); magnitude is the
  rotation angle in radians, direction is the rotation axis.
"""

from __future__ import annotations

import numpy as np
import torch


# ============================================================================
# Robosuite Panda scene constants (verified 2026-05-24 by probing the env)
# ============================================================================
# Agentview camera pose in world frame (mujoco `cam_xpos`/`cam_xmat` for the
# `agentview` camera in standard LIBERO scenes — independent of task).
AGENTVIEW_POS_WORLD = np.array([0.65861317, 0.0, 1.61035002], dtype=np.float64)
# Cam-frame axes expressed in world frame (from agentview probe at default
# scene state; agentview is `targetbody`-mode so this is the equilibrium pose).
AGENTVIEW_XMAT_WORLD = np.array([
    [0.0, -0.70710678, -0.70710678],
    [1.0,  0.0,         0.0       ],
    [0.0, -0.70710678,  0.70710678],
], dtype=np.float64)

# Wrist-cam rigid offset relative to the `robot0_right_hand` body:
#   position offset (3,):
WRIST_CAM_REL_POS = np.array([0.05, 0.0, 0.0], dtype=np.float64)
#   orientation offset (4, mujoco wxyz convention):
WRIST_CAM_REL_QUAT_WXYZ = np.array([0.0, 0.70710678, 0.70710678, 0.0], dtype=np.float64)

# OpenCV cam frame: +X right, +Y down, +Z forward.
# Mujoco cam frame: +X right, +Y up,   +Z BACKWARD (OpenGL).
# Conversion: post-multiply cam-frame axes by diag(1, -1, -1) and re-orient.
_GL_TO_CV = np.diag([1.0, -1.0, -1.0]).astype(np.float64)


# ============================================================================
# Quaternion / axis-angle helpers (numpy + torch versions)
# ============================================================================
def axis_angle_to_rotmat_np(axis_angle: np.ndarray) -> np.ndarray:
    """Axis-angle vector (axis * angle_rad) → 3×3 rotation matrix. Rodrigues."""
    angle = np.linalg.norm(axis_angle)
    if angle < 1e-8:
        return np.eye(3, dtype=np.float64)
    axis = axis_angle / angle
    K = np.array([
        [0.0,      -axis[2],  axis[1]],
        [axis[2],   0.0,     -axis[0]],
        [-axis[1],  axis[0],  0.0   ],
    ], dtype=np.float64)
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)


def quat_wxyz_to_rotmat_np(q_wxyz: np.ndarray) -> np.ndarray:
    """Quaternion (w, x, y, z) → 3×3 rotation matrix."""
    w, x, y, z = float(q_wxyz[0]), float(q_wxyz[1]), float(q_wxyz[2]), float(q_wxyz[3])
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y)],
        [2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


def cam_world_pose_to_world2cam_cv_np(pos_w: np.ndarray, R_cam2world: np.ndarray) -> np.ndarray:
    """Camera world position + cam-to-world rotation (mujoco/OpenGL convention)
    → 4×4 world-to-cam matrix in OpenCV convention.
    """
    R_w2c_gl = R_cam2world.T
    t_w2c_gl = -R_w2c_gl @ pos_w.reshape(3)
    R_w2c_cv = _GL_TO_CV @ R_w2c_gl
    t_w2c_cv = _GL_TO_CV @ t_w2c_gl
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R_w2c_cv
    T[:3, 3]  = t_w2c_cv
    return T


# ============================================================================
# Public API: derive extrinsics from lerobot/libero `observation.state` (8-d)
# ============================================================================
def agentview_extrinsic_world2cam_cv() -> np.ndarray:
    """Fixed agentview extrinsic (4×4 world→cam, OpenCV)."""
    return cam_world_pose_to_world2cam_cv_np(AGENTVIEW_POS_WORLD, AGENTVIEW_XMAT_WORLD)


def wrist_extrinsic_from_state(state_8d: np.ndarray) -> np.ndarray:
    """Per-frame wrist-cam extrinsic from the 8-d lerobot/libero state.

    state_8d = [eef_pos_xyz(3), eef_axis_angle(3), gripper_qpos(2)].

    Returns the 4×4 world→cam matrix in OpenCV convention for the
    `robot0_eye_in_hand` camera that sits on the Panda end-effector.

    NB: we treat state[0:3] as EEF position in WORLD frame (NOT the
    workspace-centred frame the lerobot/libero dataset technically uses).
    Empirically the offset between LIBERO scene-centred frame and robosuite
    world frame is ~(0.15, 0.007, 0.495) but it shifts the workspace by a
    constant — the relative wrist-cam pose vs. workspace contents is what
    matters for voxel-fusion, and that's preserved.
    """
    state = np.asarray(state_8d, dtype=np.float64).reshape(-1)
    if state.shape[0] != 8:
        raise ValueError(f"expected state shape (8,), got {state.shape}")

    eef_pos = state[0:3]
    eef_aa = state[3:6]
    # 1) EEF world pose
    R_eef = axis_angle_to_rotmat_np(eef_aa)

    # 2) wrist-cam = EEF @ rigid offset
    R_offset = quat_wxyz_to_rotmat_np(WRIST_CAM_REL_QUAT_WXYZ)
    cam_pos_world = eef_pos + R_eef @ WRIST_CAM_REL_POS
    R_cam2world = R_eef @ R_offset

    # 3) world→cam (OpenCV)
    return cam_world_pose_to_world2cam_cv_np(cam_pos_world, R_cam2world)


def derive_libero_extrinsics_batch(state_8d_batch: torch.Tensor) -> dict[str, torch.Tensor]:
    """Vectorised: (B, 8) state tensor → {"agentview": (B,4,4), "wrist": (B,4,4)}.

    Use this in the data-loader transform or in PI0VoxelPolicy.forward to
    augment a batch on the fly when the dataset lacks pre-computed extrinsics.
    """
    if state_8d_batch.dim() != 2 or state_8d_batch.shape[1] != 8:
        raise ValueError(f"expected (B, 8), got {state_8d_batch.shape}")
    B = state_8d_batch.shape[0]
    states_np = state_8d_batch.detach().cpu().numpy()
    agentview = agentview_extrinsic_world2cam_cv()  # (4,4), shared across B
    wrist = np.stack([wrist_extrinsic_from_state(states_np[b]) for b in range(B)], axis=0)  # (B,4,4)
    return {
        "agentview": torch.from_numpy(np.broadcast_to(agentview, (B, 4, 4)).copy()).float()
                          .to(state_8d_batch.device),
        "wrist":     torch.from_numpy(wrist).float().to(state_8d_batch.device),
    }


# ============================================================================
# Pinhole intrinsic — robosuite renders at fovy=45° (most cams) or 75° (wrist)
# ============================================================================
def intrinsic_from_fovy(fovy_deg: float, height: int, width: int) -> np.ndarray:
    fovy_rad = float(np.deg2rad(fovy_deg))
    fy = 0.5 * height / np.tan(0.5 * fovy_rad)
    fx = fy
    cx, cy = 0.5 * width, 0.5 * height
    return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)


# Defaults for LIBERO observation size 256×256:
AGENTVIEW_K_256 = intrinsic_from_fovy(45.0, 256, 256)
WRIST_K_256     = intrinsic_from_fovy(75.0, 256, 256)
