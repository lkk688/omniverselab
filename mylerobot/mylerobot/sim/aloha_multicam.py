"""Multi-camera wrapper around `gym_aloha.AlohaEnv`.

The default `AlohaEnv` only renders `camera_id="top"`. The aloha
`scene.xml` already defines 5 cameras (`top`, `left_pillar`, `right_pillar`,
`angle`, `front_close`), they're just not exposed through the gym observation
dict. This wrapper fixes that and adds camera calibration extraction.

What you get from this wrapper that vanilla AlohaEnv doesn't give you:
  * `obs["pixels"]` becomes a dict {camera_name: HxWx3 uint8 array}
    instead of just {"top": ...}.
  * `env.get_calibration()` returns per-camera 4x4 world-to-cam extrinsics
    (OpenCV convention) and 3x3 intrinsics derived from fovy + image size.
  * `env.add_camera(...)` lets you inject extra cameras at runtime by
    appending to the MuJoCo XML model (e.g. wrist-mounted cameras).
    [Not implemented in this revision — placeholder for Phase 3 stretch goal.]

Conventions
-----------
MuJoCo cameras look down -Z (OpenGL convention) with +Y up. We expose
extrinsics in the OpenCV convention (camera looks down +Z, +Y down) so
they can be fed directly to standard reprojection / lift-splat code.
The conversion is a fixed 180° rotation around the camera's X axis:
    R_opencv = R_mujoco @ diag(1, -1, -1)
"""

from __future__ import annotations

from typing import Any, Sequence

import gymnasium as gym
import numpy as np
from gymnasium import spaces

from gym_aloha.env import AlohaEnv

# Cameras defined in gym_aloha/assets/scene.xml. All exist out of the box.
DEFAULT_SCENE_CAMERAS = ("top", "left_pillar", "right_pillar", "angle")
# 'front_close' targets vx300s_left/camera_focus (moves with the arm). Useful
# but disabled by default so static-extrinsic experiments aren't surprised.
OPTIONAL_DYNAMIC_CAMERAS = ("front_close",)

# OpenCV ←→ OpenGL conversion (negate Y and Z axes in camera frame).
_GL_TO_CV = np.diag([1.0, -1.0, -1.0]).astype(np.float64)


def intrinsic_from_fovy(fovy_deg: float, height: int, width: int) -> np.ndarray:
    """Standard pinhole intrinsic matrix from MuJoCo fovy and image size.

    MuJoCo's `fovy` is the vertical field of view in degrees. We assume
    square pixels (fx == fy), giving a 3x3 K = [[fx,0,cx],[0,fy,cy],[0,0,1]].
    """
    fovy_rad = np.deg2rad(fovy_deg)
    fy = 0.5 * height / np.tan(0.5 * fovy_rad)
    fx = fy  # square pixels
    cx = 0.5 * width
    cy = 0.5 * height
    K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
    return K


def mujoco_cam_pose_to_opencv_extrinsic(cam_xpos: np.ndarray, cam_xmat: np.ndarray) -> np.ndarray:
    """Convert MuJoCo (cam_xpos, cam_xmat) → 4x4 world-to-camera matrix in OpenCV frame.

    `cam_xmat` is the camera's rotation in world frame (camera-to-world, OpenGL).
    We invert (transpose) to world-to-camera, then flip Y/Z to get OpenCV
    (camera looks down +Z, +Y down).
    """
    R_cam2world_gl = np.asarray(cam_xmat).reshape(3, 3)
    R_world2cam_gl = R_cam2world_gl.T
    t_world2cam_gl = -R_world2cam_gl @ np.asarray(cam_xpos).reshape(3)
    # GL → CV: post-multiply by diag(1,-1,-1) in camera frame
    R_world2cam_cv = _GL_TO_CV @ R_world2cam_gl
    t_world2cam_cv = _GL_TO_CV @ t_world2cam_gl
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R_world2cam_cv
    T[:3, 3] = t_world2cam_cv
    return T


class AlohaMultiCamEnv(AlohaEnv):
    """`AlohaEnv` that exposes multiple cameras + extrinsics.

    Args
    ----
    task            : "insertion" or "transfer_cube" (same as AlohaEnv).
    cameras         : sequence of MuJoCo camera names to render each step.
                      Default: all 4 static scene cameras (top/left/right/angle).
    include_dynamic : if True, also render `front_close` (moves with left arm).
    obs_height/width: render size (default 480x640 matches aloha_static_* shape).
    """

    def __init__(
        self,
        task: str,
        cameras: Sequence[str] | None = None,
        include_dynamic: bool = False,
        observation_width: int = 640,
        observation_height: int = 480,
        visualization_width: int = 640,
        visualization_height: int = 480,
    ):
        super().__init__(
            task=task,
            obs_type="pixels_agent_pos",
            render_mode="rgb_array",
            observation_width=observation_width,
            observation_height=observation_height,
            visualization_width=visualization_width,
            visualization_height=visualization_height,
        )
        if cameras is None:
            cameras = list(DEFAULT_SCENE_CAMERAS) + (
                list(OPTIONAL_DYNAMIC_CAMERAS) if include_dynamic else []
            )
        self.cameras = list(cameras)
        self._validate_cameras()

        # Override observation_space to advertise multiple cameras.
        img_space = lambda: spaces.Box(  # noqa: E731
            low=0, high=255,
            shape=(self.observation_height, self.observation_width, 3),
            dtype=np.uint8,
        )
        self.observation_space = spaces.Dict({
            "pixels": spaces.Dict({name: img_space() for name in self.cameras}),
            "agent_pos": spaces.Box(
                low=-1000.0, high=1000.0,
                shape=self.observation_space["agent_pos"].shape,
                dtype=np.float64,
            ),
        })

    def _validate_cameras(self) -> None:
        available = self._available_camera_names()
        missing = [c for c in self.cameras if c not in available]
        if missing:
            raise ValueError(
                f"Cameras {missing} not found in MuJoCo model. "
                f"Available: {sorted(available)}"
            )

    def _available_camera_names(self) -> list[str]:
        """All cameras defined in the loaded XML."""
        model = self._env.physics.model
        names: list[str] = []
        for i in range(model.ncam):
            n = model.id2name(i, "camera")
            if n:
                names.append(n)
        return names

    def _format_raw_obs(self, raw_obs):
        # raw_obs["qpos"] is the 14-d agent state. raw_obs["images"]["top"]
        # is whatever the task pre-rendered (which we ignore — we render all
        # cameras ourselves at the wrapper's chosen size).
        images = self.render_all_cameras()
        return {
            "pixels": images,
            "agent_pos": np.asarray(raw_obs["qpos"]),
        }

    def render_camera(self, name: str) -> np.ndarray:
        return self._env.physics.render(
            height=self.observation_height,
            width=self.observation_width,
            camera_id=name,
        )

    def render_all_cameras(self) -> dict[str, np.ndarray]:
        return {name: self.render_camera(name) for name in self.cameras}

    def get_calibration(self) -> dict[str, dict[str, Any]]:
        """Per-camera extrinsic (4x4, OpenCV, world→cam) + intrinsic (3x3) + raw MuJoCo pose.

        Call AFTER `reset()` / `step()` so cam_xmat reflects the current
        target-body pose (cameras using mode="targetbody" track their target
        each step).
        """
        physics = self._env.physics
        out: dict[str, dict[str, Any]] = {}
        for name in self.cameras:
            xpos = np.asarray(physics.named.data.cam_xpos[name], dtype=np.float64).copy()
            xmat = np.asarray(physics.named.data.cam_xmat[name], dtype=np.float64).reshape(3, 3).copy()
            fovy = float(physics.named.model.cam_fovy[name])
            K = intrinsic_from_fovy(fovy, self.observation_height, self.observation_width)
            T_world2cam_cv = mujoco_cam_pose_to_opencv_extrinsic(xpos, xmat)
            out[name] = {
                "cam_xpos_world": xpos,           # (3,) raw MuJoCo
                "cam_xmat_world": xmat,           # (3,3) raw MuJoCo (cam→world, GL)
                "fovy_deg": fovy,
                "intrinsic_K": K,                 # (3,3)
                "extrinsic_world2cam_cv": T_world2cam_cv,  # (4,4)
            }
        return out


def make_aloha_multicam_env(
    task: str = "insertion",
    cameras: Sequence[str] | None = None,
    include_dynamic: bool = False,
    **kwargs,
) -> gym.Env:
    """Convenience factory mirroring `gym.make` for `gym_aloha`."""
    return AlohaMultiCamEnv(
        task=task, cameras=cameras, include_dynamic=include_dynamic, **kwargs
    )
