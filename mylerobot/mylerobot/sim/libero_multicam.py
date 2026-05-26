"""Multi-camera wrapper around `libero.libero.envs.OffScreenRenderEnv`.

LIBERO's default scene exposes only 2 cameras (`agentview` + `robot0_eye_in_hand`),
but the underlying MuJoCo model has **7 cameras defined**:

    frontview, birdview, agentview, sideview, galleryview,
    robot0_robotview, robot0_eye_in_hand

This wrapper renders any subset of those and pulls their world-to-camera
extrinsics (OpenCV convention) + pinhole intrinsics from the mujoco model,
so we can produce datasets suitable for our voxel-fusion module.

Conventions match `mylerobot/sim/aloha_multicam.py` and
`IsaacSim/isaac_multicam_addons.py`:
  - extrinsic: 4x4 world→camera in OpenCV (+X right, +Y down, +Z forward)
  - intrinsic: 3x3 K matrix in pixel units of the rendered image
"""

from __future__ import annotations

from typing import Sequence

import numpy as np

from libero.libero.envs import OffScreenRenderEnv


ALL_LIBERO_CAMERAS = (
    "frontview", "birdview", "agentview", "sideview",
    "galleryview", "robot0_robotview", "robot0_eye_in_hand",
)
# Default 5 cams: agentview + eye_in_hand (LIBERO default, for compat) + 3 diverse
DEFAULT_MULTICAM = ("agentview", "robot0_eye_in_hand", "frontview", "sideview", "birdview")

# MuJoCo cam frame: +X right, +Y up, +Z BACKWARD (camera looks down -Z, OpenGL).
# OpenCV image frame: +X right, +Y down, +Z forward.
# Conversion R_opencv_from_mujoco = diag(1, -1, -1).
_GL_TO_CV = np.diag([1.0, -1.0, -1.0]).astype(np.float64)


def intrinsic_from_mujoco_camera(fovy_deg: float, height: int, width: int) -> np.ndarray:
    """Pinhole K matrix from mujoco camera fovy + image size."""
    fovy_rad = np.deg2rad(fovy_deg)
    fy = 0.5 * height / np.tan(0.5 * fovy_rad)
    fx = fy  # square pixels
    cx = 0.5 * width
    cy = 0.5 * height
    return np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)


def mujoco_cam_pose_to_world2cam_cv(cam_xpos: np.ndarray, cam_xmat: np.ndarray) -> np.ndarray:
    """Mujoco (cam_xpos, cam_xmat) -> 4x4 world-to-cam in OpenCV convention.

    `cam_xmat` is the camera's rotation in world frame (cam→world, OpenGL).
    We invert (transpose) to world→cam, then flip Y and Z to get OpenCV.
    """
    R_cam2world_gl = np.asarray(cam_xmat, dtype=np.float64).reshape(3, 3)
    R_world2cam_gl = R_cam2world_gl.T
    t_world2cam_gl = -R_world2cam_gl @ np.asarray(cam_xpos, dtype=np.float64).reshape(3)
    R_world2cam_cv = _GL_TO_CV @ R_world2cam_gl
    t_world2cam_cv = _GL_TO_CV @ t_world2cam_gl
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R_world2cam_cv
    T[:3, 3] = t_world2cam_cv
    return T


class LiberoMultiCamEnv(OffScreenRenderEnv):
    """OffScreenRenderEnv exposing N cameras + per-step extrinsics.

    Args (passed via kwargs to OffScreenRenderEnv):
        bddl_file_name: path to the task BDDL file (LIBERO standard).
        camera_names: list of mujoco camera names to render. Default: 5 cams.
        camera_heights / camera_widths: render size for ALL cameras.

    Methods added on top of OffScreenRenderEnv:
        get_calibration() -> dict[cam_name -> {extrinsic, intrinsic, pos_w, xmat_w}]
            Call AFTER reset() / step() so cam poses reflect the current scene.
            For static cams the extrinsic doesn't change between steps; for
            wrist-attached ones it does. Caller should sample per-step.
    """

    def __init__(self, *, camera_names: Sequence[str] = DEFAULT_MULTICAM,
                 camera_heights: int = 256, camera_widths: int = 256, **kwargs):
        camera_names = list(camera_names)
        # Validate
        for c in camera_names:
            if c not in ALL_LIBERO_CAMERAS:
                raise ValueError(
                    f"Unknown LIBERO camera '{c}'. Known: {ALL_LIBERO_CAMERAS}"
                )
        kwargs["camera_names"] = camera_names
        kwargs["camera_heights"] = camera_heights
        kwargs["camera_widths"] = camera_widths
        super().__init__(**kwargs)
        self.cameras = camera_names
        self.observation_height = camera_heights
        self.observation_width = camera_widths

    def _mj_model_data(self):
        # robosuite -> ControlEnv.env is the underlying robosuite RobotEnv; .sim is the mujoco sim
        return self.env.sim.model, self.env.sim.data

    def _cam_id(self, name: str) -> int:
        model, _ = self._mj_model_data()
        cid = model.camera_name2id(name)
        if cid < 0:
            raise RuntimeError(f"Camera '{name}' not found in mujoco model")
        return cid

    def get_calibration(self) -> dict[str, dict[str, np.ndarray]]:
        """Per-camera extrinsic + intrinsic for the current scene.

        Call after env.reset() / env.step(). For wrist cameras the
        extrinsic changes per step; for static cameras it doesn't, but
        we still sample fresh each time to be safe.
        """
        model, data = self._mj_model_data()
        H, W = self.observation_height, self.observation_width
        out: dict[str, dict[str, np.ndarray]] = {}
        for name in self.cameras:
            cid = self._cam_id(name)
            xpos = np.array(data.cam_xpos[cid], dtype=np.float64).copy()
            xmat = np.array(data.cam_xmat[cid], dtype=np.float64).reshape(3, 3).copy()
            fovy = float(model.cam_fovy[cid])
            out[name] = {
                "cam_xpos_world": xpos,
                "cam_xmat_world": xmat,  # cam→world, OpenGL
                "fovy_deg": fovy,
                "intrinsic_K": intrinsic_from_mujoco_camera(fovy, H, W),
                "extrinsic_world2cam_cv": mujoco_cam_pose_to_world2cam_cv(xpos, xmat),
            }
        return out

    def render_all_cameras(self, obs: dict | None = None) -> dict[str, np.ndarray]:
        """Return {cam_name: (H, W, 3) uint8} for the current step.

        If `obs` is the dict returned by reset()/step(), pulls images from it
        (zero-cost — robosuite already rendered them). Otherwise renders again.
        """
        if obs is not None:
            out = {}
            for cam in self.cameras:
                key = f"{cam}_image"
                if key in obs:
                    out[cam] = np.asarray(obs[key], dtype=np.uint8)
                else:
                    raise KeyError(f"obs missing '{key}'. Was camera '{cam}' in camera_names?")
            return out
        # Fallback: re-render explicitly (slower)
        rendered = {}
        for cam in self.cameras:
            rendered[cam] = self.env.sim.render(
                width=self.observation_width, height=self.observation_height, camera_name=cam
            )[::-1]  # mujoco renders upside-down
        return rendered


# ============================================================================
# LiberoMultiCamCaptureEnv  —  subclass of lerobot's LiberoEnv
# ============================================================================
# Why this exists separately from LiberoMultiCamEnv above:
#   * LiberoMultiCamEnv = simple OffScreenRenderEnv wrapper, used by our
#     standalone smoke tests, no lerobot dependency.
#   * LiberoMultiCamCaptureEnv = subclass of lerobot's LiberoEnv adapter.
#     Used for re-rendering datasets via lerobot's validated pi0 pipeline.
#     Standard observation is unchanged (so pi0 still hits its 66% baseline);
#     extra cameras + extrinsics are exposed as a side channel via
#     `env.recent_extras` after every step/reset.

from typing import Any  # noqa: E402  (kept here to localise subclass deps)


class LiberoMultiCamCaptureEnv:
    """Lazy-imported wrapper. We import lerobot.envs.libero at construction
    time (not at module load) so this file can be partially used without
    triggering lerobot's env-shim chain. See `make_capture_env(...)` below.
    """
    pass


def make_capture_env(
    task_suite,
    task_id: int,
    task_suite_name: str,
    extra_cameras: Sequence[str] = ("frontview", "sideview", "birdview"),
    observation_height: int = 256,
    observation_width: int = 256,
    **liberoenv_kwargs,
):
    """Factory: return a LiberoEnv subclass instance with multi-cam capture.

    The subclass overrides:
      * `_ensure_env`: creates underlying OffScreenRenderEnv with all
        cameras (lerobot's default 2 + our `extra_cameras`).
      * `_format_raw_obs`: captures extras into `self.recent_extras` before
        delegating to parent. No extra render cost — `raw_obs` already has
        the extra cam images (robosuite renders all configured cams).

    After every reset() / step(), call `env.get_recent_extras()` to get:
        {cam_name: {"image": HxWx3 uint8,
                    "extrinsic_world2cam_cv": (4,4) float64,
                    "intrinsic_K": (3,3) float64}}
    """
    from lerobot.envs.libero import LiberoEnv
    from libero.libero.envs import OffScreenRenderEnv as _OSE

    class _CaptureLiberoEnv(LiberoEnv):
        def __init__(self_, *args, **kwargs):
            self_._extra_cams = tuple(extra_cameras)
            self_.recent_extras: dict | None = None
            super().__init__(*args, **kwargs)

        def _all_cams_mujoco(self_):
            """Return mujoco short camera names (without `_image` suffix).

            lerobot's `camera_name` may use `<name>_image` style for obs lookup,
            but robosuite's `OffScreenRenderEnv(camera_names=...)` wants the
            bare mujoco model camera name. Normalize here.
            """
            def strip(c: str) -> str:
                return c[: -len("_image")] if c.endswith("_image") else c
            base = [strip(c) for c in self_.camera_name]
            extras = [strip(c) for c in self_._extra_cams]
            seen: set = set()
            ordered: list = []
            for c in base + extras:
                if c not in seen:
                    ordered.append(c)
                    seen.add(c)
            return ordered

        def _ensure_env(self_) -> None:
            if self_._env is not None:
                return
            # Override parent to pass extended camera list to OffScreenRenderEnv.
            env = _OSE(
                bddl_file_name=self_._task_bddl_file,
                camera_names=self_._all_cams_mujoco(),
                camera_heights=self_.observation_height,
                camera_widths=self_.observation_width,
            )
            env.reset()
            self_._env = env

        def _capture_extras_from_raw(self_, raw_obs: dict) -> None:
            sim = self_._env.env.sim
            H, W = self_.observation_height, self_.observation_width
            captured: dict[str, dict[str, Any]] = {}
            for cam in self_._all_cams_mujoco():
                img_key = f"{cam}_image"  # robosuite obs convention
                img = np.asarray(raw_obs[img_key], dtype=np.uint8) if img_key in raw_obs else None
                cam_id = sim.model.camera_name2id(cam)
                xpos = np.array(sim.data.cam_xpos[cam_id], dtype=np.float64)
                xmat = np.array(sim.data.cam_xmat[cam_id], dtype=np.float64).reshape(3, 3)
                fovy = float(sim.model.cam_fovy[cam_id])
                captured[cam] = {
                    "image": img,
                    "extrinsic_world2cam_cv": mujoco_cam_pose_to_world2cam_cv(xpos, xmat),
                    "intrinsic_K": intrinsic_from_mujoco_camera(fovy, H, W),
                }
            self_.recent_extras = captured

        def _format_raw_obs(self_, raw_obs):
            # Capture BEFORE delegating to parent; parent formatting unchanged.
            self_._capture_extras_from_raw(raw_obs)
            return super()._format_raw_obs(raw_obs)

        def get_recent_extras(self_) -> dict | None:
            return self_.recent_extras

    return _CaptureLiberoEnv(
        task_suite=task_suite,
        task_id=task_id,
        task_suite_name=task_suite_name,
        observation_height=observation_height,
        observation_width=observation_width,
        **liberoenv_kwargs,
    )
