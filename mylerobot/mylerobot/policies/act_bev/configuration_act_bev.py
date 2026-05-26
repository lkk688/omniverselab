"""ACT-BEV: ACT with a calibrated multi-view BEV/voxel encoder + cube-heatmap aux head.

Design summary
--------------
- Replace ACT's per-camera ResNet branch (one feature map per camera) with ONE
  BEV feature map produced by lift-splat from a set of calibrated cameras.
- The BEV grid is fixed in the WORKSPACE (world frame) — voxels at metric
  positions, so the network learns to *read* a metrically grounded feature,
  not infer 3D geometry from pixels alone.
- An auxiliary cube-heatmap head predicts where the target object is on the
  BEV grid. Supervision comes from ground-truth `obs/cube_pose` recorded by
  the collector. This is the dense gradient signal that breaks the proprio
  shortcut in 200-demo regimes.

Camera calibration
------------------
For the Franka tabletop Isaac scene with `--cams top,front,wrist` recorded at
480×640 and resized to 240×320, the static intrinsics & extrinsics for the
`top` and `front` cameras are hard-coded (`get_static_bev_calibration`). The
wrist camera is intentionally NOT included in BEV phase 1: its extrinsics
change every frame (and the v6 collector's saved value is stale anyway). The
wrist branch is added in a later phase, deriving extrinsics on-the-fly from
panda_hand state via FK.

References
----------
- LSS lifting: Philion & Fidler, "Lift, Splat, Shoot", ECCV 2020.
- Voxel aggregation across views via depth-agnostic mean pooling (we don't
  predict depth — geometry is fixed by calibration and we sample features at
  every voxel-camera intersection).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Tuple

import numpy as np

from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.act.configuration_act import ACTConfig


# --- Static BEV calibration ---------------------------------------------------
# We derive extrinsics from scratch via look_at_opencv() — the per-frame
# matrices saved by the v6 collector are based on Isaac's internal camera
# quaternion whose convention differs from OpenCV in a way that puts the
# workspace at z_cam=0 (degenerate projection). Computing them here from the
# known camera positions + look-at targets fixes that.

_K_480x640 = np.array(
    # focal_length=14mm, horizontal_aperture=20.955mm, image_w=640
    # fx = fy = (14 / 20.955) * 640 = 427.49
    [[427.49958244, 0.0, 320.0],
     [0.0, 427.49958244, 240.0],
     [0.0, 0.0, 1.0]],
    dtype=np.float64,
)

_STATIC_CAMS = {
    "top":   {"pos": (0.5, 0.0, 1.25), "target": (0.5, 0.0, 0.05)},
    "front": {"pos": (1.0, 0.5, 0.4),  "target": (0.5, 0.0, 0.05)},  # front-left corner agent view
}


def _look_at_opencv_w2c(cam_pos, target, world_up=(0.0, 0.0, 1.0)):
    """world->camera 4x4 matrix for an OpenCV-frame camera (+X right, +Y down,
    +Z forward) located at cam_pos and pointing at target."""
    cam_pos = np.asarray(cam_pos, dtype=np.float64)
    target = np.asarray(target, dtype=np.float64)
    world_up = np.asarray(world_up, dtype=np.float64)

    fwd = target - cam_pos
    fwd /= np.linalg.norm(fwd) + 1e-12
    right = np.cross(fwd, world_up)
    rn = np.linalg.norm(right)
    if rn < 1e-6:
        # Camera looking straight up/down — choose an arbitrary right axis.
        right = np.array([1.0, 0.0, 0.0])
    else:
        right /= rn
    down = np.cross(fwd, right)

    R_c2w = np.stack([right, down, fwd], axis=1)
    R_w2c = R_c2w.T
    t_w2c = -R_w2c @ cam_pos
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R_w2c
    T[:3, 3] = t_w2c
    return T


def get_static_bev_calibration(
    cam_keys: Tuple[str, ...],
    policy_hw: Tuple[int, int] = (240, 320),
) -> Tuple[np.ndarray, np.ndarray]:
    """Return (intrinsics_K (N,3,3), extrinsics_w2c (N,4,4)) for the BEV camera set,
    rescaled to the policy input resolution.

    Args:
        cam_keys: tuple of LeRobotDataset feature names, e.g.
            ("observation.images.top", "observation.images.front")
        policy_hw: (H, W) at which the policy reads images. K is rescaled by
            H/480 (and W/640) on the relevant axes.
    """
    if len(cam_keys) == 0:
        raise ValueError("cam_keys is empty")
    K_list = []
    E_list = []
    scale_h = policy_hw[0] / 480.0
    scale_w = policy_hw[1] / 640.0
    for key in cam_keys:
        name = key.split(".")[-1]  # "top" / "front"
        if name not in _STATIC_CAMS:
            raise ValueError(
                f"BEV calibration not available for camera '{name}'. "
                f"Supported: {list(_STATIC_CAMS.keys())}."
            )
        K = _K_480x640.copy()
        spec = _STATIC_CAMS[name]
        E = _look_at_opencv_w2c(spec["pos"], spec["target"])
        K[0, 0] *= scale_w
        K[1, 1] *= scale_h
        K[0, 2] *= scale_w
        K[1, 2] *= scale_h
        K_list.append(K)
        E_list.append(E)
    return np.stack(K_list, axis=0), np.stack(E_list, axis=0)


@PreTrainedConfig.register_subclass("act_bev")
@dataclass
class ACTBEVConfig(ACTConfig):
    """ACT-BEV configuration.

    Adds a calibrated multi-view BEV encoder and a cube-heatmap auxiliary head
    on top of the standard ACT spine.
    """

    # ─── BEV voxel grid (world frame, metres) ────────────────────────────────
    voxel_x_range: Tuple[float, float] = (0.20, 0.80)
    voxel_y_range: Tuple[float, float] = (-0.30, 0.30)
    voxel_z_range: Tuple[float, float] = (0.00, 0.30)
    voxel_size: float = 0.02   # 2 cm

    # ─── BEV cameras ─────────────────────────────────────────────────────────
    # Must be a subset of self.image_features and must have static calibration
    # registered in `get_static_bev_calibration`.
    bev_cam_keys: Tuple[str, ...] = (
        "observation.images.top",
        "observation.images.front",
    )

    # ─── BEV encoder ─────────────────────────────────────────────────────────
    bev_backbone_out_channels: int = 256   # ResNet18 stage-4 default channels
    bev_feature_dim: int = 128             # per-voxel embedding dim (after 1x1)
    bev_3d_conv_layers: int = 1            # extra 3D conv refinement blocks
    bev_pool_mode: str = "mean"            # "mean" | "max" across cameras

    # ─── Cube-heatmap auxiliary head ─────────────────────────────────────────
    aux_enable: bool = True
    aux_weight: float = 0.5                # λ in L_total = L_action + λ · L_aux
    aux_heatmap_sigma_m: float = 0.03      # Gaussian σ in metres on the BEV plane
    aux_cube_key: str = "observation.cube_pose"  # batch key for the (B,3) target

    # ─── Wrist-camera side-channel (phase 1: keep raw, fed via ACT vision branch) ──
    # If True, wrist image is fed through the original per-camera ResNet branch
    # in addition to the BEV path. Phase 1: True (gives ACT something useful
    # during the close-grasp moment when the static BEV cams can't see the cube).
    keep_wrist_raw_branch: bool = True
    wrist_cam_key: str = "observation.images.wrist"

    # ─── BEV centroid → state injection ──────────────────────────────────────
    # When True, the soft-argmax of the aux cube-heatmap is computed inside
    # the model and concatenated to observation.state before projection into
    # the encoder's state token. This bypasses the spatial-attention
    # bottleneck — the transformer reads the BEV-derived cube XY *directly*
    # like a perceived "oracle". Concretely, state goes 16 -> 16+2 = 18 dims
    # before the state input projection.
    inject_bev_centroid_in_state: bool = True

    def __post_init__(self):
        super().__post_init__()
        if self.bev_pool_mode not in {"mean", "max"}:
            raise ValueError(f"bev_pool_mode must be mean|max, got {self.bev_pool_mode}")
        if self.aux_weight < 0:
            raise ValueError(f"aux_weight must be >= 0")
        if self.voxel_size <= 0:
            raise ValueError(f"voxel_size must be > 0")
