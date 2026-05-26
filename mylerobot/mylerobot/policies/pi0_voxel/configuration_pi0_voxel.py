"""PI0VoxelConfig — extends lerobot's PI0Config with voxel-fusion params."""

from dataclasses import dataclass, field

from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.pi0.configuration_pi0 import PI0Config


@PreTrainedConfig.register_subclass("pi0_voxel")
@dataclass
class PI0VoxelConfig(PI0Config):
    """π0 with an added 3D voxel-fusion perception module.

    The voxel module consumes multi-camera RGB features + per-camera extrinsics
    (world-to-cam, OpenCV convention) + intrinsics, builds a 3D voxel grid via
    cross-attention (mylerobot.policies.vision_fusion.voxel_xattn), and injects
    the resulting tokens into PaliGemma's prefix sequence alongside the standard
    image + language tokens.

    Two variants are intended (selected by `voxel_arch`):
      - "cross_attn"  — VoxelCrossAttnFusion (BEVDet-port, already written + tested)
      - "unet"        — Tesla-style UNet 2D→3D + depth-back-projection occupancy (TODO)

    The extrinsics / intrinsics are expected in the batch under
    `observation.cam_extrinsics_world2cam_cv.<cam>` and
    `observation.cam_intrinsics_K.<cam>` keys (this is the convention produced
    by our `mylerobot.sim.libero_multicam.LiberoMultiCamCaptureEnv` and by the
    multicam rerender pipeline).
    """

    # ------------------------------------------------------------------
    # Voxel grid geometry (workspace bounds in world frame, metres)
    # ------------------------------------------------------------------
    # Defaults sized for robosuite Panda tabletop (table at z~0.8m,
    # workspace centred ~ (0, 0, 1.0)). Override per dataset/robot.
    workspace_bounds: tuple[tuple[float, float, float], tuple[float, float, float]] = field(
        default_factory=lambda: ((-0.4, -0.4, 0.7), (0.4, 0.4, 1.3))
    )
    voxel_resolution: tuple[int, int, int] = (8, 16, 16)   # (Hz, Hy, Hx)
    voxel_out_tokens: int = 64                              # after 3D-conv pooling

    # ------------------------------------------------------------------
    # Voxel-cross-attention hyperparams
    # ------------------------------------------------------------------
    voxel_arch: str = "cross_attn"  # one of {"cross_attn", "unet"}; validated in __post_init__
    voxel_num_heads: int = 4
    voxel_num_kv_groups: int = 1                            # 1 = MHA, >1 = GQA
    voxel_attn_chunk: int = 4096
    voxel_dropout: float = 0.0

    # ------------------------------------------------------------------
    # Multi-camera config
    # ------------------------------------------------------------------
    # Which keys in the batch to use as fusion inputs. If a key is missing
    # from a particular batch (e.g. some cameras not in dataset), it's
    # silently skipped — voxel module masks the contribution.
    voxel_camera_keys: tuple[str, ...] = (
        "observation.images.front",
        "observation.images.wrist",
    )
    # The feature backbone used to extract 2D features from each camera image
    # before voxel fusion. By default we reuse PaliGemma's SigLIP outputs
    # (cheap — already computed for the standard image-token path). Set to
    # "separate" to use an independent ResNet/SigLIP backbone for voxel inputs.
    voxel_feat_source: str = "paligemma_siglip"  # one of {"paligemma_siglip", "separate"}

    # ------------------------------------------------------------------
    # Tesla-style occupancy aux head (only used when voxel_arch="unet")
    # ------------------------------------------------------------------
    use_occupancy_aux: bool = False
    occupancy_aux_weight: float = 0.1
    occupancy_grid_resolution: tuple[int, int, int] = (32, 64, 64)  # finer than voxel

    # ------------------------------------------------------------------
    # When True (default), allow batches to omit the extrinsics keys and
    # synthesize identity placeholders — useful for ACT-style training
    # without voxel signal, or for loading old PI0 checkpoints with strict=False.
    # ------------------------------------------------------------------
    extrinsics_optional: bool = True

    def __post_init__(self):
        super().__post_init__()
        if self.voxel_arch not in {"cross_attn", "unet"}:
            raise ValueError(f"voxel_arch must be 'cross_attn' or 'unet', got {self.voxel_arch}")
        if self.use_occupancy_aux and self.voxel_arch != "unet":
            raise ValueError("occupancy aux head only supported with voxel_arch='unet'")
        (xmin, ymin, zmin), (xmax, ymax, zmax) = self.workspace_bounds
        if not (xmax > xmin and ymax > ymin and zmax > zmin):
            raise ValueError(f"workspace_bounds must be (min) < (max) on every axis: {self.workspace_bounds}")
        if any(d <= 0 for d in self.voxel_resolution):
            raise ValueError(f"voxel_resolution must be all positive: {self.voxel_resolution}")
        if self.voxel_out_tokens <= 0:
            raise ValueError(f"voxel_out_tokens must be positive: {self.voxel_out_tokens}")
