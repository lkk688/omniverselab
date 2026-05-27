"""PI0VoxelPolicy / PI0VoxelPytorch — voxel-augmented π0 subclass.

Wires `mylerobot.policies.vision_fusion.voxel_xattn.VoxelCrossAttnFusion` into
lerobot's PI0 by overriding `embed_prefix` to append voxel tokens to the
[image-tokens..., language-tokens] prefix sequence that PaliGemma consumes.

Design notes
============
- Subclass `PI0Policy` (the wrapper) → swap inner `PI0Pytorch` for
  `PI0VoxelPytorch` (our subclass with voxel module).
- Subclass `PI0Pytorch` → override `embed_prefix` to inject voxel tokens
  between image and language tokens. Keep upstream `forward` and
  `sample_actions` intact (they call our overridden `embed_prefix`).
- For extrinsics + intrinsics: by default we derive from `observation.state`
  via `extrinsics_libero.derive_libero_extrinsics_batch` (works for the
  Sylvest/libero_plus_lerobot dataset with no extra columns). Datasets that
  *do* ship extrinsics (e.g. our `local/libero_multicam_v0`) can pass them
  through the batch under `observation.cam_extrinsics_world2cam_cv.<cam>`
  and `observation.cam_intrinsics_K.<cam>` — `_get_extrinsics_from_batch`
  prefers those when present.
- Voxel input image features: by default we use the SigLIP outputs that
  PaliGemma already computed for the standard image-token path. This is
  cheap (we just intercept the per-camera image embedding from the loop in
  the parent `embed_prefix`). When `config.voxel_feat_source="separate"`,
  we'd instantiate a separate visual backbone (TODO; not needed for v1).
"""

from __future__ import annotations

from typing import Sequence

import torch
from torch import Tensor

from lerobot.policies.pi0.modeling_pi0 import PI0Policy, PI0Pytorch

from mylerobot.policies.pi0_voxel.configuration_pi0_voxel import PI0VoxelConfig
from mylerobot.policies.pi0_voxel.extrinsics_libero import (
    AGENTVIEW_K_256,
    WRIST_K_256,
    derive_libero_extrinsics_batch,
)
from mylerobot.policies.vision_fusion.voxel_xattn import VoxelCrossAttnFusion


class PI0VoxelPytorch(PI0Pytorch):
    """PI0Pytorch + voxel-fusion module."""

    def __init__(self, config: PI0VoxelConfig, rtc_processor=None):
        super().__init__(config, rtc_processor=rtc_processor)
        self._voxel_cfg = config

        # Read PaliGemma + SigLIP dims directly from the loaded vision tower
        # (so we always agree with the actual model even across variants).
        vt = self.paligemma_with_expert.paligemma.model.vision_tower
        siglip_feat_channels = vt.config.hidden_size
        self._siglip_image_size = vt.config.image_size
        self._siglip_patch_size = vt.config.patch_size
        text_cfg = self.paligemma_with_expert.paligemma.config.text_config
        paligemma_hidden = text_cfg.hidden_size

        self.voxel_fusion = VoxelCrossAttnFusion(
            feat_channels=siglip_feat_channels,
            out_channels=paligemma_hidden,
            voxel_resolution=config.voxel_resolution,
            workspace_bounds=config.workspace_bounds,
            num_heads=config.voxel_num_heads,
            num_kv_groups=config.voxel_num_kv_groups,
            attn_chunk=config.voxel_attn_chunk,
            out_tokens=config.voxel_out_tokens,
            dropout=config.voxel_dropout,
        )

        # Road B-5: auxiliary state-prediction head on pooled voxel tokens.
        # Predicts EEF position (3D) from voxel features to force the voxel
        # module to actually encode geometric content. Only built when
        # `aux_state_pred_weight > 0` to avoid changing parameter count
        # / state_dict structure when disabled.
        if getattr(config, "aux_state_pred_weight", 0.0) > 0.0:
            self.aux_state_head = torch.nn.Sequential(
                torch.nn.Linear(paligemma_hidden, paligemma_hidden // 4),
                torch.nn.ReLU(inplace=True),
                torch.nn.Linear(paligemma_hidden // 4, 3),  # predict eef_pos (x, y, z)
            )
        else:
            self.aux_state_head = None

    def _siglip_spatial_features(self, image_bchw: Tensor) -> Tensor:
        """Run a single (B, 3, 224, 224) image through SigLIP and return
        spatial features (B, C=1152, Hf, Wf) suitable for voxel cross-attn."""
        vt = self.paligemma_with_expert.paligemma.model.vision_tower
        img_size = self._siglip_image_size
        if image_bchw.shape[-2] != img_size or image_bchw.shape[-1] != img_size:
            image_bchw = torch.nn.functional.interpolate(
                image_bchw, size=(img_size, img_size), mode="bilinear", align_corners=False
            )
        in_dtype = image_bchw.dtype
        if image_bchw.dtype != torch.float32:
            image_bchw = image_bchw.to(torch.float32)
        out = vt(image_bchw)
        feats = out.last_hidden_state  # (B, N, C)
        B, N, C = feats.shape
        side = int(N ** 0.5)
        assert side * side == N, f"Non-square SigLIP token grid: N={N}"
        feats = feats.transpose(1, 2).reshape(B, C, side, side)
        if feats.dtype != in_dtype:
            feats = feats.to(in_dtype)
        return feats

    def build_voxel_tokens(
        self,
        cam_image_feats: Sequence[Tensor],
        intrinsics: Tensor,
        extrinsics: Tensor,
        cam_mask: Tensor | None = None,
        workspace_translate: Tensor | None = None,
    ) -> tuple[Tensor, Tensor, list]:
        tokens = self.voxel_fusion(
            cam_image_feats, intrinsics, extrinsics, cam_mask=cam_mask,
            workspace_translate=workspace_translate,
        )
        B, N, _ = tokens.shape
        pad_mask = torch.ones((B, N), dtype=torch.bool, device=tokens.device)
        ar_mask = [0] * N
        return tokens, pad_mask, ar_mask

    def embed_prefix(
        self, images, img_masks, lang_tokens, lang_masks,
    ) -> tuple[Tensor, Tensor, Tensor]:
        embs, pad_masks, att_masks = super().embed_prefix(
            images, img_masks, lang_tokens, lang_masks,
        )
        aux = getattr(self, "_pending_voxel_aux", None)
        if aux is None:
            return embs, pad_masks, att_masks

        # Compute SigLIP spatial features from the SAME images that the parent
        # just embedded. `aux["cam_indices"]` maps voxel_camera_keys → indices
        # into the `images` list (set by PI0VoxelPolicy._prepare_voxel_aux).
        cam_indices = aux["cam_indices"]
        cam_image_feats = [self._siglip_spatial_features(images[i]) for i in cam_indices]

        voxel_tokens, voxel_pad, voxel_ar = self.build_voxel_tokens(
            cam_image_feats,
            aux["intrinsics"],
            aux["extrinsics"],
            cam_mask=aux.get("cam_mask"),
            workspace_translate=aux.get("workspace_translate"),
        )

        B = embs.shape[0]
        ar_extra = torch.tensor(voxel_ar, dtype=torch.bool, device=att_masks.device)
        ar_extra = ar_extra[None, :].expand(B, len(voxel_ar))

        embs_out = torch.cat([embs, voxel_tokens], dim=1)
        pad_masks_out = torch.cat([pad_masks, voxel_pad], dim=1)
        att_masks_out = torch.cat([att_masks, ar_extra], dim=1)

        # Road B-5: stash voxel tokens so PI0VoxelPolicy.forward can compute
        # the auxiliary state-prediction loss after the action loss is computed.
        # Note: cleared by PI0VoxelPolicy.forward, not here (so multiple
        # samples within a step's diffusion loop all see the same voxel tokens).
        if self.aux_state_head is not None:
            self._last_voxel_tokens = voxel_tokens  # (B, N_voxel, paligemma_hidden)

        self._pending_voxel_aux = None
        return embs_out, pad_masks_out, att_masks_out


class PI0VoxelPolicy(PI0Policy):
    config_class = PI0VoxelConfig
    name = "pi0_voxel"

    def __init__(self, config: PI0VoxelConfig, **kwargs):
        super().__init__(config, **kwargs)
        self.model = PI0VoxelPytorch(config, rtc_processor=self.rtc_processor)
        if config.gradient_checkpointing:
            self.model.gradient_checkpointing_enable()
        self.model.to(config.device)
        # Eval-side ablation hook: set to 'identity' or 'shuffle' to corrupt
        # extrinsics before voxel projection (see _prepare_voxel_aux).
        self._extrinsics_corruption: str | None = None

    def _prepare_voxel_aux(self, batch: dict[str, Tensor]) -> None:
        cfg = self.config

        # Build a deterministic ordering of all visual input keys — must match
        # the order PI0Policy uses when assembling the `images` list passed to
        # embed_prefix. That ordering is dict-insertion order of input_features
        # (Python 3.7+) restricted to VISUAL features.
        from lerobot.configs.types import FeatureType
        visual_keys = [
            k for k, f in cfg.input_features.items() if f.type == FeatureType.VISUAL
        ]
        # Map voxel_camera_keys → indices into the `images` list seen by embed_prefix
        cam_indices: list[int] = []
        cam_mask_list: list[bool] = []
        for cam_key in cfg.voxel_camera_keys:
            if cam_key in visual_keys and cam_key in batch:
                cam_indices.append(visual_keys.index(cam_key))
                cam_mask_list.append(True)
            else:
                cam_mask_list.append(False)
        if not cam_indices:
            if cfg.extrinsics_optional:
                return
            raise KeyError(
                f"No voxel camera images in batch (expected any of {cfg.voxel_camera_keys})"
            )

        any_cam_key = next(k for k in cfg.voxel_camera_keys if k in batch)
        B = batch[any_cam_key].shape[0]
        device = batch[any_cam_key].device

        extr_per_cam: list[Tensor] = []
        intr_per_cam: list[Tensor] = []
        missing_extr = False
        for cam_key in cfg.voxel_camera_keys:
            short_cam = cam_key.rsplit(".", 1)[-1]
            ek = f"observation.cam_extrinsics_world2cam_cv.{short_cam}"
            ik = f"observation.cam_intrinsics_K.{short_cam}"
            if ek in batch and ik in batch:
                extr_per_cam.append(batch[ek])
                intr_per_cam.append(batch[ik])
            else:
                missing_extr = True
                break

        if missing_extr:
            state = batch.get("observation.state")
            if state is None:
                if cfg.extrinsics_optional:
                    return
                raise KeyError(
                    "No extrinsics keys in batch AND no observation.state to derive from"
                )
            extr_map = derive_libero_extrinsics_batch(state)
            extr_per_cam, intr_per_cam = [], []
            for cam_key in cfg.voxel_camera_keys:
                short = cam_key.rsplit(".", 1)[-1]
                # Aliases: lerobot/libero uses ".image"/".image2"; Sylvest uses
                # ".front"/".wrist"; LiberoMultiCam uses ".agentview"/".robot0_eye_in_hand".
                if short in ("front", "agentview", "image"):
                    extr_per_cam.append(extr_map["agentview"])
                    K = torch.from_numpy(AGENTVIEW_K_256).float().to(device)
                elif short in ("wrist", "robot0_eye_in_hand", "image2"):
                    extr_per_cam.append(extr_map["wrist"])
                    K = torch.from_numpy(WRIST_K_256).float().to(device)
                else:
                    continue
                intr_per_cam.append(K.unsqueeze(0).expand(B, 3, 3))

        if not extr_per_cam:
            if cfg.extrinsics_optional:
                return
            raise RuntimeError("Could not assemble voxel extrinsics for any camera")

        extrinsics = torch.stack(extr_per_cam, dim=1).to(device)
        intrinsics = torch.stack(intr_per_cam, dim=1).to(device)
        cam_mask = torch.tensor(
            [cam_mask_list[:len(extr_per_cam)]] * B,
            dtype=torch.bool,
            device=device,
        )

        # Eval-side ablation hook: corrupt extrinsics to isolate whether the
        # voxel module actually uses geometry. Set via
        # `policy._extrinsics_corruption = 'identity' | 'shuffle' | None`.
        # - 'identity': replace world->cam with identity; cameras pretend to sit
        #   at world origin looking down +Z. Removes ALL geometric info.
        # - 'shuffle': randomly permute the per-camera extrinsics so the wrong
        #   image features get attached to each projected voxel. Keeps the
        #   distribution but destroys the correspondence.
        corruption = getattr(self, "_extrinsics_corruption", None)
        if corruption == "identity":
            eye = torch.eye(4, device=device, dtype=extrinsics.dtype)
            extrinsics = eye.view(1, 1, 4, 4).expand_as(extrinsics).contiguous()
        elif corruption == "shuffle":
            B_e, Ncam = extrinsics.shape[:2]
            perm = torch.randperm(Ncam, device=device)
            extrinsics = extrinsics[:, perm].contiguous()
        elif corruption is not None:
            raise ValueError(
                f"Unknown _extrinsics_corruption: {corruption!r} "
                "(expected None, 'identity', or 'shuffle')"
            )

        # Optional per-batch workspace re-centring: voxel grid follows the
        # gripper instead of being pinned to a fixed Panda tabletop centre.
        workspace_translate = None
        if getattr(cfg, "workspace_centered_on_eef", False):
            state = batch.get("observation.state")
            if state is not None and state.shape[-1] >= 3:
                eef_pos = state[..., :3].to(device=device, dtype=torch.float32)
                canon = torch.tensor(
                    cfg.canonical_eef_center, device=device, dtype=torch.float32
                )
                workspace_translate = eef_pos - canon

        self.model._pending_voxel_aux = {
            "cam_indices": cam_indices,
            "extrinsics": extrinsics,
            "intrinsics": intrinsics,
            "cam_mask": cam_mask,
            "workspace_translate": workspace_translate,
        }

    def predict_action_chunk(self, batch, **kwargs):
        self._prepare_voxel_aux(batch)
        return super().predict_action_chunk(batch, **kwargs)

    def forward(self, batch, reduction: str = "mean"):
        self._prepare_voxel_aux(batch)
        # Clear any stale voxel-token stash before the forward (so the aux
        # head only sees this call's tokens, not the last call's).
        if getattr(self.model, "aux_state_head", None) is not None:
            self.model._last_voxel_tokens = None

        loss, out = super().forward(batch, reduction=reduction)

        # Road B-5: auxiliary state-prediction loss.
        aux_w = float(getattr(self.config, "aux_state_pred_weight", 0.0))
        if aux_w > 0.0 and self.model.aux_state_head is not None:
            voxel_tokens = getattr(self.model, "_last_voxel_tokens", None)
            state = batch.get("observation.state")
            if voxel_tokens is not None and state is not None and state.shape[-1] >= 3:
                # Pool: mean over voxel-token dim (B, N, C) -> (B, C)
                pool = self.config.aux_state_pool
                if pool == "mean":
                    pooled = voxel_tokens.mean(dim=1)
                else:
                    raise NotImplementedError(f"aux_state_pool='{pool}' not implemented")
                # Predict EEF position (3,) in world frame
                pred_eef = self.model.aux_state_head(pooled.to(self.model.aux_state_head[0].weight.dtype))
                true_eef = state[..., :3].to(pred_eef.dtype)
                if pred_eef.shape == true_eef.shape:
                    aux_loss = torch.nn.functional.mse_loss(
                        pred_eef, true_eef, reduction=reduction
                    )
                    loss = loss + aux_w * aux_loss
                    if isinstance(out, dict):
                        out["aux_state_loss"] = aux_loss.detach()
                        out["aux_state_l1"] = (pred_eef.detach() - true_eef.detach()).abs().mean()
                # Clear stash after use
                self.model._last_voxel_tokens = None

        return loss, out


# Backwards-compat shims for older callers / tests that used the lazy factory
# pattern. They now just return the eager classes above.
def _import_pi0_classes():
    return PI0Policy, PI0Pytorch


def _make_pi0_voxel_pytorch_class():
    return PI0VoxelPytorch


def _make_pi0_voxel_policy_class():
    return PI0VoxelPolicy
