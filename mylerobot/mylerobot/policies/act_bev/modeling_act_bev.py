"""ACT-BEV: ACT spine + calibrated multi-view BEV encoder + cube-heatmap aux head.

Why we fork ACT.forward instead of subclassing more lightly:
  - Upstream's vision branch hard-codes "iterate config.image_features and run
    each through self.backbone". We need to (a) bypass that for the BEV cameras
    and replace them with one BEV-derived feature map, (b) optionally keep ONE
    of the cameras (wrist) on the original per-camera path. Easiest is to fork
    the forward (~80 lines) and inject the BEV branch where vision tokens
    are built.

Public class is `ACTBEVPolicy`. It is registered via the config's
`@PreTrainedConfig.register_subclass("act_bev")` decorator and surfaced through
`mylerobot/__init__.py`.
"""

from __future__ import annotations

import einops
import torch
import torch.nn as nn
from torch import Tensor

from lerobot.policies.act.modeling_act import ACT, ACTPolicy
from lerobot.utils.constants import ACTION, OBS_ENV_STATE, OBS_IMAGES, OBS_STATE

from mylerobot.policies.act_bev.aux_head import (
    CubeHeatmapHead,
    build_target_heatmap,
    cube_heatmap_loss,
)
from mylerobot.policies.act_bev.bev_encoder import LiftSplatBEV
from mylerobot.policies.act_bev.configuration_act_bev import (
    ACTBEVConfig,
    get_static_bev_calibration,
)


class ACTBEV(ACT):
    """ACT backbone with vision tokens produced by a BEV/lift-splat module.

    Re-uses the upstream `self.encoder_img_feat_input_proj` / pos embed scheme
    by treating the BEV feature map as a single (B, C, H, W) "camera-feature
    map" with H=X, W=Y. The 1x1 input projection that ACT applies to each
    camera's CNN feature also fits the BEV path — keeping the downstream
    transformer 100% unchanged.

    The wrist camera (if `keep_wrist_raw_branch=True`) still goes through the
    standard per-camera ResNet branch — giving the policy a close-up view at
    the grasp moment when top/front cams have very little angular resolution.
    """

    def __init__(self, config: ACTBEVConfig):
        super().__init__(config)
        self.bev_config = config

        # Resolve BEV cameras and persist static calibration as buffers.
        Hp, Wp = config.image_features[config.bev_cam_keys[0]].shape[1:]
        K_np, E_np = get_static_bev_calibration(
            tuple(config.bev_cam_keys), policy_hw=(Hp, Wp)
        )
        self.register_buffer(
            "bev_K", torch.from_numpy(K_np).float(), persistent=False
        )  # (N_cam, 3, 3)
        self.register_buffer(
            "bev_E", torch.from_numpy(E_np).float(), persistent=False
        )  # (N_cam, 4, 4)

        # LiftSplat module
        self.bev_encoder = LiftSplatBEV(
            n_cam=len(config.bev_cam_keys),
            voxel_x_range=config.voxel_x_range,
            voxel_y_range=config.voxel_y_range,
            voxel_z_range=config.voxel_z_range,
            voxel_size=config.voxel_size,
            feature_dim=config.bev_feature_dim,
            backbone_out_channels=config.bev_backbone_out_channels,
            n_3d_refine_layers=config.bev_3d_conv_layers,
            pool_mode=config.bev_pool_mode,
            image_hw=(Hp, Wp),
        )

        # The upstream encoder_img_feat_input_proj is built for a per-camera
        # backbone with 512 channels (resnet18 layer4). Our BEV feature has
        # `bev_feature_dim` channels, so we need a *separate* projection.
        # ACT's `encoder_cam_feat_pos_embed` is a SinusoidalPositionEmbedding2d
        # over (B, C, H, W); it doesn't depend on C, so we can reuse it.
        self.bev_input_proj = nn.Conv2d(
            config.bev_feature_dim, config.dim_model, kernel_size=1
        )

        # Auxiliary cube-heatmap head
        if config.aux_enable:
            self.aux_head = CubeHeatmapHead(config.bev_feature_dim)
        else:
            self.aux_head = None

        # BEV centroid → state injection. Replace the inherited state-input
        # projection with one that accepts 2 extra dims (predicted cube xy).
        if config.inject_bev_centroid_in_state and config.robot_state_feature:
            state_dim_in = config.robot_state_feature.shape[0]
            self.encoder_robot_state_input_proj = nn.Linear(
                state_dim_in + 2, config.dim_model
            )
            # Same for the VAE branch (which also reads observation.state).
            if config.use_vae:
                self.vae_encoder_robot_state_input_proj = nn.Linear(
                    state_dim_in + 2, config.dim_model
                )
            # Precompute the BEV grid's world-XY coordinates as a buffer,
            # used by soft-argmax for centroid extraction.
            x_lo, x_hi = config.voxel_x_range
            y_lo, y_hi = config.voxel_y_range
            nx = max(1, int(round((x_hi - x_lo) / config.voxel_size)))
            ny = max(1, int(round((y_hi - y_lo) / config.voxel_size)))
            xs = torch.linspace(x_lo + config.voxel_size / 2,
                                x_hi - config.voxel_size / 2, nx)
            ys = torch.linspace(y_lo + config.voxel_size / 2,
                                y_hi - config.voxel_size / 2, ny)
            gx, gy = torch.meshgrid(xs, ys, indexing="ij")
            # (X, Y) world coordinates for each BEV pixel.
            self.register_buffer("bev_x_coords", gx, persistent=False)
            self.register_buffer("bev_y_coords", gy, persistent=False)
            # Z-score normalization for the injected centroid so it has the
            # same scale as the (normalized) proprio state. Stats are hard-
            # coded from the n=200 Lift dataset; they're stable across re-
            # collections of the same env.
            self.register_buffer(
                "centroid_mean", torch.tensor([0.4972, -0.0106], dtype=torch.float32), persistent=False
            )
            self.register_buffer(
                "centroid_std", torch.tensor([0.0597, 0.1333], dtype=torch.float32), persistent=False
            )

    # ─── Helpers ─────────────────────────────────────────────────────────────
    def _bev_centroid_softargmax(self, bev_feat: Tensor) -> Tensor | None:
        """Compute the predicted cube (x, y) world coordinates via soft-argmax
        over the aux head's heatmap.

        Returns (B, 2) world coordinates, or None if aux_head is disabled.
        """
        if self.aux_head is None:
            return None
        logits = self.aux_head(bev_feat)             # (B, X, Y)
        B, X, Y = logits.shape
        # Softmax over the flattened spatial grid (per-batch independently).
        probs = logits.reshape(B, -1).softmax(dim=1).reshape(B, X, Y)
        x = (probs * self.bev_x_coords.to(probs.dtype)).sum(dim=(1, 2))
        y = (probs * self.bev_y_coords.to(probs.dtype)).sum(dim=(1, 2))
        return torch.stack([x, y], dim=-1)            # (B, 2)

    def _augment_state(self, batch: dict[str, Tensor], bev_feat: Tensor) -> Tensor:
        """Return the (possibly centroid-augmented) observation.state tensor."""
        state = batch[OBS_STATE]
        if self.bev_config.inject_bev_centroid_in_state and self.aux_head is not None:
            centroid = self._bev_centroid_softargmax(bev_feat)  # (B, 2)
            # Z-score to match the magnitude of the (already-normalized) proprio.
            centroid_norm = (centroid - self.centroid_mean) / self.centroid_std
            state = torch.cat([state, centroid_norm.to(state.dtype)], dim=-1)
        return state

    def _bev_tokens(
        self, batch: dict[str, Tensor]
    ) -> tuple[Tensor, Tensor, Tensor | None]:
        """Return (vis_tokens, vis_pos, bev_feat).

        `vis_tokens`  shape (S, B, D_model)
        `vis_pos`     shape (S, B, D_model)
        `bev_feat`    shape (B, Cv, X, Y) — kept for the aux head.
        """
        cfg = self.bev_config

        # Stack BEV cam images: (B, N_cam, 3, H, W)
        imgs = torch.stack([batch[k] for k in cfg.bev_cam_keys], dim=1)
        B, N, C, H, W = imgs.shape
        device = imgs.device

        # Broadcast static calibration across the batch.
        K = self.bev_K.unsqueeze(0).expand(B, -1, -1, -1).to(device)
        E = self.bev_E.unsqueeze(0).expand(B, -1, -1, -1).to(device)

        bev_out = self.bev_encoder(imgs, K, E)
        bev_feat = bev_out["bev"]                              # (B, Cv, X, Y)

        # Project to D_model and build position embedding via upstream module.
        proj = self.bev_input_proj(bev_feat)                   # (B, D, X, Y)
        pos = self.encoder_cam_feat_pos_embed(proj).to(dtype=proj.dtype)

        proj = einops.rearrange(proj, "b c h w -> (h w) b c")
        pos = einops.rearrange(pos, "b c h w -> (h w) b c")
        return proj, pos, bev_feat

    def _wrist_tokens(self, batch: dict[str, Tensor]) -> tuple[Tensor, Tensor] | None:
        """Standard ACT per-camera path for the wrist camera (kept as a
        close-up side-channel)."""
        cfg = self.bev_config
        if not cfg.keep_wrist_raw_branch or cfg.wrist_cam_key not in batch:
            return None
        img = batch[cfg.wrist_cam_key]
        cam_features = self.backbone(img)["feature_map"]  # (B, C_b, h, w)
        cam_pos = self.encoder_cam_feat_pos_embed(cam_features).to(cam_features.dtype)
        cam_features = self.encoder_img_feat_input_proj(cam_features)  # (B, D, h, w)
        cam_features = einops.rearrange(cam_features, "b c h w -> (h w) b c")
        cam_pos = einops.rearrange(cam_pos, "b c h w -> (h w) b c")
        return cam_features, cam_pos

    # ─── Forward (fork of ACT.forward) ───────────────────────────────────────
    def forward(self, batch: dict[str, Tensor]) -> tuple[Tensor, dict, Tensor | None]:
        """Returns (actions, (mu, log_sigma_x2), bev_feat).
        bev_feat is exposed so ACTBEVPolicy.forward can compute the aux loss
        without re-running the encoder.
        """
        if self.config.use_vae and self.training:
            assert ACTION in batch

        # Determine batch size from one of the BEV cam images
        first_cam_key = self.bev_config.bev_cam_keys[0]
        batch_size = batch[first_cam_key].shape[0]

        # ── Build BEV first so we can derive the centroid for state injection ──
        bev_tokens, bev_pos, bev_feat = self._bev_tokens(batch)
        augmented_state = self._augment_state(batch, bev_feat)

        # ── VAE encoder branch (unchanged from upstream) ────────────────────
        if self.config.use_vae and ACTION in batch and self.training:
            cls_embed = einops.repeat(
                self.vae_encoder_cls_embed.weight, "1 d -> b 1 d", b=batch_size
            )
            if self.config.robot_state_feature:
                robot_state_embed = self.vae_encoder_robot_state_input_proj(
                    augmented_state
                ).unsqueeze(1)
            action_embed = self.vae_encoder_action_input_proj(batch[ACTION])

            vae_in = (
                [cls_embed, robot_state_embed, action_embed]
                if self.config.robot_state_feature
                else [cls_embed, action_embed]
            )
            vae_encoder_input = torch.cat(vae_in, axis=1)
            pos_embed = self.vae_encoder_pos_enc.clone().detach()
            cls_joint_is_pad = torch.full(
                (batch_size, 2 if self.config.robot_state_feature else 1),
                False,
                device=batch[OBS_STATE].device,
            )
            key_padding_mask = torch.cat([cls_joint_is_pad, batch["action_is_pad"]], axis=1)
            cls_token_out = self.vae_encoder(
                vae_encoder_input.permute(1, 0, 2),
                pos_embed=pos_embed.permute(1, 0, 2),
                key_padding_mask=key_padding_mask,
            )[0]
            latent_pdf_params = self.vae_encoder_latent_output_proj(cls_token_out)
            mu = latent_pdf_params[:, : self.config.latent_dim]
            log_sigma_x2 = latent_pdf_params[:, self.config.latent_dim :]
            latent_sample = mu + log_sigma_x2.div(2).exp() * torch.randn_like(mu)
        else:
            mu = log_sigma_x2 = None
            ref = batch[OBS_STATE] if self.config.robot_state_feature else batch[first_cam_key]
            latent_sample = torch.zeros(
                [batch_size, self.config.latent_dim], dtype=torch.float32, device=ref.device
            )

        # ── 1-D encoder tokens (latent + state) ─────────────────────────────
        encoder_in_tokens = [self.encoder_latent_input_proj(latent_sample)]
        encoder_in_pos_embed = list(self.encoder_1d_feature_pos_embed.weight.unsqueeze(1))
        if self.config.robot_state_feature:
            encoder_in_tokens.append(self.encoder_robot_state_input_proj(augmented_state))
        if self.config.env_state_feature:
            encoder_in_tokens.append(self.encoder_env_state_input_proj(batch[OBS_ENV_STATE]))

        token_1d = torch.stack(encoder_in_tokens, axis=0)             # (T_1d, B, D)
        pos_1d = torch.stack(encoder_in_pos_embed, axis=0)            # (T_1d, 1, D)

        # ── Vision tokens: BEV (already computed) + optional wrist raw ──────
        vis_tokens_seq: list[Tensor] = []
        vis_pos_seq: list[Tensor] = []

        vis_tokens_seq.append(bev_tokens)
        vis_pos_seq.append(bev_pos)

        wrist = self._wrist_tokens(batch)
        if wrist is not None:
            vis_tokens_seq.append(wrist[0])
            vis_pos_seq.append(wrist[1])

        vis_tokens = torch.cat(vis_tokens_seq, dim=0)
        vis_pos = torch.cat(vis_pos_seq, dim=0)
        all_tokens = torch.cat([token_1d, vis_tokens], dim=0)
        all_pos = torch.cat([pos_1d, vis_pos], dim=0)

        # ── Transformer encoder/decoder (unchanged) ─────────────────────────
        encoder_out = self.encoder(all_tokens, pos_embed=all_pos)
        decoder_in = torch.zeros(
            (self.config.chunk_size, batch_size, self.config.dim_model),
            dtype=all_pos.dtype,
            device=all_pos.device,
        )
        decoder_out = self.decoder(
            decoder_in,
            encoder_out,
            encoder_pos_embed=all_pos,
            decoder_pos_embed=self.decoder_pos_embed.weight.unsqueeze(1),
        )
        decoder_out = decoder_out.transpose(0, 1)
        actions = self.action_head(decoder_out)
        return actions, (mu, log_sigma_x2), bev_feat


class ACTBEVPolicy(ACTPolicy):
    """ACTPolicy variant that uses the ACT-BEV model and adds an auxiliary
    cube-heatmap loss."""

    config_class = ACTBEVConfig
    name = "act_bev"

    def __init__(self, config: ACTBEVConfig, **kwargs):
        from lerobot.policies.pretrained import PreTrainedPolicy

        # Strip cube_pose from input_features so LeRobot's normalizer doesn't
        # touch it (we want it as raw world-frame supervision for the aux head).
        if config.aux_cube_key in config.input_features:
            new_inputs = {
                k: v for k, v in config.input_features.items()
                if k != config.aux_cube_key
            }
            config.input_features = new_inputs

        PreTrainedPolicy.__init__(self, config)
        config.validate_features()
        self.config = config
        self.model = ACTBEV(config)
        if config.temporal_ensemble_coeff is not None:
            from lerobot.policies.act.modeling_act import ACTTemporalEnsembler

            self.temporal_ensembler = ACTTemporalEnsembler(
                config.temporal_ensemble_coeff, config.chunk_size
            )
        self.reset()

    # `predict_action_chunk` and `select_action` inherited from ACTPolicy
    # — they call self.model(batch) and expect a tuple. We return 3-tuple, so
    # we need to override the action extraction here too.
    def predict_action_chunk(self, batch: dict[str, Tensor]) -> Tensor:
        self.eval()
        # Skip the upstream OBS_IMAGES list construction; our model reads
        # individual image keys directly from batch.
        actions = self.model(batch)[0]
        return actions

    def forward(self, batch: dict[str, Tensor]) -> tuple[Tensor, dict]:
        actions_hat, (mu_hat, log_sigma_x2_hat), bev_feat = self.model(batch)

        import torch.nn.functional as F

        l1_loss = (
            F.l1_loss(batch[ACTION], actions_hat, reduction="none")
            * ~batch["action_is_pad"].unsqueeze(-1)
        ).mean()
        loss_dict: dict[str, float] = {"l1_loss": l1_loss.item()}
        total = l1_loss

        if self.config.use_vae:
            mean_kld = (
                (
                    -0.5
                    * (1 + log_sigma_x2_hat - mu_hat.pow(2) - (log_sigma_x2_hat).exp())
                )
                .sum(-1)
                .mean()
            )
            loss_dict["kld_loss"] = mean_kld.item()
            total = total + mean_kld * self.config.kl_weight

        # ── Aux loss: cube heatmap on BEV ───────────────────────────────────
        if self.config.aux_enable and self.model.aux_head is not None:
            cube_key = self.config.aux_cube_key
            if cube_key in batch:
                cube_pose = batch[cube_key].to(dtype=bev_feat.dtype)
                tgt = build_target_heatmap(
                    cube_pose,
                    self.config.voxel_x_range,
                    self.config.voxel_y_range,
                    self.config.voxel_size,
                    self.config.aux_heatmap_sigma_m,
                )
                pred = self.model.aux_head(bev_feat)            # (B, X, Y)
                aux = cube_heatmap_loss(pred, tgt)
                loss_dict["aux_cube_heatmap_loss"] = aux.item()
                total = total + self.config.aux_weight * aux

        return total, loss_dict
