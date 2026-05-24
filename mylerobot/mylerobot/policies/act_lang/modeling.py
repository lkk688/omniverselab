"""Language-conditioned ACT.

Subclasses `lerobot.policies.act.modeling_act.{ACT, ACTPolicy}` and injects a
language side-channel via one of three fusion modes (concat / FiLM / xattn).

Design constraints we intentionally honour:
- Do not touch `ACTEncoder` / `ACTDecoder` / `ACTEncoderLayer` — those are the
  upstream-volatile pieces. We only add tokens / modulate features at the
  *boundary* between vision backbone and transformer encoder.
- Loading a vanilla ACT checkpoint must still work (with `strict=False`); the
  new fusion params are initialized fresh.
- A batch missing `language_embedding` falls back to zeros when
  `config.lang_optional=True`. Lets us run the same model on unannotated data
  for ablation.
"""

from __future__ import annotations

import einops
import torch
from torch import Tensor

from lerobot.policies.act.modeling_act import ACT, ACTPolicy
from lerobot.utils.constants import ACTION, OBS_ENV_STATE, OBS_IMAGES, OBS_STATE

from mylerobot.policies.act_lang.configuration import ACTLangConfig
from mylerobot.policies.act_lang.fusion import ConcatFusion, FiLMFusion, XAttnFusion

LANG_EMBED_KEY = "language_embedding"


class ACTLang(ACT):
    """ACT backbone + language fusion module.

    Re-implements `forward()` by copying upstream `ACT.forward` and inserting
    fusion at the right place. This is a deliberate (and isolated) duplication
    of one method, traded against tighter coupling — pinning the upstream commit
    lets us catch divergence with `tests/test_against_upstream.py` later.
    """

    def __init__(self, config: ACTLangConfig):
        super().__init__(config)
        self.lang_config = config
        if config.fusion_mode == "concat":
            self.fusion = ConcatFusion(config.lang_emb_dim, config.dim_model, config.fusion_dropout)
        elif config.fusion_mode == "film":
            # FiLM operates on backbone feature maps (pre-projection), so channels
            # come from the ResNet's final block, not dim_model.
            if not config.image_features:
                raise ValueError("fusion_mode='film' requires image_features")
            feat_channels = self.encoder_img_feat_input_proj.in_channels
            self.fusion = FiLMFusion(config.lang_emb_dim, feat_channels, config.fusion_dropout)
        elif config.fusion_mode == "xattn":
            self.fusion = XAttnFusion(
                lang_emb_dim=config.lang_emb_dim,
                dim_model=config.dim_model,
                n_heads=config.fusion_n_heads,
                dropout=config.fusion_dropout,
            )
        else:  # pragma: no cover  -- enforced in config __post_init__
            raise ValueError(config.fusion_mode)

    def _get_lang_emb(self, batch: dict[str, Tensor], batch_size: int, device, dtype) -> Tensor:
        if LANG_EMBED_KEY in batch:
            return batch[LANG_EMBED_KEY].to(device=device, dtype=dtype)
        if self.lang_config.lang_optional:
            return torch.zeros(
                (batch_size, self.lang_config.lang_emb_dim), device=device, dtype=dtype
            )
        raise KeyError(
            f"Batch missing '{LANG_EMBED_KEY}' and lang_optional=False. "
            f"Either provide it or set lang_optional=True in ACTLangConfig."
        )

    def forward(self, batch: dict[str, Tensor]) -> tuple[Tensor, tuple[Tensor, Tensor] | tuple[None, None]]:
        # NOTE: This is a fork of ACT.forward (lerobot 81948979). Keep diffs minimal;
        #       fusion-specific lines are marked `# >>> mylerobot`.
        if self.config.use_vae and self.training:
            assert ACTION in batch, (
                "actions must be provided when using the variational objective in training mode."
            )

        batch_size = batch[OBS_IMAGES][0].shape[0] if OBS_IMAGES in batch else batch[OBS_ENV_STATE].shape[0]

        # ---- VAE encoder branch (unchanged from upstream) ----
        if self.config.use_vae and ACTION in batch and self.training:
            cls_embed = einops.repeat(self.vae_encoder_cls_embed.weight, "1 d -> b 1 d", b=batch_size)
            if self.config.robot_state_feature:
                robot_state_embed = self.vae_encoder_robot_state_input_proj(batch[OBS_STATE]).unsqueeze(1)
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
            ref = batch[OBS_STATE] if self.config.robot_state_feature else batch[OBS_IMAGES][0]
            latent_sample = torch.zeros(
                [batch_size, self.config.latent_dim], dtype=torch.float32, device=ref.device
            )

        # ---- Build encoder inputs ----
        encoder_in_tokens = [self.encoder_latent_input_proj(latent_sample)]
        encoder_in_pos_embed = list(self.encoder_1d_feature_pos_embed.weight.unsqueeze(1))
        if self.config.robot_state_feature:
            encoder_in_tokens.append(self.encoder_robot_state_input_proj(batch[OBS_STATE]))
        if self.config.env_state_feature:
            encoder_in_tokens.append(self.encoder_env_state_input_proj(batch[OBS_ENV_STATE]))

        device = latent_sample.device
        dtype = latent_sample.dtype
        lang_emb = self._get_lang_emb(batch, batch_size, device=device, dtype=dtype)  # >>> mylerobot

        # ---- Vision tokens + (optional) FiLM modulation ----
        vis_tokens_seq: list[Tensor] = []
        vis_pos_seq: list[Tensor] = []
        if self.config.image_features:
            for img in batch[OBS_IMAGES]:
                cam_features = self.backbone(img)["feature_map"]  # (B, C_b, H, W)
                if self.lang_config.fusion_mode == "film":  # >>> mylerobot
                    cam_features = self.fusion.modulate(cam_features, lang_emb)
                cam_pos_embed = self.encoder_cam_feat_pos_embed(cam_features).to(dtype=cam_features.dtype)
                cam_features = self.encoder_img_feat_input_proj(cam_features)  # (B, dim_model, H, W)
                cam_features = einops.rearrange(cam_features, "b c h w -> (h w) b c")
                cam_pos_embed = einops.rearrange(cam_pos_embed, "b c h w -> (h w) b c")
                vis_tokens_seq.append(cam_features)
                vis_pos_seq.append(cam_pos_embed)

        # encoder_in_tokens so far is a list of (B, D); stack the 1d ones first.
        token_1d = torch.stack(encoder_in_tokens, axis=0)  # (T_1d, B, D)
        pos_1d = torch.stack(encoder_in_pos_embed, axis=0)  # (T_1d, 1, D)

        if vis_tokens_seq:
            vis_tokens = torch.cat(vis_tokens_seq, dim=0)  # (S_v, B, D)
            vis_pos = torch.cat(vis_pos_seq, dim=0)
            if self.lang_config.fusion_mode == "xattn":  # >>> mylerobot
                vis_tokens = self.fusion(vis_tokens, lang_emb)
            all_tokens = torch.cat([token_1d, vis_tokens], dim=0)
            all_pos = torch.cat([pos_1d, vis_pos], dim=0)
        else:
            all_tokens = token_1d
            all_pos = pos_1d

        if self.lang_config.fusion_mode == "concat":  # >>> mylerobot
            lang_tok = self.fusion.token(lang_emb).unsqueeze(0)  # (1, B, D)
            # Reuse a sinusoidal position 0 (zeros) for the lang token — keeps it
            # positionally distinct from the vision grid without needing a new param.
            lang_pos = torch.zeros_like(lang_tok[:, :1, :])  # (1, 1, D)
            all_tokens = torch.cat([lang_tok, all_tokens], dim=0)
            all_pos = torch.cat([lang_pos, all_pos], dim=0)

        # ---- Transformer encoder + decoder (unchanged) ----
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
        return actions, (mu, log_sigma_x2)


class ACTLangPolicy(ACTPolicy):
    """Drop-in replacement for ACTPolicy that uses `ACTLang` as its backbone."""

    config_class = ACTLangConfig
    name = "act_lang"

    def __init__(self, config: ACTLangConfig, **kwargs):
        # We cannot call ACTPolicy.__init__ unmodified because it constructs `ACT(config)`.
        # Replicate the relevant init steps and swap in ACTLang.
        from lerobot.policies.pretrained import PreTrainedPolicy

        PreTrainedPolicy.__init__(self, config)
        config.validate_features()
        self.config = config
        self.model = ACTLang(config)
        if config.temporal_ensemble_coeff is not None:
            from lerobot.policies.act.modeling_act import ACTTemporalEnsembler

            self.temporal_ensembler = ACTTemporalEnsembler(
                config.temporal_ensemble_coeff, config.chunk_size
            )
        self.reset()
