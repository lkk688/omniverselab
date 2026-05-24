"""Phase 0 smoke test: build an ACTLangPolicy in each fusion mode, run a fake
batch forward + backward, and check shapes / finiteness.

No real text encoder, no real dataset — just a deterministic synthetic batch.
This protects us from upstream-ACT changes breaking our subclass.
"""

from __future__ import annotations

import pytest
import torch

from lerobot.configs.types import FeatureType, PolicyFeature
from lerobot.utils.constants import ACTION, OBS_IMAGE, OBS_IMAGES, OBS_STATE

from mylerobot.policies.act_lang import ACTLangConfig, ACTLangPolicy
from mylerobot.policies.act_lang.modeling import LANG_EMBED_KEY


CAM_KEY = f"{OBS_IMAGE}.top"  # any key starting with observation.image is treated as a camera


def _make_config(fusion_mode: str, lang_emb_dim: int = 64) -> ACTLangConfig:
    cfg = ACTLangConfig(
        # Tiny-ish dims for fast forward.
        chunk_size=8,
        n_action_steps=8,
        dim_model=64,
        n_heads=4,
        dim_feedforward=128,
        n_encoder_layers=2,
        n_decoder_layers=1,
        n_vae_encoder_layers=2,
        latent_dim=8,
        # No torchvision download in CI: drop pretrained weights.
        pretrained_backbone_weights=None,
        # Fusion-specific
        fusion_mode=fusion_mode,
        lang_emb_dim=lang_emb_dim,
        fusion_n_heads=2,
        lang_optional=True,
    )
    # Manually populate input / output features (skipping dataset stats parsing).
    cfg.input_features = {
        CAM_KEY: PolicyFeature(type=FeatureType.VISUAL, shape=(3, 96, 96)),
        OBS_STATE: PolicyFeature(type=FeatureType.STATE, shape=(7,)),
    }
    cfg.output_features = {
        ACTION: PolicyFeature(type=FeatureType.ACTION, shape=(7,)),
    }
    return cfg


def _fake_batch(cfg: ACTLangConfig, batch_size: int = 2, with_lang: bool = True) -> dict:
    B, S = batch_size, cfg.chunk_size
    batch = {
        CAM_KEY: torch.randn(B, 3, 96, 96),
        OBS_STATE: torch.randn(B, 7),
        ACTION: torch.randn(B, S, 7),
        "action_is_pad": torch.zeros(B, S, dtype=torch.bool),
    }
    if with_lang:
        batch[LANG_EMBED_KEY] = torch.randn(B, cfg.lang_emb_dim)
    return batch


@pytest.mark.parametrize("fusion_mode", ["concat", "film", "xattn"])
def test_forward_with_lang(fusion_mode):
    cfg = _make_config(fusion_mode)
    policy = ACTLangPolicy(cfg).train()
    batch = _fake_batch(cfg, with_lang=True)

    loss, info = policy.forward(batch)

    assert torch.isfinite(loss).item(), f"loss not finite in mode={fusion_mode}: {loss}"
    assert loss.requires_grad, "loss must require grad in train mode"
    loss.backward()  # check graph is well-formed
    # at least one fusion param should have a non-zero grad
    fusion_params = [p for n, p in policy.named_parameters() if "fusion" in n]
    assert fusion_params, "no fusion params found — check subclass wiring"
    assert any(p.grad is not None and p.grad.abs().sum() > 0 for p in fusion_params), (
        f"fusion params got no gradient in mode={fusion_mode}"
    )


@pytest.mark.parametrize("fusion_mode", ["concat", "film", "xattn"])
def test_forward_without_lang_when_optional(fusion_mode):
    """lang_optional=True must let us run without `language_embedding` in batch."""
    cfg = _make_config(fusion_mode)
    policy = ACTLangPolicy(cfg).train()
    batch = _fake_batch(cfg, with_lang=False)
    loss, _ = policy.forward(batch)
    assert torch.isfinite(loss).item()


def test_missing_lang_raises_when_required():
    cfg = _make_config("concat")
    cfg.lang_optional = False
    policy = ACTLangPolicy(cfg).train()
    batch = _fake_batch(cfg, with_lang=False)
    with pytest.raises(KeyError):
        policy.forward(batch)


def test_inference_action_shape():
    """eval-mode action chunk has shape (B, chunk_size, action_dim)."""
    cfg = _make_config("concat")
    policy = ACTLangPolicy(cfg).eval()
    batch = _fake_batch(cfg, with_lang=True)
    # predict_action_chunk pops ACTION from batch internally only when training,
    # so we keep it (it's ignored in eval).
    actions = policy.predict_action_chunk(batch)
    assert actions.shape == (2, cfg.chunk_size, 7), actions.shape
    assert torch.isfinite(actions).all()


def test_loading_vanilla_act_checkpoint_keys_compatible():
    """The new fusion params must be the *only* state_dict keys missing when
    loading a vanilla ACT state dict. This guarantees `strict=False` warm-start
    of the 2026-02-22 checkpoints will succeed cleanly."""
    from lerobot.policies.act.configuration_act import ACTConfig
    from lerobot.policies.act.modeling_act import ACTPolicy

    base_cfg = ACTConfig(
        chunk_size=8,
        n_action_steps=8,
        dim_model=64,
        n_heads=4,
        dim_feedforward=128,
        n_encoder_layers=2,
        n_decoder_layers=1,
        n_vae_encoder_layers=2,
        latent_dim=8,
        pretrained_backbone_weights=None,
    )
    base_cfg.input_features = {
        CAM_KEY: PolicyFeature(type=FeatureType.VISUAL, shape=(3, 96, 96)),
        OBS_STATE: PolicyFeature(type=FeatureType.STATE, shape=(7,)),
    }
    base_cfg.output_features = {ACTION: PolicyFeature(type=FeatureType.ACTION, shape=(7,))}
    base = ACTPolicy(base_cfg)
    base_keys = set(base.state_dict().keys())

    lang_cfg = _make_config("concat")
    lang = ACTLangPolicy(lang_cfg)
    lang_keys = set(lang.state_dict().keys())

    extra = lang_keys - base_keys
    missing = base_keys - lang_keys
    assert missing == set(), f"subclass should not drop any upstream keys, missing={missing}"
    # All new keys must live under model.fusion.*
    assert all(k.startswith("model.fusion.") for k in extra), f"unexpected new keys: {extra}"
