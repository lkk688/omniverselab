from dataclasses import dataclass
from typing import Literal

from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.act.configuration_act import ACTConfig


@PreTrainedConfig.register_subclass("act_lang")
@dataclass
class ACTLangConfig(ACTConfig):
    """ACT with a language-conditioning side-channel.

    The language input is expected as a precomputed embedding under the batch key
    `language_embedding` (shape: (B, lang_emb_dim)). The reason for taking an
    embedding rather than raw text is so the heavy text encoder (CLIP / DistilBERT
    / etc.) can be cached or run once per task — ACT itself stays cheap and 50Hz.

    Three fusion modes are implemented (see `mylerobot.policies.act_lang.fusion`):
    - "concat":  project lang_emb to dim_model, prepend as one extra encoder token.
    - "film":    use FiLM (gamma, beta) modulation on the visual feature maps
                 before they are projected and flattened into encoder tokens.
    - "xattn":   one cross-attention block where vision-tokens query language as
                 key/value, applied after vision projection, before transformer.
    """

    # Language side-channel
    lang_emb_dim: int = 512  # CLIP-ViT-B/32 text dim; override for DistilBERT (768)
    fusion_mode: Literal["concat", "film", "xattn"] = "concat"
    fusion_n_heads: int = 4   # used only by xattn
    fusion_dropout: float = 0.0
    # If True, allow batches to omit `language_embedding` and substitute zeros.
    # Useful for ablation runs and for loading old non-lang ACT checkpoints cleanly.
    lang_optional: bool = True

    def __post_init__(self):
        super().__post_init__()
        if self.fusion_mode not in {"concat", "film", "xattn"}:
            raise ValueError(f"fusion_mode must be concat|film|xattn, got {self.fusion_mode}")
        if self.lang_emb_dim <= 0:
            raise ValueError(f"lang_emb_dim must be positive, got {self.lang_emb_dim}")
