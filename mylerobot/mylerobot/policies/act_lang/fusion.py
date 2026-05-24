"""Three fusion modes for injecting a language embedding into ACT.

Each module's `forward` returns a *callable that operates on encoder inputs* or
directly produces a token to be appended. The ACTLang model in `modeling.py`
picks the right one based on `config.fusion_mode`.

Contract:
- `LangProj` (used by all modes): projects (B, lang_emb_dim) -> (B, dim_model).
- `ConcatFusion.token(lang_emb)`        -> (B, dim_model) extra encoder token.
- `FiLMFusion.modulate(feat, lang_emb)` -> (B, C, H, W) modulated feature map.
- `XAttnFusion(vis_tok, lang_tok)`      -> (S, B, dim_model) updated vision tokens.
"""

from __future__ import annotations

import torch
from torch import Tensor, nn


class LangProj(nn.Module):
    """Linear projection from lang embedding space to ACT's dim_model."""

    def __init__(self, lang_emb_dim: int, dim_model: int):
        super().__init__()
        self.proj = nn.Linear(lang_emb_dim, dim_model)

    def forward(self, x: Tensor) -> Tensor:
        return self.proj(x)


class ConcatFusion(nn.Module):
    """Project language to a single token; caller prepends it to encoder inputs."""

    def __init__(self, lang_emb_dim: int, dim_model: int, dropout: float = 0.0):
        super().__init__()
        self.proj = LangProj(lang_emb_dim, dim_model)
        self.drop = nn.Dropout(dropout)

    def token(self, lang_emb: Tensor) -> Tensor:
        """(B, lang_emb_dim) -> (B, dim_model)"""
        return self.drop(self.proj(lang_emb))


class FiLMFusion(nn.Module):
    """FiLM (Perez et al. 2018): scale + shift each channel of a visual feature map.

    Generates (gamma, beta) per channel from the language embedding, applied as
        feat = (1 + gamma) * feat + beta
    on the (B, C, H, W) backbone feature maps *before* the 1x1 conv projection.
    """

    def __init__(self, lang_emb_dim: int, feat_channels: int, dropout: float = 0.0):
        super().__init__()
        self.feat_channels = feat_channels
        hidden = max(feat_channels, lang_emb_dim)
        self.gamma_beta = nn.Sequential(
            nn.Linear(lang_emb_dim, hidden),
            nn.GELU(),
            nn.Dropout(dropout),
            nn.Linear(hidden, 2 * feat_channels),
        )

    def modulate(self, feat: Tensor, lang_emb: Tensor) -> Tensor:
        """feat: (B, C, H, W). lang_emb: (B, lang_emb_dim). -> (B, C, H, W)."""
        gb = self.gamma_beta(lang_emb)  # (B, 2C)
        gamma, beta = gb.chunk(2, dim=-1)  # each (B, C)
        gamma = gamma.unsqueeze(-1).unsqueeze(-1)
        beta = beta.unsqueeze(-1).unsqueeze(-1)
        return (1.0 + gamma) * feat + beta


class XAttnFusion(nn.Module):
    """One cross-attention block: vision tokens query language as key/value.

    Operates on (S, B, dim_model) sequences (ACT's convention is sequence-first).
    Output has the same shape as the vision input.
    """

    def __init__(
        self,
        lang_emb_dim: int,
        dim_model: int,
        n_heads: int = 4,
        dropout: float = 0.0,
    ):
        super().__init__()
        self.lang_proj = LangProj(lang_emb_dim, dim_model)
        self.attn = nn.MultiheadAttention(dim_model, n_heads, dropout=dropout)
        self.norm_q = nn.LayerNorm(dim_model)
        self.norm_kv = nn.LayerNorm(dim_model)
        self.norm_out = nn.LayerNorm(dim_model)
        self.dropout = nn.Dropout(dropout)

    def forward(self, vis_tokens: Tensor, lang_emb: Tensor) -> Tensor:
        """vis_tokens: (S, B, D). lang_emb: (B, lang_emb_dim). -> (S, B, D)."""
        kv = self.lang_proj(lang_emb).unsqueeze(0)  # (1, B, D) — one language token
        q = self.norm_q(vis_tokens)
        kv_n = self.norm_kv(kv)
        attended, _ = self.attn(query=q, key=kv_n, value=kv_n)
        return self.norm_out(vis_tokens + self.dropout(attended))
