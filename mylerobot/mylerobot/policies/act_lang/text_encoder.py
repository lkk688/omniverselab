"""Frozen text encoder wrappers.

Phase 0 ships only the interface + a deterministic dummy used by tests. Real
CLIP / DistilBERT wrappers are gated behind the `text` extra (transformers) and
will be loaded lazily so the smoke test does not pull HF Hub.
"""

from __future__ import annotations

from typing import Iterable

import torch
from torch import Tensor, nn


class DummyTextEncoder(nn.Module):
    """Deterministic hash-based pseudo-encoder.

    Maps strings to a fixed embedding via Python's `hash` + torch.manual_seed.
    NOT for training — purely for smoke tests so we don't depend on HF Hub.
    """

    def __init__(self, embed_dim: int = 512):
        super().__init__()
        self.embed_dim = embed_dim

    @torch.no_grad()
    def forward(self, texts: Iterable[str]) -> Tensor:
        out = []
        for t in texts:
            g = torch.Generator().manual_seed(abs(hash(t)) % (2**31))
            out.append(torch.randn(self.embed_dim, generator=g))
        return torch.stack(out, dim=0)


def build_text_encoder(name: str, embed_dim: int) -> nn.Module:
    """Lazy factory. Only `dummy` is built-in; others require `pip install -e .[text]`."""
    if name == "dummy":
        return DummyTextEncoder(embed_dim=embed_dim)
    if name.startswith("clip:"):
        try:
            from transformers import CLIPTextModel, CLIPTokenizer  # noqa: F401
        except ImportError as e:
            raise ImportError("Install with `pip install -e mylerobot[text]` to use CLIP.") from e
        raise NotImplementedError("CLIP wrapper to be added in Phase 1; use 'dummy' for now.")
    raise ValueError(f"unknown text encoder: {name}")
