"""Policy architectures for the causal-shortcut study.

Three ways perception enters the action head:
  - concat  : perception encoder -> flatten -> concat to proprio -> MLP
              (the act_bev v2/v3 design that failed)
  - xattn   : proprio is the query, perception tokens are keys/values,
              cross-attention -> action (dual-system / PointACT-style)
  - replace : perception decoded to an explicit tgt_hat, substituted into the
              proprio target slot (act_bev Option-A style)

Optional auxiliary head predicts the target from perception features (the
cube-heatmap analog) to test whether geometric supervision alone fixes
consumption (it shouldn't, per the act_bev result).

torch only. Small enough to train on CPU in seconds.
"""

from __future__ import annotations

import torch
import torch.nn as nn

from toy_reach import IMG_SIZE

PERC_FEAT_DIM = 64   # perception encoder output dim
HIDDEN = 128


def make_perception_encoder(perception_mode: str) -> tuple[nn.Module, int, bool]:
    """Returns (encoder, feat_dim, is_token).

    is_token=True means the encoder produces a sequence of tokens (for xattn);
    otherwise a single pooled vector.
    """
    if perception_mode == "none":
        return nn.Identity(), 0, False
    if perception_mode == "raw":
        # target is already a 2D coordinate; a tiny MLP lifts it
        enc = nn.Sequential(nn.Linear(2, PERC_FEAT_DIM), nn.ReLU(),
                            nn.Linear(PERC_FEAT_DIM, PERC_FEAT_DIM))
        return enc, PERC_FEAT_DIM, False
    if perception_mode == "image":
        return _ConvTokenizer(), PERC_FEAT_DIM, True
    raise ValueError(perception_mode)


class _ConvTokenizer(nn.Module):
    """16x16 single-channel blob -> a grid of tokens (B, N, PERC_FEAT_DIM).

    Also exposes a pooled vector via .pooled(feat). The token grid is what xattn
    consumes; concat/replace use the pooled vector / a decoded coordinate.

    NOTE on the pooled() path: mean-pooling over the spatial token grid DESTROYS
    location information — averaging a blob's features over positions loses where
    the blob was. For a localization task this caps decode accuracy. The
    softargmax injection path avoids this (see _SoftArgmaxHead).
    """

    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Conv2d(1, 32, 3, stride=2, padding=1),  # 16 -> 8
            nn.ReLU(),
            nn.Conv2d(32, PERC_FEAT_DIM, 3, stride=2, padding=1),  # 8 -> 4
            nn.ReLU(),
        )

    def forward(self, img: torch.Tensor) -> torch.Tensor:
        # img: (B, 1, 16, 16) -> (B, 16, PERC_FEAT_DIM) tokens (4x4 grid flattened)
        f = self.net(img)                      # (B, C, 4, 4)
        B, C, H, W = f.shape
        return f.flatten(2).transpose(1, 2)    # (B, H*W, C)

    @staticmethod
    def pooled(tokens: torch.Tensor) -> torch.Tensor:
        return tokens.mean(dim=1)              # (B, C)


class _SoftArgmaxHead(nn.Module):
    """Localization-appropriate readout: image -> heatmap -> soft-argmax -> (x,y).

    Predicts a per-pixel heatmap at FULL input resolution (no spatial
    downsampling that loses precision), softmaxes over the grid, and takes the
    expected coordinate. This is the inductive bias act_bev used to hit 1.18 cm
    — the spatial structure is preserved through to a continuous coordinate.
    Output is a 2D coordinate in [-1, 1] that the action head consumes directly.
    """

    def __init__(self, size: int = IMG_SIZE):
        super().__init__()
        self.net = nn.Sequential(
            nn.Conv2d(1, 32, 3, padding=1), nn.ReLU(),
            nn.Conv2d(32, 32, 3, padding=1), nn.ReLU(),
            nn.Conv2d(32, 1, 1),            # (B, 1, size, size) heatmap logits
        )
        coords = torch.linspace(-1.0, 1.0, size)
        gx, gy = torch.meshgrid(coords, coords, indexing="ij")
        self.register_buffer("grid_x", gx.reshape(-1))  # (size*size,)
        self.register_buffer("grid_y", gy.reshape(-1))

    def forward(self, img: torch.Tensor) -> torch.Tensor:
        logits = self.net(img).flatten(1)              # (B, size*size)
        w = torch.softmax(logits, dim=-1)              # (B, size*size)
        x = (w * self.grid_x).sum(-1, keepdim=True)    # (B, 1)
        y = (w * self.grid_y).sum(-1, keepdim=True)
        return torch.cat([x, y], dim=-1)               # (B, 2) coordinate


class ReachPolicy(nn.Module):
    def __init__(self, proprio_dim: int, perception_mode: str, injection: str):
        super().__init__()
        self.perception_mode = perception_mode
        self.injection = injection
        self.encoder, self.perc_dim, self.is_token = make_perception_encoder(perception_mode)

        self.is_softargmax = (injection == "softargmax")
        if self.is_softargmax:
            assert perception_mode == "image", "softargmax injection requires image perception"
            self.softargmax = _SoftArgmaxHead()
            # the soft-argmax module replaces the generic encoder; perc readout is a 2D coord
            self.encoder = self.softargmax
            self.perc_dim = 2
            self.is_token = False

        if perception_mode == "none":
            in_dim = proprio_dim
        elif injection == "concat":
            in_dim = proprio_dim + self.perc_dim
        elif injection == "xattn":
            in_dim = proprio_dim + self.perc_dim  # proprio + attended perception
        elif injection == "replace":
            # decoded target (2) replaces / augments proprio
            in_dim = proprio_dim + 2
            self.decoder = nn.Sequential(nn.Linear(self.perc_dim, 64), nn.ReLU(),
                                         nn.Linear(64, 2))
        elif injection == "softargmax":
            in_dim = proprio_dim + 2   # decoded coordinate feeds the action head
        else:
            raise ValueError(injection)

        if injection == "xattn" and perception_mode != "none":
            self.q_proj = nn.Linear(proprio_dim, self.perc_dim)
            self.attn = nn.MultiheadAttention(self.perc_dim, num_heads=4, batch_first=True)

        self.head = nn.Sequential(
            nn.Linear(in_dim, HIDDEN), nn.ReLU(),
            nn.Linear(HIDDEN, HIDDEN), nn.ReLU(),
            nn.Linear(HIDDEN, 2),
        )

        # Auxiliary target-prediction head (the cube-heatmap analog).
        # For softargmax, the decoded coordinate IS the aux prediction (the
        # heatmap is supervised directly), so no separate aux MLP is needed.
        if perception_mode == "none" or self.is_softargmax:
            self.aux_head = None
        else:
            self.aux_head = nn.Sequential(nn.Linear(self.perc_dim, 64), nn.ReLU(),
                                          nn.Linear(64, 2))

        self._last_perc_pooled = None  # stashed for aux loss
        self._last_coord = None        # stashed soft-argmax coordinate

    def _encode_perception(self, perception: torch.Tensor):
        """Returns (tokens_or_none, pooled_vec)."""
        if self.perception_mode == "none":
            return None, None
        if self.is_token:
            tokens = self.encoder(perception)          # (B, N, C)
            pooled = _ConvTokenizer.pooled(tokens)     # (B, C)
            return tokens, pooled
        feat = self.encoder(perception)                # (B, C)
        return None, feat

    def forward(self, proprio: torch.Tensor, perception: torch.Tensor,
                perc_ablate: bool = False):
        """perc_ablate=True zeroes the perception contribution (shortcut test)."""
        if self.perception_mode == "none":
            self._last_perc_pooled = None
            return self.head(proprio)

        if self.is_softargmax:
            coord = self.softargmax(perception)              # (B, 2) decoded coordinate
            self._last_coord = coord
            if perc_ablate:
                coord = torch.zeros_like(coord)
            x = torch.cat([proprio, coord], dim=-1)
            return self.head(x)

        tokens, pooled = self._encode_perception(perception)
        self._last_perc_pooled = pooled

        if perc_ablate:
            pooled = torch.zeros_like(pooled)
            if tokens is not None:
                tokens = torch.zeros_like(tokens)

        if self.injection == "concat":
            x = torch.cat([proprio, pooled], dim=-1)
        elif self.injection == "xattn":
            q = self.q_proj(proprio).unsqueeze(1)            # (B, 1, C)
            kv = tokens if tokens is not None else pooled.unsqueeze(1)
            attended, _ = self.attn(q, kv, kv)               # (B, 1, C)
            x = torch.cat([proprio, attended.squeeze(1)], dim=-1)
        elif self.injection == "replace":
            tgt_hat = self.decoder(pooled)                   # (B, 2)
            x = torch.cat([proprio, tgt_hat], dim=-1)
        return self.head(x)

    def aux_predict(self) -> torch.Tensor | None:
        """Predict target from the last forward's perception feature.
        For softargmax this is the decoded coordinate (the heatmap is supervised
        directly); otherwise it's the aux MLP on pooled features."""
        if self.is_softargmax:
            return self._last_coord
        if self.aux_head is None or self._last_perc_pooled is None:
            return None
        return self.aux_head(self._last_perc_pooled)

    def probe_target(self, perception: torch.Tensor) -> torch.Tensor | None:
        """Decode target directly from perception, bypassing proprio + action head.

        Used by the perception-probe metric so it doesn't depend on the
        training-time proprio dimension. Note: the aux_head is only trained
        when aux_weight > 0; with aux_weight=0 this measures the *untrained*
        aux head (≈ chance), which is the correct null — it tells us the
        perception probe only works when we explicitly supervise it.
        """
        if self.perception_mode == "none":
            return None
        if self.is_softargmax:
            return self.softargmax(perception)
        if self.aux_head is None:
            return None
        _, pooled = self._encode_perception(perception)
        return self.aux_head(pooled)

    def perception_params(self):
        """Params of the perception module to aux-pretrain + freeze in the
        two-stage decouple. Injection-agnostic so train_policy doesn't special-case."""
        if self.is_softargmax:
            return list(self.softargmax.parameters())
        return list(self.encoder.parameters())
