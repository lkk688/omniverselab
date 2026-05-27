"""ACT-Cheat-Noisy policy.

Subclasses ACTPolicy and injects Gaussian noise into the cube_pose slice of
`observation.state` during training only. The injected noise lives in
*normalized* state space (post-MEAN_STD), so its magnitude in real-world
units is `cube_pose_noise_std_normalized × dataset.std[cube_pose_dim]`.

Inference path is unchanged — at eval the policy receives whatever cube_pose
value the caller provides (GT oracle or BEV prediction).
"""

from __future__ import annotations

import torch
from torch import Tensor

from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.utils.constants import OBS_STATE

from mylerobot.policies.act_cheat_noisy.configuration_act_cheat_noisy import (
    ACTCheatNoisyConfig,
)


class ACTCheatNoisyPolicy(ACTPolicy):
    config_class = ACTCheatNoisyConfig
    name = "act_cheat_noisy"

    def __init__(self, config: ACTCheatNoisyConfig, **kwargs):
        super().__init__(config, **kwargs)
        self.noisy_cfg = config

        # Pre-build a per-dim std tensor over the full state width, with the
        # configured σ at the cube_pose indices and 0 elsewhere. This makes
        # the noise injection one tensor multiply + add.
        state_dim = config.robot_state_feature.shape[0] if config.robot_state_feature else 0
        std = torch.zeros(state_dim)
        for idx, sigma in zip(
            config.cube_pose_state_indices, config.cube_pose_noise_std_normalized
        ):
            if 0 <= idx < state_dim:
                std[idx] = sigma
        self.register_buffer("_noise_std_per_dim", std, persistent=False)

    def _inject_state_noise(self, batch: dict[str, Tensor]) -> dict[str, Tensor]:
        if OBS_STATE not in batch:
            return batch
        state = batch[OBS_STATE]
        if state.ndim < 2:
            return batch
        std = self._noise_std_per_dim.to(device=state.device, dtype=state.dtype)
        # Per-sample, per-dim independent noise.
        noise = torch.randn_like(state) * std
        new_state = state + noise

        # Optional dropout: with prob p, zero out the cube_pose slice for an
        # entire sample. Mean of normalized cube_pose is 0 (post-MEAN_STD),
        # so dropping to zero is "predicted mean", a plausible BEV failure.
        if self.noisy_cfg.cube_pose_dropout_prob > 0:
            mask = (
                torch.rand(state.shape[0], device=state.device)
                < self.noisy_cfg.cube_pose_dropout_prob
            )
            if mask.any():
                idx_tensor = torch.tensor(
                    list(self.noisy_cfg.cube_pose_state_indices),
                    device=state.device,
                    dtype=torch.long,
                )
                drop_mask = torch.zeros_like(state[0], dtype=torch.bool)
                drop_mask[idx_tensor] = True
                # zero the cube_pose dims for masked samples
                drop_full = mask.unsqueeze(-1) & drop_mask.unsqueeze(0)
                new_state = torch.where(
                    drop_full, torch.zeros_like(new_state), new_state
                )

        batch = dict(batch)
        batch[OBS_STATE] = new_state
        return batch

    def forward(self, batch: dict[str, Tensor]) -> tuple[Tensor, dict]:
        if self.training:
            batch = self._inject_state_noise(batch)
        return super().forward(batch)
