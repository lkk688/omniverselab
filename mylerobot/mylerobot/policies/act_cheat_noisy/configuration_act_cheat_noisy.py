"""ACT-Cheat-Noisy: ACT trained with Gaussian noise injected into specific
state dims (intended for the cube_pose dims of the cheat dataset).

Motivation
----------
The plain cheat baseline (cube_pose in observation.state) hits 35% success.
When we substitute the BEV's predicted cube XY at inference, success drops
to 0% because the cheat policy was trained with noiseless oracle pose and
cannot tolerate the ~2-10cm prediction noise produced by the BEV encoder
at workspace-edge cube positions.

This config adds Gaussian noise (in normalized state space) to the cube_pose
dims of observation.state during training only. The policy then learns a
control that is robust to small cube_pose perturbations, matching what the
BEV perception module produces at inference.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Tuple

from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.act.configuration_act import ACTConfig


@PreTrainedConfig.register_subclass("act_cheat_noisy")
@dataclass
class ACTCheatNoisyConfig(ACTConfig):
    """ACT with per-dim Gaussian noise on a slice of observation.state.

    Defaults match the cheat dataset's 19-D state layout:
      [j0..j8=arm/fingers(9), ee_pos(3), ee_quat(4), cube_x, cube_y, cube_z]
    so cube_pose lives at indices 16, 17, 18.

    The noise std values are in *normalized* state space (i.e. after the
    LeRobot MEAN_STD normalizer has been applied). Empirically, on the n=200
    Lift dataset, the cube_pose stats are
       std = (0.060, 0.133, 0.098)
    so a real-world noise σ of 2 cm corresponds to normalized σ of about
    (0.33, 0.15, 0.20). The defaults below approximate that.
    """

    # Indices into observation.state where cube_pose lives.
    cube_pose_state_indices: Tuple[int, ...] = (16, 17, 18)

    # Per-dim Gaussian σ in *normalized* state units (i.e. after MEAN_STD).
    # Applied training-only.
    cube_pose_noise_std_normalized: Tuple[float, ...] = (0.33, 0.15, 0.20)

    # If True, also randomly DROP cube_pose (replace with mean) on some fraction
    # of training samples — encourages the policy to fall back on vision when
    # the BEV perception fails entirely. Set to 0 for pure noise injection.
    cube_pose_dropout_prob: float = 0.0

    def __post_init__(self):
        super().__post_init__()
        n_idx = len(self.cube_pose_state_indices)
        n_std = len(self.cube_pose_noise_std_normalized)
        if n_idx != n_std:
            raise ValueError(
                f"cube_pose_state_indices ({n_idx}) and "
                f"cube_pose_noise_std_normalized ({n_std}) must match length"
            )
