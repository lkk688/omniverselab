"""mylerobot - external research extensions on top of lerobot.

The first thing this package does is install env-compat shims (see
`mylerobot._env_shim`). Those must run BEFORE any `lerobot.*` import,
otherwise the broken groot `@strict` clash blows up `lerobot.policies/__init__.py`.

After the shim is in place, we import + register `ACTLangPolicy` so configs
with `type: act_lang` resolve correctly through lerobot's policy registry.
"""

# IMPORTANT: keep `_env_shim` as the very first import. Order matters.
from mylerobot import _env_shim as _env_shim  # noqa: F401

from mylerobot.policies.act_lang.configuration import ACTLangConfig
from mylerobot.policies.act_lang.modeling import ACTLangPolicy

# Side-effect: registers PI0VoxelConfig as type="pi0_voxel" so lerobot_train's
# CLI choices include it. Safe to import early — modeling uses lazy factories
# that don't pull in heavy pi0 deps until actually instantiated.
from mylerobot.policies.pi0_voxel.configuration_pi0_voxel import PI0VoxelConfig

# ACT-BEV: calibrated multi-view BEV encoder + cube-heatmap aux head (rtx5090 work).
from mylerobot.policies.act_bev.configuration_act_bev import ACTBEVConfig
from mylerobot.policies.act_bev.modeling_act_bev import ACTBEVPolicy

# ACT-Cheat-Noisy: ACT trained with Gaussian noise on cube_pose state dims (rtx5090 work).
from mylerobot.policies.act_cheat_noisy.configuration_act_cheat_noisy import ACTCheatNoisyConfig
from mylerobot.policies.act_cheat_noisy.modeling_act_cheat_noisy import ACTCheatNoisyPolicy

__all__ = [
    "ACTLangConfig", "ACTLangPolicy",
    "PI0VoxelConfig",
    "ACTBEVConfig", "ACTBEVPolicy",
    "ACTCheatNoisyConfig", "ACTCheatNoisyPolicy",
]
__version__ = "0.0.1"
