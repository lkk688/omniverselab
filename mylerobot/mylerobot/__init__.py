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
from mylerobot.policies.act_bev.configuration_act_bev import ACTBEVConfig
from mylerobot.policies.act_bev.modeling_act_bev import ACTBEVPolicy
from mylerobot.policies.act_cheat_noisy.configuration_act_cheat_noisy import ACTCheatNoisyConfig
from mylerobot.policies.act_cheat_noisy.modeling_act_cheat_noisy import ACTCheatNoisyPolicy

__all__ = [
    "ACTLangConfig", "ACTLangPolicy",
    "ACTBEVConfig", "ACTBEVPolicy",
    "ACTCheatNoisyConfig", "ACTCheatNoisyPolicy",
]
__version__ = "0.0.1"
