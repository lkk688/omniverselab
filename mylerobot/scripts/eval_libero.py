#!/usr/bin/env python
"""Thin shim wrapper around `lerobot.scripts.lerobot_eval` for LIBERO.

Reason: lerobot_eval's `eval_main` does the full preprocessing pipeline
(`preprocess_observation` → `env_preprocessor` → policy `preprocessor`)
and uses `make_env_pre_post_processors` to build the LIBERO-specific
remapping. Our previous hand-rolled rollout (`eval_pi0_libero.py`) only
handled the policy-side processor and failed on the env↔policy boundary.

Wrap-and-delegate is cheaper than reimplementing.

Usage:
    python scripts/eval_libero.py \\
        --policy.path=lerobot/pi0_libero_finetuned_v044 \\
        --env.type=libero --env.task=libero_spatial \\
        --eval.n_episodes=10 --eval.batch_size=1 \\
        --eval.use_async_envs=false \\
        --output_dir=outputs/eval/pi0_libero_spatial_smoke
"""

import mylerobot  # noqa: F401  -- installs env shim before lerobot.* imports
import runpy

if __name__ == "__main__":
    runpy.run_module("lerobot.scripts.lerobot_eval", run_name="__main__")
