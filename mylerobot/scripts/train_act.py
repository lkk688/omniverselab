#!/usr/bin/env python
"""Thin shim wrapper around `lerobot.scripts.lerobot_train`.

Why this exists: in our current py312 env, `lerobot.policies/__init__.py`
fails to import due to a groot @strict bug (huggingface_hub 1.7.1 strictness).
We install the test-and-dev shim in `mylerobot._env_shim` BEFORE handing
control to upstream lerobot_train.

Forwards all CLI args verbatim to the upstream entry. Usage:

    python scripts/train_act.py \\
        --dataset.repo_id=lerobot/aloha_sim_insertion_human \\
        --policy.type=act \\
        --env.type=aloha --env.task=AlohaInsertion-v0 \\
        --output_dir=outputs/train/<date>/<run_name> \\
        --job_name=aloha_act_100k --steps=100000 ...

Identical to invoking `python -m lerobot.scripts.lerobot_train ...` directly
once the env is fixed.
"""

import mylerobot  # noqa: F401  -- installs env shim
import runpy

if __name__ == "__main__":
    runpy.run_module("lerobot.scripts.lerobot_train", run_name="__main__")
