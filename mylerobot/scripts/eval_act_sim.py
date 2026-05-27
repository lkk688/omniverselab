#!/usr/bin/env python
"""Single-process gym_aloha rollout eval for a saved ACT (or ACTLang) checkpoint.

Bypasses lerobot.scripts.lerobot_eval's full plumbing — we don't need vector
envs or video saving for the Phase 1 baseline number. Just enough to:
  * load policy + processors from a HuggingFace-format pretrained dir,
  * roll N episodes through `gym_aloha/AlohaInsertion-v0` (or TransferCube),
  * report success rate + mean reward + per-episode breakdown,
  * write a JSON summary next to the checkpoint.

Usage:
    python -m mylerobot.scripts.eval_act_sim \
        --checkpoint /data/.../2026-02-22/21-21-20_aloha_act/checkpoints/020000/pretrained_model \
        --task insertion --n_episodes 50 --seed 0
"""

from __future__ import annotations

# CRITICAL: import mylerobot first so the env-compat shim is installed BEFORE
# any lerobot.* import.
import mylerobot  # noqa: F401

import argparse
import json
import os
import time
from pathlib import Path

import gymnasium as gym
import gym_aloha  # noqa: F401  -- registers gym_aloha/AlohaInsertion-v0 etc.
import numpy as np
import torch

from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors
from lerobot.envs.utils import preprocess_observation
from lerobot.utils.constants import ACTION

os.environ.setdefault("MUJOCO_GL", "egl")

TASK_TO_ENV_ID = {
    "insertion": "gym_aloha/AlohaInsertion-v0",
    "transfer_cube": "gym_aloha/AlohaTransferCube-v0",
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--checkpoint", required=True, type=Path,
                   help="Path to .../checkpoints/<N>/pretrained_model")
    p.add_argument("--task", default="insertion", choices=list(TASK_TO_ENV_ID))
    p.add_argument("--n_episodes", default=50, type=int)
    p.add_argument("--seed", default=0, type=int)
    p.add_argument("--max_steps", default=400, type=int,
                   help="Max steps per episode. Aloha tasks usually terminate well under 400.")
    p.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    p.add_argument("--output", default=None, type=Path,
                   help="JSON output path; defaults to <checkpoint>/eval_<task>_n<N>.json")
    return p.parse_args()


def load_policy(checkpoint: Path, device: str) -> tuple[ACTPolicy, object, object]:
    """Load policy + pre/post processors from a pretrained dir.

    Use `PreTrainedConfig.from_pretrained` (parent dispatcher) rather than
    `ACTConfig.from_pretrained` — the saved config.json carries a `type` key
    that the parent strips for dispatch but the child dataclass rejects.
    """
    cfg = PreTrainedConfig.from_pretrained(checkpoint)
    pre, post = make_pre_post_processors(cfg, pretrained_path=str(checkpoint))
    policy = ACTPolicy.from_pretrained(checkpoint)
    policy.to(device).eval()
    return policy, pre, post


def make_env(task: str, seed: int) -> gym.Env:
    env = gym.make(
        TASK_TO_ENV_ID[task],
        task=task,
        obs_type="pixels_agent_pos",  # gives both top image and 14-d qpos
        observation_width=640,
        observation_height=480,
    )
    env.reset(seed=seed)
    return env


def to_batched_numpy(obs: dict) -> dict:
    """gym single-env returns unbatched; preprocess_observation expects (B, ...).
    Convert each leaf to (1, ...) numpy."""
    out = {}
    for k, v in obs.items():
        if isinstance(v, dict):
            out[k] = {kk: vv[None] for kk, vv in v.items()}
        else:
            out[k] = v[None]
    return out


def rollout_episode(env: gym.Env, policy: ACTPolicy, pre, post,
                    max_steps: int, device: str, episode_seed: int) -> dict:
    obs, info = env.reset(seed=episode_seed)
    policy.reset()
    total_reward = 0.0
    max_reward = 0.0
    success = False
    steps = 0
    for step in range(max_steps):
        batched = to_batched_numpy(obs)
        observation = preprocess_observation(batched)
        # observation keys are now lerobot canonical: observation.images.top, observation.state
        # move to device
        observation = {k: v.to(device) if torch.is_tensor(v) else v for k, v in observation.items()}
        observation["task"] = [""]  # ACT ignores this, but the preprocessor pipeline may not
        observation = pre(observation)
        with torch.inference_mode():
            action = policy.select_action(observation)  # (1, action_dim)
        # `post` is a PolicyAction-in / PolicyAction-out pipeline (handles
        # unnormalization). It expects a bare tensor, not a {ACTION: tensor} dict.
        action = post(action)
        action_np = action.detach().cpu().numpy().reshape(-1)
        obs, reward, terminated, truncated, info = env.step(action_np)
        total_reward += float(reward)
        max_reward = max(max_reward, float(reward))
        if info.get("is_success", False):
            success = True
        steps = step + 1
        if terminated or truncated:
            break
    return {
        "success": bool(success),
        "sum_reward": total_reward,
        "max_reward": max_reward,
        "steps": steps,
        "seed": episode_seed,
    }


def main() -> None:
    args = parse_args()
    print(f"[eval_act_sim] checkpoint  = {args.checkpoint}")
    print(f"[eval_act_sim] task        = {args.task}")
    print(f"[eval_act_sim] n_episodes  = {args.n_episodes}")
    print(f"[eval_act_sim] device      = {args.device}")
    print(f"[eval_act_sim] MUJOCO_GL   = {os.environ.get('MUJOCO_GL')}")

    t0 = time.time()
    policy, pre, post = load_policy(args.checkpoint, args.device)
    print(f"[eval_act_sim] policy loaded in {time.time()-t0:.1f}s ({type(policy).__name__})")

    env = make_env(args.task, seed=args.seed)
    print(f"[eval_act_sim] env created: {TASK_TO_ENV_ID[args.task]}")

    episodes = []
    t0 = time.time()
    for i in range(args.n_episodes):
        ep_seed = args.seed + i
        ep = rollout_episode(env, policy, pre, post, args.max_steps, args.device, ep_seed)
        episodes.append(ep)
        running = sum(e["success"] for e in episodes) / len(episodes)
        elapsed = time.time() - t0
        eta = elapsed / (i + 1) * (args.n_episodes - i - 1)
        print(f"[eval_act_sim] ep {i+1:3d}/{args.n_episodes}  "
              f"success={ep['success']!s:5}  sum_r={ep['sum_reward']:.1f}  "
              f"max_r={ep['max_reward']:.1f}  steps={ep['steps']:3d}  "
              f"running_pc_success={running*100:.1f}%  eta={eta:.0f}s",
              flush=True)
    env.close()

    successes = [e["success"] for e in episodes]
    summary = {
        "checkpoint": str(args.checkpoint),
        "task": args.task,
        "n_episodes": args.n_episodes,
        "seed": args.seed,
        "max_steps": args.max_steps,
        "device": args.device,
        "pc_success": 100.0 * float(np.mean(successes)),
        "avg_sum_reward": float(np.mean([e["sum_reward"] for e in episodes])),
        "avg_max_reward": float(np.mean([e["max_reward"] for e in episodes])),
        "avg_steps": float(np.mean([e["steps"] for e in episodes])),
        "wall_time_s": time.time() - t0,
        "episodes": episodes,
    }
    out = args.output or (args.checkpoint / f"eval_{args.task}_n{args.n_episodes}.json")
    out.write_text(json.dumps(summary, indent=2))
    print(f"\n[eval_act_sim] === SUMMARY ===")
    print(f"  success rate : {summary['pc_success']:.1f}%  ({sum(successes)}/{args.n_episodes})")
    print(f"  avg sum_r    : {summary['avg_sum_reward']:.2f}")
    print(f"  avg max_r    : {summary['avg_max_reward']:.2f}")
    print(f"  avg steps    : {summary['avg_steps']:.1f}")
    print(f"  wall time    : {summary['wall_time_s']:.0f}s")
    print(f"  written to   : {out}")


if __name__ == "__main__":
    main()
