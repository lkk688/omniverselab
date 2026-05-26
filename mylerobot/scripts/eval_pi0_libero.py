#!/usr/bin/env python
"""LIBERO sim-rollout eval for a lerobot π0 checkpoint.

Mirrors `eval_act_sim.py` but for LIBERO + PI0Policy. Uses lerobot's
existing `LiberoEnv` adapter (which already remaps `agentview_image` /
`eye_in_hand_image` to canonical `observation.images.*` keys), so the
policy sees the same shape as it was trained on.

Usage:
    python scripts/eval_pi0_libero.py \
        --checkpoint lerobot/pi0_libero_finetuned_v044 \
        --suite libero_spatial \
        --n_episodes_per_task 3 \
        --max_steps 600

Either a HF Hub repo_id or a local path works for --checkpoint.
"""

from __future__ import annotations

# CRITICAL: import mylerobot first — installs env shim before lerobot.* loads.
import mylerobot  # noqa: F401

import argparse
import json
import os
import time
from pathlib import Path

import numpy as np
import torch

os.environ.setdefault("MUJOCO_GL", "egl")

from lerobot.configs import PreTrainedConfig
from lerobot.envs.libero import LiberoEnv
from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.pi0.modeling_pi0 import PI0Policy
from lerobot.utils.constants import ACTION
from libero.libero import benchmark


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--checkpoint", required=True, type=str,
                   help="HF repo_id or local dir of a PI0 checkpoint")
    p.add_argument("--suite", default="libero_spatial",
                   choices=["libero_spatial", "libero_object", "libero_goal",
                            "libero_10", "libero_90", "libero_100"])
    p.add_argument("--task_ids", type=int, nargs="+", default=None,
                   help="Subset of task indices in the suite (default: all)")
    p.add_argument("--n_episodes_per_task", type=int, default=3)
    p.add_argument("--max_steps", type=int, default=600)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    p.add_argument("--output", default=None, type=Path)
    p.add_argument("--obs_h", type=int, default=224)
    p.add_argument("--obs_w", type=int, default=224)
    return p.parse_args()


def load_policy(checkpoint: str, device: str):
    """Load PI0Policy + pre/post processors. Works with HF Hub repo_id or local dir."""
    cfg = PreTrainedConfig.from_pretrained(checkpoint)
    pre, post = make_pre_post_processors(cfg, pretrained_path=str(checkpoint))
    policy = PI0Policy.from_pretrained(checkpoint)
    policy.to(device).eval()
    return policy, pre, post


def make_env(suite_name: str, task_id: int, obs_h: int, obs_w: int) -> LiberoEnv:
    """Build a single LIBERO env for a specific (suite, task).

    See lerobot/envs/libero.py:110 for the actual constructor signature —
    camera_name defaults to "agentview_image,robot0_eye_in_hand_image" which
    the env auto-remaps to canonical `image`/`image2` keys via
    camera_name_mapping. We accept those defaults.
    """
    suite = benchmark.get_benchmark_dict()[suite_name]()
    env = LiberoEnv(
        task_suite=suite,
        task_id=task_id,
        task_suite_name=suite_name,
        observation_height=obs_h,
        observation_width=obs_w,
    )
    return env


def rollout_episode(env: LiberoEnv, policy, pre, post,
                    max_steps: int, device: str, episode_seed: int) -> dict:
    obs, info = env.reset(seed=episode_seed)
    policy.reset()
    success = False
    steps = 0
    total_reward = 0.0
    max_reward = 0.0

    for step in range(max_steps):
        # The env adapter already returns canonical lerobot-format observation.
        # Add batch dim and move to device.
        batch = {}
        for k, v in obs.items():
            if isinstance(v, np.ndarray):
                t = torch.from_numpy(v)
            elif isinstance(v, torch.Tensor):
                t = v
            else:
                continue
            if t.ndim == 1:
                t = t.unsqueeze(0)
            elif t.ndim == 3:  # HWC image -> add batch
                t = t.unsqueeze(0)
            batch[k] = t.to(device)
        batch["task"] = [env.task_description]

        batch = pre(batch)
        with torch.inference_mode():
            action = policy.select_action(batch)
        action = post(action)
        action_np = action.detach().cpu().numpy().reshape(-1)

        obs, reward, terminated, truncated, info = env.step(action_np)
        total_reward += float(reward)
        max_reward = max(max_reward, float(reward))
        if info.get("is_success", False) or terminated:
            success = info.get("is_success", False) or success
        steps = step + 1
        if terminated or truncated:
            break

    return {
        "success": bool(success),
        "steps": steps,
        "sum_reward": total_reward,
        "max_reward": max_reward,
        "seed": episode_seed,
    }


def main() -> None:
    args = parse_args()
    print(f"[eval_pi0_libero] checkpoint = {args.checkpoint}")
    print(f"[eval_pi0_libero] suite      = {args.suite}")
    print(f"[eval_pi0_libero] device     = {args.device}")
    print(f"[eval_pi0_libero] MUJOCO_GL  = {os.environ.get('MUJOCO_GL')}")

    t0 = time.time()
    policy, pre, post = load_policy(args.checkpoint, args.device)
    print(f"[eval_pi0_libero] policy loaded in {time.time()-t0:.1f}s ({type(policy).__name__})")

    suite = benchmark.get_benchmark_dict()[args.suite]()
    all_task_ids = list(range(len(suite.tasks)))
    task_ids = args.task_ids if args.task_ids is not None else all_task_ids
    print(f"[eval_pi0_libero] suite has {len(all_task_ids)} tasks; evaluating {len(task_ids)}: {task_ids}")

    all_results = {}
    t0 = time.time()
    for task_id in task_ids:
        task = suite.tasks[task_id]
        task_name = task.name
        print(f"\n[eval_pi0_libero] === task {task_id}: {task_name} ===")
        env = make_env(args.suite, task_id, args.obs_h, args.obs_w)
        task_eps = []
        for ep in range(args.n_episodes_per_task):
            ep_seed = args.seed + task_id * 100 + ep
            ep_result = rollout_episode(env, policy, pre, post,
                                         args.max_steps, args.device, ep_seed)
            task_eps.append(ep_result)
            running_pc = 100 * sum(e["success"] for e in task_eps) / len(task_eps)
            print(f"  ep {ep+1}/{args.n_episodes_per_task}  "
                  f"success={ep_result['success']!s:5}  steps={ep_result['steps']:3d}  "
                  f"max_r={ep_result['max_reward']:.1f}  task_pc={running_pc:.0f}%",
                  flush=True)
        env.close()
        all_results[task_name] = task_eps

    # Aggregate stats
    flat = [ep for eps in all_results.values() for ep in eps]
    n_total = len(flat)
    n_succ = sum(e["success"] for e in flat)
    summary = {
        "checkpoint": str(args.checkpoint),
        "suite": args.suite,
        "n_tasks_evaluated": len(task_ids),
        "n_episodes_per_task": args.n_episodes_per_task,
        "n_total_episodes": n_total,
        "n_successful": n_succ,
        "pc_success": 100.0 * n_succ / max(n_total, 1),
        "wall_time_s": time.time() - t0,
        "per_task": all_results,
    }
    out = args.output or Path(f"/data/rnd-liu/aiprojects/lerobot/outputs/eval/pi0_libero_{args.suite}.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(summary, indent=2))

    print(f"\n[eval_pi0_libero] === SUMMARY ===")
    print(f"  suite          : {args.suite}")
    print(f"  pc_success     : {summary['pc_success']:.1f}%  ({n_succ}/{n_total})")
    print(f"  wall_time      : {summary['wall_time_s']:.0f}s")
    print(f"  written to     : {out}")


if __name__ == "__main__":
    main()
