#!/usr/bin/env python
"""LIBERO-Plus fragility scan: per-(suite, perturbation-category) success rate.

Samples N tasks per (suite, category) and rolls out a single π0 episode against
each via lerobot's full pipeline (preprocess → env_pre → pre → policy → post →
env_post). Outputs a 4×7 markdown table + per-task JSON.

Default: pi0_libero_finetuned_v044 (our official baseline). Compare future
voxel-fusion checkpoints against this table to quantify the robustness gain.

Usage:
    python scripts/eval_libero_plus_fragility.py \\
        --policy_path lerobot/pi0_libero_finetuned_v044 \\
        --suites libero_spatial libero_object libero_goal libero_10 \\
        --max_tasks_per_category 10 \\
        --max_steps 220 \\
        --output_json /data/rnd-liu/aiprojects/lerobot/outputs/eval/pi0_libero_plus_fragility.json
"""

from __future__ import annotations

# Shim must come before any lerobot import.
import mylerobot  # noqa: F401

import argparse
import json
import os
import time
from collections import defaultdict
from pathlib import Path

import numpy as np
import torch

os.environ.setdefault("MUJOCO_GL", "egl")

from lerobot.configs.policies import PreTrainedConfig
from lerobot.envs.configs import LiberoEnv as LiberoEnvConfig
from lerobot.envs.factory import make_env_pre_post_processors
from lerobot.envs.libero import LiberoEnv
from lerobot.envs.utils import preprocess_observation
from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.pi0.modeling_pi0 import PI0Policy
from lerobot.utils.constants import ACTION
from libero.libero import benchmark


def load_policy_polymorphic(path: str, device: str):
    """Load whichever PI0 variant the checkpoint's config.json declares.

    `PreTrainedConfig.from_pretrained` reads the saved config's `type` field
    and dispatches to the registered subclass (PI0Config or PI0VoxelConfig).
    We then pick the corresponding policy class by name.
    """
    cfg = PreTrainedConfig.from_pretrained(path)
    cfg_type = getattr(cfg, "type", None) or type(cfg).__name__
    if cfg_type == "pi0_voxel" or "Voxel" in type(cfg).__name__:
        # Ensure registration + import so the eager subclass is loaded.
        import mylerobot.policies.pi0_voxel  # noqa: F401
        from mylerobot.policies.pi0_voxel.modeling_pi0_voxel import PI0VoxelPolicy
        PolicyCls = PI0VoxelPolicy
    else:
        PolicyCls = PI0Policy
    pre, post = make_pre_post_processors(cfg, pretrained_path=str(path))
    policy = PolicyCls.from_pretrained(path).to(device).eval()
    return policy, pre, post, cfg


CATEGORIES = [
    "Camera Viewpoints",     # ← our headline target
    "Robot Initial States",
    "Objects Layout",
    "Light Conditions",
    "Background Textures",
    "Sensor Noise",
    "Language Instructions",
]


def _batched(obs: dict) -> dict:
    out = {}
    for k, v in obs.items():
        if isinstance(v, dict):
            out[k] = _batched(v)
        elif isinstance(v, np.ndarray):
            out[k] = v[None]
        else:
            out[k] = v
    return out


def rollout_one(env: LiberoEnv, policy, env_pre, env_post, pre, post,
                max_steps: int, device: str, task_desc: str,
                expected_keys: set[str] | None = None,
                drop_cam: str | None = None) -> dict:
    obs, info = env.reset(seed=None)
    policy.reset()
    success = False
    steps = 0
    max_r = 0.0
    for step in range(max_steps):
        observation = preprocess_observation(_batched(obs))
        # Keep on CPU while env_pre (LiberoProcessorStep) runs torch.flip etc.
        # Only move to device right before the policy. Cuts peak GPU usage and
        # avoids OOM when sharing the GPU with other processes.
        observation["task"] = [task_desc]
        observation = env_pre(observation)
        # Camera-dropout ablation: zero out one camera's image to simulate a
        # sensor failure. Must happen BEFORE rename below so the key still has
        # the env's original name (e.g. "observation.images.image").
        if drop_cam is not None:
            drop_key = f"observation.images.{drop_cam}"
            if drop_key in observation:
                v = observation[drop_key]
                if torch.is_tensor(v):
                    observation[drop_key] = torch.zeros_like(v)
        # Rename LIBERO env image keys -> dataset keys if the trained policy
        # expects different keys (e.g. pi0_voxel trained on Sylvest expects
        # observation.images.{front,wrist} but the env emits .image/.image2).
        # Must happen BEFORE `pre()` — the policy preprocessor validates against
        # the trained input_features schema. Only rename if the policy actually
        # expects the renamed key — otherwise we'd break baseline policies.
        expected = expected_keys or set()
        if "observation.images.front" in expected and "observation.images.image" in observation:
            observation["observation.images.front"] = observation.pop("observation.images.image")
        if "observation.images.wrist" in expected and "observation.images.image2" in observation:
            observation["observation.images.wrist"] = observation.pop("observation.images.image2")
        observation = pre(observation)
        observation = {k: (v.to(device, non_blocking=True) if torch.is_tensor(v) else v)
                       for k, v in observation.items()}
        with torch.inference_mode():
            action = policy.select_action(observation)
        action = post(action)
        transition = env_post({ACTION: action})
        action_np = transition[ACTION].detach().cpu().numpy().reshape(-1).astype(np.float32)

        obs, reward, terminated, truncated, info = env.step(action_np)
        max_r = max(max_r, float(reward))
        if info.get("is_success", False):
            success = True
        steps = step + 1
        if terminated or truncated or success:
            break
    return {"success": bool(success), "n_steps": steps, "max_reward": max_r}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--policy_path", default="lerobot/pi0_libero_finetuned_v044")
    p.add_argument("--suites", nargs="+",
                   default=["libero_spatial", "libero_object", "libero_goal", "libero_10"])
    p.add_argument("--categories", nargs="+", default=CATEGORIES)
    p.add_argument("--max_tasks_per_category", type=int, default=10,
                   help="Tasks sampled per (suite, category) cell")
    p.add_argument("--max_steps", type=int, default=220)
    p.add_argument("--n_episodes_per_task", type=int, default=1)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    p.add_argument("--output_json", type=Path,
                   default=Path("/data/rnd-liu/aiprojects/lerobot/outputs/eval/pi0_libero_plus_fragility.json"))
    p.add_argument("--classification_json", type=Path,
                   default=Path("/data/rnd-liu/aiprojects/LIBERO-plus/libero/libero/benchmark/task_classification.json"))
    p.add_argument("--corrupt_extrinsics", choices=["none", "identity", "shuffle"],
                   default="none",
                   help="Ablation hook for PI0VoxelPolicy: replace extrinsics with "
                        "identity matrices or shuffle per-camera. Tests whether the "
                        "voxel module actually uses geometry vs. just adds capacity.")
    p.add_argument("--drop_cam", choices=["none", "image", "image2"], default="none",
                   help="Sensor-failure ablation: zero out one camera's image at "
                        "every step (after env_pre, before policy). 'image' drops "
                        "the agentview cam, 'image2' drops the wrist cam. Tests "
                        "sensor-agnostic claim — how much does the policy degrade "
                        "when one cam is gone?")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    print(f"[fragility] policy = {args.policy_path}")
    print(f"[fragility] suites = {args.suites}")
    print(f"[fragility] categories = {args.categories}")
    print(f"[fragility] tasks/cell = {args.max_tasks_per_category}, "
          f"episodes/task = {args.n_episodes_per_task}, max_steps = {args.max_steps}")
    print(f"[fragility] output = {args.output_json}")

    # Load classification map.
    classification = json.loads(args.classification_json.read_text())

    # Load policy once (polymorphic — handles both PI0Policy + PI0VoxelPolicy ckpts).
    t0 = time.time()
    policy, pre, post, cfg = load_policy_polymorphic(args.policy_path, args.device)
    expected_input_keys = set(getattr(cfg, "input_features", {}).keys())
    print(f"[fragility] policy loaded in {time.time()-t0:.1f}s — type={type(policy).__name__}")

    if args.corrupt_extrinsics != "none":
        if not hasattr(policy, "_extrinsics_corruption"):
            print(f"[fragility] WARNING: --corrupt_extrinsics={args.corrupt_extrinsics} "
                  f"but policy {type(policy).__name__} has no _extrinsics_corruption hook "
                  "(only PI0VoxelPolicy supports this). Flag will be ignored.")
        else:
            policy._extrinsics_corruption = args.corrupt_extrinsics
            print(f"[fragility] EXTRINSICS CORRUPTED: mode={args.corrupt_extrinsics}")

    # Results: {(suite, category): {"successes": int, "total": int, "task_details": [...]}}
    results: dict = defaultdict(lambda: {"successes": 0, "total": 0, "task_details": []})
    total_episodes_planned = (
        len(args.suites) * len(args.categories)
        * args.max_tasks_per_category * args.n_episodes_per_task
    )
    print(f"[fragility] planned: {total_episodes_planned} total episodes")

    overall_t0 = time.time()
    episode_counter = 0

    for suite_name in args.suites:
        suite = benchmark.get_benchmark_dict()[suite_name]()
        # Map task NAME → position in suite.tasks (LiberoEnv accepts position as task_id)
        name_to_idx = {t.name: i for i, t in enumerate(suite.tasks)}
        suite_classif = classification.get(suite_name, [])

        # Build env_pre/env_post once per suite (depends on suite-level env_cfg)
        env_cfg = LiberoEnvConfig(task=suite_name)
        env_pre, env_post = make_env_pre_post_processors(env_cfg=env_cfg, policy_cfg=cfg)

        for category in args.categories:
            cat_tasks = [t for t in suite_classif if t["category"] == category]
            # Sample deterministically from the start; could randomize with --seed
            sampled = cat_tasks[: args.max_tasks_per_category]

            print(f"\n[fragility] === suite={suite_name} category={category} "
                  f"sampling {len(sampled)} of {len(cat_tasks)} tasks ===", flush=True)

            for task_entry in sampled:
                task_name = task_entry["name"]
                if task_name not in name_to_idx:
                    print(f"  SKIP task not in suite.tasks: {task_name[:60]}")
                    continue
                task_id = name_to_idx[task_name]
                task = suite.tasks[task_id]

                for ep_idx in range(args.n_episodes_per_task):
                    # Construct env fresh per episode (LiberoEnv increments init_state_id internally,
                    # we want deterministic episode_index)
                    env = LiberoEnv(
                        task_suite=suite,
                        task_id=task_id,
                        task_suite_name=suite_name,
                        observation_height=256,
                        observation_width=256,
                        obs_type="pixels_agent_pos",
                        episode_index=args.seed + ep_idx,
                        is_libero_plus=True,  # use the perturbation-suffix-aware init-state loader
                    )
                    t1 = time.time()
                    r = rollout_one(env, policy, env_pre, env_post, pre, post,
                                    args.max_steps, args.device, task.language,
                                    expected_keys=expected_input_keys,
                                    drop_cam=(args.drop_cam if args.drop_cam != "none" else None))
                    if hasattr(env, "close"):
                        env.close()

                    cell = results[(suite_name, category)]
                    cell["total"] += 1
                    if r["success"]:
                        cell["successes"] += 1
                    cell["task_details"].append({
                        "task_id_in_suite": task_id,
                        "task_name": task_name,
                        "difficulty": task_entry.get("difficulty_level"),
                        "success": r["success"],
                        "n_steps": r["n_steps"],
                        "max_reward": r["max_reward"],
                        "wall_s": time.time() - t1,
                    })
                    episode_counter += 1
                    pc = 100.0 * cell["successes"] / cell["total"]
                    elapsed = time.time() - overall_t0
                    eta = elapsed / episode_counter * (total_episodes_planned - episode_counter)
                    print(f"  ep {episode_counter}/{total_episodes_planned}  "
                          f"{'SUCC' if r['success'] else 'fail'}  steps={r['n_steps']:3d}  "
                          f"max_r={r['max_reward']:.1f}  ({time.time()-t1:.1f}s)  "
                          f"cell={cell['successes']}/{cell['total']} ({pc:.0f}%)  eta={eta/60:.0f}m",
                          flush=True)

    elapsed = time.time() - overall_t0
    print(f"\n[fragility] === DONE in {elapsed/60:.1f} min ===")

    # Build the markdown summary table.
    print("\n## Fragility table: % success (n=tasks/cell)\n")
    header = "| Suite | " + " | ".join(args.categories) + " | row mean |"
    sep = "|" + "|".join(["---"] * (len(args.categories) + 2)) + "|"
    print(header)
    print(sep)
    summary: dict = {}
    for suite_name in args.suites:
        row = [suite_name]
        suite_pcs = []
        for category in args.categories:
            cell = results.get((suite_name, category), {"successes": 0, "total": 0})
            if cell["total"] > 0:
                pc = 100.0 * cell["successes"] / cell["total"]
                row.append(f"{pc:.1f} (n={cell['total']})")
                suite_pcs.append(pc)
            else:
                row.append("—")
        row_mean = float(np.mean(suite_pcs)) if suite_pcs else 0.0
        row.append(f"{row_mean:.1f}")
        print("| " + " | ".join(row) + " |")
        summary[suite_name] = {
            cat: results.get((suite_name, cat), {"successes": 0, "total": 0})
            for cat in args.categories
        }
        summary[suite_name]["__row_mean_pc__"] = row_mean

    # Column means
    col_row = ["**col mean**"]
    all_pcs = []
    for category in args.categories:
        col_pcs = [100.0 * results[(s, category)]["successes"] / results[(s, category)]["total"]
                   for s in args.suites if results.get((s, category), {}).get("total", 0) > 0]
        col_mean = float(np.mean(col_pcs)) if col_pcs else 0.0
        col_row.append(f"**{col_mean:.1f}**")
        all_pcs.extend(col_pcs)
    overall_mean = float(np.mean(all_pcs)) if all_pcs else 0.0
    col_row.append(f"**{overall_mean:.1f}**")
    print("| " + " | ".join(col_row) + " |")

    # Persist
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    serializable = {
        "policy": args.policy_path,
        "suites": args.suites,
        "categories": args.categories,
        "max_tasks_per_category": args.max_tasks_per_category,
        "n_episodes_per_task": args.n_episodes_per_task,
        "max_steps": args.max_steps,
        "seed": args.seed,
        "corrupt_extrinsics": args.corrupt_extrinsics,
        "drop_cam": args.drop_cam,
        "wall_min": elapsed / 60.0,
        "overall_mean_pc": overall_mean,
        "per_suite": summary,
    }
    args.output_json.write_text(json.dumps(serializable, indent=2))
    print(f"\n[fragility] wrote {args.output_json}")


if __name__ == "__main__":
    main()
