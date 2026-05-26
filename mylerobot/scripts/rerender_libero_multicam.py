#!/usr/bin/env python
"""v2: re-render LIBERO with multi-camera + extrinsics via lerobot's validated
pi0 inference pipeline.

Why v2 over v1:
v1 (`build_policy_batch`) tried to manually construct pi0 inputs from raw
robosuite observations. That mis-encoded `observation.state` (different
reference frame from what lerobot/libero used at training time) → policy
failed 100% of episodes. v2 delegates ALL policy-side preprocessing to
lerobot's own machinery — same code path that gives 66% mean on libero_spatial.

Architecture:
  - `LiberoMultiCamCaptureEnv` (subclass of `lerobot.envs.libero.LiberoEnv`)
    renders extra cameras + captures extrinsics into `env.recent_extras`.
    Standard observation (what pi0 sees) is UNCHANGED.
  - Drive pi0 via `make_pre_post_processors` + `make_env_pre_post_processors`
    + `policy.select_action`, mirroring `lerobot.scripts.lerobot_eval.rollout`
    but on a single env.
  - After each step, pull `env.get_recent_extras()` → write into a per-episode
    buffer. On `info["is_success"]`, save the episode to a LeRobotDataset.

Usage:
    python scripts/rerender_libero_multicam.py \\
        --suites libero_spatial \\
        --max_init_states 2 \\
        --output_repo_id local/libero_multicam_smoke_v2

For a full run drop `--max_init_states` (uses all 50/task).
"""

from __future__ import annotations

import mylerobot  # noqa: F401  -- install env shim BEFORE any lerobot import

import argparse
import os
import time
from pathlib import Path
from typing import Sequence

import numpy as np
import torch

os.environ.setdefault("MUJOCO_GL", "egl")

from lerobot.configs import PreTrainedConfig
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.envs.configs import LiberoEnv as LiberoEnvConfig
from lerobot.envs.utils import preprocess_observation
from lerobot.envs.factory import make_env_pre_post_processors
from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.pi0.modeling_pi0 import PI0Policy
from lerobot.utils.constants import ACTION
from libero.libero import benchmark

from mylerobot.sim.libero_multicam import DEFAULT_MULTICAM, make_capture_env


def _batched(obs: dict) -> dict:
    """Add a leading batch dim to every leaf in `obs` (matches gym vector-env
    semantics that lerobot's `preprocess_observation` expects)."""
    out = {}
    for k, v in obs.items():
        if isinstance(v, dict):
            out[k] = _batched(v)
        elif isinstance(v, np.ndarray):
            out[k] = v[None]
        else:
            out[k] = v
    return out


def rollout_one_episode(env, policy, env_pre, env_post, pre, post,
                        max_steps: int, device: str, task_description: str):
    obs, info = env.reset(seed=None)
    policy.reset()

    extras_seq: list[dict] = []
    actions_seq: list[np.ndarray] = []
    states_seq: list[np.ndarray] = []  # use the 8d state pi0 saw (from env_pre output)
    success = False

    for step in range(max_steps):
        # Capture multi-cam extras for THIS frame (before policy acts).
        extras = env.get_recent_extras()
        extras_seq.append(extras)

        # Build pi0 input via the validated lerobot pipeline.
        observation = preprocess_observation(_batched(obs))
        observation = {k: (v.to(device) if torch.is_tensor(v) else v) for k, v in observation.items()}
        observation["task"] = [task_description]
        observation = env_pre(observation)
        # Record RAW state (after LiberoProcessorStep flatten, before pre's normalization)
        # so the saved dataset matches lerobot/libero conventions and downstream
        # normalization stats are computed cleanly.
        if "observation.state" in observation:
            s = observation["observation.state"][0].detach().cpu().numpy().astype(np.float32)
            states_seq.append(s)
        observation = pre(observation)

        with torch.inference_mode():
            action = policy.select_action(observation)
        action = post(action)
        action_dict = {ACTION: action}
        action_dict = env_post(action_dict)
        action_np = action_dict[ACTION].detach().cpu().numpy().reshape(-1).astype(np.float32)
        actions_seq.append(action_np)

        obs, reward, terminated, truncated, info = env.step(action_np)
        if info.get("is_success", False):
            success = True
        if terminated or truncated or success:
            # Capture final frame's extras too
            extras_seq.append(env.get_recent_extras())
            break

    return {
        "success": success,
        "n_steps": len(actions_seq),
        "extras_seq": extras_seq[: len(actions_seq)],  # align lengths
        "states": np.stack(states_seq) if states_seq else np.zeros((0, 8), dtype=np.float32),
        "actions": np.stack(actions_seq) if actions_seq else np.zeros((0, 7), dtype=np.float32),
    }


def features_for_multicam(cams: Sequence[str], H: int, W: int, state_dim: int, action_dim: int) -> dict:
    feats = {
        "observation.state": {"dtype": "float32", "shape": (state_dim,), "names": ["state"]},
        "action": {"dtype": "float32", "shape": (action_dim,), "names": ["action"]},
    }
    for c in cams:
        feats[f"observation.images.{c}"] = {
            "dtype": "image", "shape": (H, W, 3), "names": ["height", "width", "channels"],
        }
        feats[f"observation.cam_extrinsics_world2cam_cv.{c}"] = {
            "dtype": "float32", "shape": (4, 4), "names": ["row", "col"],
        }
        feats[f"observation.cam_intrinsics_K.{c}"] = {
            "dtype": "float32", "shape": (3, 3), "names": ["row", "col"],
        }
    return feats


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--policy_path", default="lerobot/pi0_libero_finetuned_v044")
    p.add_argument("--suites", nargs="+", default=["libero_spatial"])
    p.add_argument("--extra_cameras", nargs="+",
                   default=["frontview", "sideview", "birdview"],
                   help="Extra cameras to render beyond agentview + robot0_eye_in_hand")
    p.add_argument("--cam_h", type=int, default=256)
    p.add_argument("--cam_w", type=int, default=256)
    p.add_argument("--max_steps", type=int, default=400)
    p.add_argument("--max_init_states", type=int, default=None)
    p.add_argument("--max_tasks_per_suite", type=int, default=None)
    p.add_argument("--output_root", default="/data/rnd-liu/.cache/lerobot_local")
    p.add_argument("--output_repo_id", default="local/libero_multicam_v0")
    p.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    p.add_argument("--fps", type=int, default=20)
    return p.parse_args()


def main() -> None:
    args = parse_args()
    standard_cams = ("agentview", "robot0_eye_in_hand")
    all_cam_names = list(standard_cams) + list(args.extra_cameras)
    print(f"[rerender v2] policy = {args.policy_path}")
    print(f"[rerender v2] suites = {args.suites}")
    print(f"[rerender v2] all cams = {all_cam_names}  size=({args.cam_h}, {args.cam_w})")
    print(f"[rerender v2] output = {args.output_root}/{args.output_repo_id}")

    t0 = time.time()
    cfg = PreTrainedConfig.from_pretrained(args.policy_path)
    pre, post = make_pre_post_processors(cfg, pretrained_path=str(args.policy_path))
    policy = PI0Policy.from_pretrained(args.policy_path).to(args.device).eval()
    print(f"[rerender v2] policy loaded in {time.time()-t0:.1f}s")

    out_root = Path(args.output_root) / args.output_repo_id
    if out_root.exists():
        print(f"[rerender v2] FATAL: {out_root} already exists; pick a fresh repo_id or rm -rf.")
        return

    features = features_for_multicam(all_cam_names, args.cam_h, args.cam_w, 8, 7)
    ds = LeRobotDataset.create(
        repo_id=args.output_repo_id, fps=args.fps, root=out_root,
        features=features, use_videos=True, image_writer_threads=4,
    )

    total_success = 0
    total_attempts = 0
    suite_dict = benchmark.get_benchmark_dict()
    for suite_name in args.suites:
        suite = suite_dict[suite_name]()
        n_tasks = len(suite.tasks)
        if args.max_tasks_per_suite is not None:
            n_tasks = min(n_tasks, args.max_tasks_per_suite)

        # Build env-cfg + env_pre/post once per suite (env_pre depends on policy_cfg too)
        env_cfg = LiberoEnvConfig(task=suite_name)
        env_pre, env_post = make_env_pre_post_processors(env_cfg=env_cfg, policy_cfg=cfg)

        for task_id in range(n_tasks):
            task = suite.tasks[task_id]
            init_states = suite.get_task_init_states(task_id)
            n_init = init_states.shape[0]
            if args.max_init_states is not None:
                n_init = min(n_init, args.max_init_states)

            print(f"\n[rerender v2] === suite={suite_name} task={task_id} "
                  f"({task.name[:55]}) init_states={n_init} ===", flush=True)

            for ep_idx in range(n_init):
                # Fresh env per init_state to guarantee correct init_state selection.
                # (LiberoEnv increments init_state_id on each reset; explicit
                #  reconstruction is simpler than mutating it between resets.)
                env = make_capture_env(
                    task_suite=suite, task_id=task_id, task_suite_name=suite_name,
                    extra_cameras=args.extra_cameras,
                    observation_height=args.cam_h, observation_width=args.cam_w,
                    episode_index=ep_idx,
                    obs_type="pixels_agent_pos",  # so robot_state is in obs → LiberoProcessorStep can build state
                )
                t1 = time.time()
                result = rollout_one_episode(
                    env, policy, env_pre, env_post, pre, post,
                    args.max_steps, args.device, task.language,
                )
                env.close() if hasattr(env, "close") else None
                total_attempts += 1
                if result["success"]:
                    total_success += 1
                    n_steps = result["n_steps"]
                    for f in range(n_steps):
                        frame = {
                            "observation.state": torch.from_numpy(result["states"][f]),
                            "action": torch.from_numpy(result["actions"][f]),
                            "task": task.language,  # add_frame expects task IN the frame dict
                        }
                        for c in all_cam_names:
                            ex = result["extras_seq"][f][c]
                            frame[f"observation.images.{c}"] = ex["image"]
                            frame[f"observation.cam_extrinsics_world2cam_cv.{c}"] = \
                                torch.from_numpy(ex["extrinsic_world2cam_cv"].astype(np.float32))
                            frame[f"observation.cam_intrinsics_K.{c}"] = \
                                torch.from_numpy(ex["intrinsic_K"].astype(np.float32))
                        ds.add_frame(frame)
                    ds.save_episode()
                    print(f"  ep {ep_idx}/{n_init}: SUCCESS {result['n_steps']:3d} steps "
                          f"({time.time()-t1:.1f}s) | total {total_success}/{total_attempts} "
                          f"({100*total_success/total_attempts:.1f}%)", flush=True)
                else:
                    print(f"  ep {ep_idx}/{n_init}: fail    {result['n_steps']:3d} steps "
                          f"({time.time()-t1:.1f}s) | total {total_success}/{total_attempts} "
                          f"({100*total_success/total_attempts:.1f}%)", flush=True)

    elapsed = time.time() - t0
    print(f"\n[rerender v2] === DONE ===")
    print(f"  Successful episodes: {total_success}/{total_attempts} "
          f"({100*total_success/max(total_attempts,1):.1f}%)")
    print(f"  Wall time: {elapsed:.0f}s ({elapsed/60:.1f} min)")
    print(f"  Output dataset: {out_root}")


if __name__ == "__main__":
    main()
