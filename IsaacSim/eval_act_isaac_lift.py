#!/usr/bin/env python
"""Online IsaacLab rollout evaluation for a LeRobot ACT checkpoint.

This evaluates the Franka IK-relative cube-lift policy trained from the
two-camera Isaac dataset:

  observation.images.front
  observation.images.wrist
  observation.state
  action

Run from IsaacLab, for example:

  ./isaaclab.sh -p /Developer/omniverselab/IsaacSim/eval_act_isaac_lift.py \
      --device cuda:1 --enable_cameras --headless \
      --checkpoint /Developer/IsaacLab/logs/train/.../checkpoints/030000/pretrained_model
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from isaaclab.app import AppLauncher


@dataclass(frozen=True)
class EnvPreset:
    task_id: str
    pick_key: str = "object"
    robot_key: str = "robot"
    ee_body: str = "panda_hand"
    action_dim: int = 7


SUPPORTED_ENVS = {
    "lift-ik-rel": EnvPreset(task_id="Isaac-Lift-Cube-Franka-IK-Rel-v0"),
}


def _parse_hw(text: str) -> tuple[int, int]:
    parts = [int(x.strip()) for x in text.split(",")]
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("expected H,W")
    return parts[0], parts[1]


parser = argparse.ArgumentParser(description="Evaluate a two-camera ACT policy in IsaacLab lift.")
parser.add_argument("--checkpoint", type=Path, required=True, help="Path to .../pretrained_model")
parser.add_argument("--env", choices=SUPPORTED_ENVS.keys(), default="lift-ik-rel")
parser.add_argument("--n_episodes", type=int, default=10)
parser.add_argument("--max_steps", type=int, default=320)
parser.add_argument("--seed", type=int, default=1000)
parser.add_argument("--cams", type=str, default="front,wrist")
parser.add_argument("--cam_hw", type=_parse_hw, default=(480, 640), help="Render H,W for Isaac cameras.")
parser.add_argument("--policy_hw", type=_parse_hw, default=(240, 320), help="Resize H,W for ACT input.")
parser.add_argument("--success_z", type=float, default=0.06, help="World z threshold for cube centre.")
parser.add_argument("--success_delta_z", type=float, default=0.0, help="Minimum lift above reset z.")
parser.add_argument("--success_hold_steps", type=int, default=10)
parser.add_argument("--action_clip", type=float, default=1.0)
parser.add_argument("--log_every", type=int, default=50)
parser.add_argument("--output", type=Path, default=None)
parser.add_argument("--gripper_mode", choices=["raw", "snap"], default="raw",
                    help="raw=use policy's continuous grip output as-is; "
                         "snap=replace with sign(grip) before sending to env.")
parser.add_argument("--disable_te", action="store_true",
                    help="Disable ACT temporal ensembling at inference (use action chunks).")
parser.add_argument("--n_action_steps", type=int, default=0,
                    help="If >0 and --disable_te, run open-loop chunks of this length.")
parser.add_argument("--trace_actions", action="store_true",
                    help="Write per-step predicted actions and cube z to <output>.trace.csv.")
parser.add_argument("--include_ee_pose", action="store_true", default=True,
                    help="Append EE [pos(3), quat_wxyz(4)] to obs/state (must match training).")
parser.add_argument("--no_include_ee_pose", dest="include_ee_pose", action="store_false")
parser.add_argument("--include_cube_pose", action="store_true", default=False,
                    help="Append cube_pos (3) to obs/state — cheat baseline. Must match training.")
parser.add_argument("--bev_perception_checkpoint", type=Path, default=None,
                    help="Optional ACT-BEV checkpoint to run as a perception drop-in: its aux "
                         "head's soft-argmax cube XY replaces the cube_x,cube_y dims in state. "
                         "Requires --include_cube_pose to wire those dims into the main policy.")
parser.add_argument("--bev_cams", type=str, default="top,front",
                    help="Comma-separated cam names fed into the BEV perception module.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()


app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


import numpy as np  # noqa: E402
import torch  # noqa: E402
import torch.nn.functional as F  # noqa: E402

sys.path.insert(0, "/Developer/omniverselab")
sys.path.insert(0, "/Developer/omniverselab/mylerobot")
sys.path.insert(0, "/Developer/lerobot/src")

import mylerobot  # noqa: E402,F401
import isaac_multicam_addons as mc  # noqa: E402
from isaaclab.envs import ManagerBasedRLEnv  # noqa: E402
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg  # noqa: E402
from lerobot.configs.policies import PreTrainedConfig  # noqa: E402
from lerobot.policies.factory import make_pre_post_processors, get_policy_class  # noqa: E402


def _cuda_index(device: str) -> Optional[int]:
    if not device.startswith("cuda"):
        return None
    if ":" not in device:
        return None
    return int(device.split(":", 1)[1])


def _load_policy(checkpoint: Path, device: str):
    idx = _cuda_index(device)
    if idx is not None and torch.cuda.is_available():
        torch.cuda.set_device(idx)

    cfg = PreTrainedConfig.from_pretrained(checkpoint)
    cfg.device = device
    if args_cli.disable_te and hasattr(cfg, "temporal_ensemble_coeff"):
        cfg.temporal_ensemble_coeff = None
        if args_cli.n_action_steps > 0:
            cfg.n_action_steps = args_cli.n_action_steps
        elif cfg.n_action_steps <= 1:
            cfg.n_action_steps = min(20, cfg.chunk_size)
        print(f"[eval] TE disabled; n_action_steps={cfg.n_action_steps}, chunk_size={cfg.chunk_size}")
    pre, post = make_pre_post_processors(
        cfg,
        pretrained_path=str(checkpoint),
        preprocessor_overrides={"device_processor": {"device": device}},
        postprocessor_overrides={"device_processor": {"device": "cpu"}},
    )
    # Resolve policy class. lerobot's get_policy_class doesn't know about
    # mylerobot's subclasses (act_lang, act_bev), so dispatch them here.
    if cfg.type == "act_bev":
        from mylerobot.policies.act_bev.modeling_act_bev import ACTBEVPolicy
        policy_cls = ACTBEVPolicy
    elif cfg.type == "act_cheat_noisy":
        from mylerobot.policies.act_cheat_noisy.modeling_act_cheat_noisy import ACTCheatNoisyPolicy
        policy_cls = ACTCheatNoisyPolicy
    elif cfg.type == "act_lang":
        from mylerobot.policies.act_lang.modeling import ACTLangPolicy
        policy_cls = ACTLangPolicy
    else:
        policy_cls = get_policy_class(cfg.type)
    policy = policy_cls.from_pretrained(checkpoint, config=cfg, local_files_only=True)
    policy.to(device).eval()
    return policy, pre, post


def _load_bev_perception(checkpoint: Path, device: str):
    """Load an ACT-BEV checkpoint and return just the BEV encoder + aux head
    plus a function that maps (B, N, 3, H, W) images to predicted (cube_x, cube_y)."""
    from mylerobot.policies.act_bev.modeling_act_bev import ACTBEVPolicy
    cfg = PreTrainedConfig.from_pretrained(checkpoint)
    cfg.device = device
    policy = ACTBEVPolicy.from_pretrained(checkpoint, config=cfg, local_files_only=True)
    policy.to(device).eval()
    bev_K = policy.model.bev_K
    bev_E = policy.model.bev_E
    bev_cam_keys = list(policy.config.bev_cam_keys)

    def predict_xy(image_dict: dict[str, torch.Tensor]) -> torch.Tensor:
        imgs = torch.stack([image_dict[k] for k in bev_cam_keys], dim=1)  # (B, N, 3, H, W)
        B = imgs.shape[0]
        K = bev_K.unsqueeze(0).expand(B, -1, -1, -1)
        E = bev_E.unsqueeze(0).expand(B, -1, -1, -1)
        with torch.inference_mode():
            bev_out = policy.model.bev_encoder(imgs, K, E)
            xy = policy.model._bev_centroid_softargmax(bev_out["bev"])  # (B, 2) world frame
        return xy[0]

    return policy, predict_xy, bev_cam_keys


def _rgb_to_policy_tensor(rgb, device: str, policy_hw: tuple[int, int]) -> torch.Tensor:
    if not torch.is_tensor(rgb):
        rgb = torch.as_tensor(rgb)
    rgb = rgb[..., :3]
    x = rgb.to(device=device, dtype=torch.float32)
    if x.max() > 2.0:
        x = x / 255.0
    x = x.permute(2, 0, 1).contiguous()
    if tuple(x.shape[-2:]) != tuple(policy_hw):
        x = F.interpolate(
            x.unsqueeze(0),
            size=policy_hw,
            mode="bilinear",
            align_corners=False,
        ).squeeze(0)
    return x


def _make_observation(env, robot, cam_names: list[str], device: str, policy_hw: tuple[int, int],
                      ee_idx: Optional[int] = None, cube_obj=None,
                      cube_xy_override: Optional[torch.Tensor] = None):
    cam_records = mc.capture_all_cams(env, cam_names)
    missing = [c for c in cam_names if c not in cam_records]
    if missing:
        raise RuntimeError(f"Missing camera records: {missing}. Available: {sorted(cam_records)}")

    state_parts = [robot.data.joint_pos[0]]
    if args_cli.include_ee_pose and ee_idx is not None:
        state_parts.append(robot.data.body_pos_w[0, ee_idx])
        state_parts.append(robot.data.body_quat_w[0, ee_idx])
    if args_cli.include_cube_pose and cube_obj is not None:
        cube_pos = cube_obj.data.root_pos_w[0].clone()
        if cube_xy_override is not None:
            cube_pos[0] = cube_xy_override[0].to(cube_pos.device, dtype=cube_pos.dtype)
            cube_pos[1] = cube_xy_override[1].to(cube_pos.device, dtype=cube_pos.dtype)
        state_parts.append(cube_pos)
    state = torch.cat(state_parts)
    obs = {
        "observation.state": state.to(device=device, dtype=torch.float32).clone(),
    }
    for cam in cam_names:
        obs[f"observation.images.{cam}"] = _rgb_to_policy_tensor(
            cam_records[cam]["rgb"], device=device, policy_hw=policy_hw
        )
    return obs


def _policy_action(policy, pre, post, raw_obs: dict, env_device: str, action_dim: int) -> torch.Tensor:
    processed = pre(raw_obs)
    with torch.inference_mode():
        action = policy.select_action(processed)
    action = post(action)
    if action.ndim == 2:
        action = action[0]
    action = action.to(device=env_device, dtype=torch.float32)
    if args_cli.action_clip > 0:
        action = torch.clamp(action, -args_cli.action_clip, args_cli.action_clip)
    if args_cli.gripper_mode == "snap" and action.shape[0] >= 7:
        action[6] = torch.sign(action[6])
    if action.shape[0] < action_dim:
        padded = torch.zeros(action_dim, device=env_device, dtype=torch.float32)
        padded[: action.shape[0]] = action
        action = padded
    return action[:action_dim].unsqueeze(0)


def _render_once(env):
    try:
        env.sim.render()
    except Exception:
        pass


def main() -> None:
    preset = SUPPORTED_ENVS[args_cli.env]
    device = args_cli.device
    cam_names = [c.strip() for c in args_cli.cams.split(",") if c.strip()]
    print(f"[eval] using cameras: {cam_names}")

    np.random.seed(args_cli.seed)
    torch.manual_seed(args_cli.seed)

    print(f"[eval] checkpoint    = {args_cli.checkpoint}")
    print(f"[eval] task          = {preset.task_id}")
    print(f"[eval] device        = {device}")
    print(f"[eval] cams          = {cam_names}")
    print(f"[eval] cam_hw        = {args_cli.cam_hw}")
    print(f"[eval] policy_hw     = {args_cli.policy_hw}")
    print(f"[eval] episodes      = {args_cli.n_episodes}")
    print(f"[eval] max_steps     = {args_cli.max_steps}")

    policy, pre, post = _load_policy(args_cli.checkpoint, device)
    print(f"[eval] loaded policy = {type(policy).__name__}")

    bev_predict_xy = None
    bev_only_cams: list[str] = []
    if args_cli.bev_perception_checkpoint is not None:
        if not args_cli.include_cube_pose:
            raise RuntimeError(
                "--bev_perception_checkpoint requires --include_cube_pose so the "
                "predicted XY can replace the cube dims in the main policy's state."
            )
        _bev_policy, bev_predict_xy, bev_keys = _load_bev_perception(
            args_cli.bev_perception_checkpoint, device
        )
        print(f"[eval] BEV perception loaded; cams = {bev_keys}")
        # Ensure Isaac renders any BEV cams not already in the main capture set.
        for k in bev_keys:
            short = k.split(".")[-1]
            if short not in cam_names:
                bev_only_cams.append(short)
                print(f"[eval] adding BEV-only cam '{short}' to render set")
    render_cam_names = cam_names + bev_only_cams

    env_cfg = parse_env_cfg(preset.task_id, device=device, use_fabric=False)
    env_cfg.seed = args_cli.seed
    env_cfg.scene.num_envs = 1
    env_cfg.episode_length_s = max(30.0, args_cli.max_steps / 50.0 + 2.0)
    mc.add_multicam_to_scene_cfg(env_cfg, render_cam_names, args_cli.cam_hw[0], args_cli.cam_hw[1])

    env = ManagerBasedRLEnv(cfg=env_cfg)
    robot = env.scene[preset.robot_key]
    ee_idx = robot.find_bodies(preset.ee_body)[0][0]
    action_dim = env.action_manager.total_action_dim
    if action_dim != preset.action_dim:
        print(f"[eval] WARNING: env action_dim={action_dim}, preset={preset.action_dim}")
    print(f"[eval] env action_dim = {action_dim}")

    episodes: list[dict] = []
    trace_rows: list[list] = []
    t0 = time.time()
    try:
        for ep_idx in range(args_cli.n_episodes):
            env.reset(seed=args_cli.seed + ep_idx)
            policy.reset()
            _render_once(env)
            cube = env.scene[preset.pick_key]
            start_z = float(cube.data.root_pos_w[0, 2].item())
            max_z = start_z
            hold = 0
            success = False
            steps_taken = 0
            last_reward = 0.0
            done_reason = "max_steps"

            for step in range(args_cli.max_steps):
                bev_xy = None
                if bev_predict_xy is not None:
                    # First capture the camera images, then run BEV perception, then
                    # build the main-policy obs with the predicted XY overriding cube_pose.
                    cam_records_for_bev = mc.capture_all_cams(env, render_cam_names)
                    img_dict = {
                        f"observation.images.{c}": _rgb_to_policy_tensor(
                            cam_records_for_bev[c]["rgb"], device=device, policy_hw=args_cli.policy_hw
                        ).unsqueeze(0)
                        for c in cam_records_for_bev
                    }
                    bev_xy = bev_predict_xy(img_dict)
                raw_obs = _make_observation(env, robot, cam_names, device, args_cli.policy_hw,
                                            ee_idx=ee_idx, cube_obj=cube,
                                            cube_xy_override=bev_xy)
                action = _policy_action(policy, pre, post, raw_obs, env.device, action_dim)
                if args_cli.trace_actions:
                    a = action[0].detach().cpu().tolist()
                    cube_xyz = cube.data.root_pos_w[0].detach().cpu().tolist()
                    ee_pos = robot.data.body_pos_w[0, ee_idx].detach().cpu().tolist()
                    trace_rows.append([ep_idx, step, *cube_xyz, *ee_pos, *a])
                _, rewards, dones, truncated, infos = env.step(action)
                steps_taken = step + 1

                cube_z = float(cube.data.root_pos_w[0, 2].item())
                max_z = max(max_z, cube_z)
                last_reward = float(rewards[0].item()) if torch.is_tensor(rewards) else float(rewards[0])
                lifted = (
                    cube_z >= args_cli.success_z
                    and (cube_z - start_z) >= args_cli.success_delta_z
                )
                hold = hold + 1 if lifted else 0
                if hold >= args_cli.success_hold_steps:
                    success = True
                    done_reason = "success_hold"
                    break

                done = bool(dones[0].item()) if torch.is_tensor(dones) else bool(dones[0])
                trunc = bool(truncated[0].item()) if torch.is_tensor(truncated) else bool(truncated[0])
                if done or trunc:
                    done_reason = "env_done" if done else "env_truncated"
                    break

                if args_cli.log_every > 0 and (step + 1) % args_cli.log_every == 0:
                    print(
                        f"[eval] ep {ep_idx + 1:02d}/{args_cli.n_episodes} "
                        f"step={step + 1:03d} cube_z={cube_z:.3f} max_z={max_z:.3f} "
                        f"hold={hold} reward={last_reward:.3f}",
                        flush=True,
                    )

            ep = {
                "episode": ep_idx,
                "success": bool(success),
                "steps": int(steps_taken),
                "start_cube_z": start_z,
                "max_cube_z": max_z,
                "lift_delta_z": max_z - start_z,
                "last_reward": last_reward,
                "done_reason": done_reason,
            }
            episodes.append(ep)
            rate = sum(e["success"] for e in episodes) / len(episodes)
            print(
                f"[eval] ep {ep_idx + 1:02d}/{args_cli.n_episodes} "
                f"success={success} steps={ep['steps']} max_z={max_z:.3f} "
                f"delta_z={ep['lift_delta_z']:.3f} running_success={rate * 100:.1f}%",
                flush=True,
            )
    finally:
        env.close()

    successes = [e["success"] for e in episodes]
    summary = {
        "checkpoint": str(args_cli.checkpoint),
        "env": args_cli.env,
        "task_id": preset.task_id,
        "device": device,
        "cams": cam_names,
        "cam_hw": list(args_cli.cam_hw),
        "policy_hw": list(args_cli.policy_hw),
        "n_episodes": args_cli.n_episodes,
        "max_steps": args_cli.max_steps,
        "seed": args_cli.seed,
        "success_z": args_cli.success_z,
        "success_delta_z": args_cli.success_delta_z,
        "success_hold_steps": args_cli.success_hold_steps,
        "success_rate": float(np.mean(successes)) if successes else 0.0,
        "success_count": int(sum(successes)),
        "avg_steps": float(np.mean([e["steps"] for e in episodes])) if episodes else 0.0,
        "avg_max_cube_z": float(np.mean([e["max_cube_z"] for e in episodes])) if episodes else 0.0,
        "avg_lift_delta_z": float(np.mean([e["lift_delta_z"] for e in episodes])) if episodes else 0.0,
        "wall_time_s": time.time() - t0,
        "episodes": episodes,
    }

    out = args_cli.output
    if out is None:
        out = args_cli.checkpoint / f"eval_isaac_lift_front_wrist_n{args_cli.n_episodes}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    if args_cli.trace_actions and trace_rows:
        import csv as _csv
        trace_path = out.with_suffix(".trace.csv")
        with trace_path.open("w", newline="") as fh:
            w = _csv.writer(fh)
            w.writerow(["ep", "step", "cube_x", "cube_y", "cube_z",
                        "ee_x", "ee_y", "ee_z",
                        "a_dx", "a_dy", "a_dz", "a_droll", "a_dpitch", "a_dyaw", "a_grip"])
            w.writerows(trace_rows)
        print(f"[eval] wrote trace = {trace_path}")

    print("\n[eval] === SUMMARY ===")
    print(f"[eval] success rate = {summary['success_rate'] * 100:.1f}% "
          f"({summary['success_count']}/{args_cli.n_episodes})")
    print(f"[eval] avg steps    = {summary['avg_steps']:.1f}")
    print(f"[eval] avg max z    = {summary['avg_max_cube_z']:.3f}")
    print(f"[eval] avg delta z  = {summary['avg_lift_delta_z']:.3f}")
    print(f"[eval] wall time    = {summary['wall_time_s']:.1f}s")
    print(f"[eval] wrote        = {out}")
    simulation_app.close()


if __name__ == "__main__":
    main()
