#!/usr/bin/env python
"""Diagnostic 2: offline (state, image) -> action regression on training episodes.

For N random episodes from the training dataset, feed each frame through the
loaded ACT policy with the SAME inference path used in eval_act_isaac_lift.py
(pre-processor -> select_action -> post-processor), and report per-dim MAE/RMSE
vs the ground-truth recorded action.

The gripper dim (index 6) is what we most care about: if RMSE there is large
(esp. around open->close transitions), temporal-ensemble smoothing is killing
the grasp. We also dump a CSV per-episode and print transition-window stats.

No Isaac Lab needed -- runs in the plain isaac_lerobot conda env.

Usage:
  conda activate isaac_lerobot
  PYTHONPATH=/Developer/lerobot/src python \\
    /Developer/omniverselab/IsaacSim/diag2_offline_regression.py \\
    --checkpoint /Developer/IsaacLab/logs/train/act_lift_front_wrist_240x320_chunk50_seed1000/checkpoints/030000/pretrained_model \\
    --dataset_root /Developer/IsaacLab/logs/lerobot_lift_front_wrist_240x320 \\
    --repo_id local/isaac_lift_front_wrist_240x320 \\
    --n_episodes 5 \\
    --device cuda:1
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import glob

import numpy as np
import pandas as pd
import torch

from lerobot.configs.policies import PreTrainedConfig
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors


def load_episode_index(dataset_root: Path) -> pd.DataFrame:
    """Read the per-episode (from_index, to_index, length) table from meta parquet."""
    files = sorted(glob.glob(str(dataset_root / "meta/episodes/chunk-*/file-*.parquet")))
    if not files:
        raise FileNotFoundError(f"No episodes parquet under {dataset_root}/meta/episodes")
    df = pd.concat([pd.read_parquet(f) for f in files], ignore_index=True)
    return df[["episode_index", "length", "dataset_from_index", "dataset_to_index"]]


def load_policy(checkpoint: Path, device: str):
    cfg = PreTrainedConfig.from_pretrained(checkpoint)
    cfg.device = device
    pre, post = make_pre_post_processors(
        cfg,
        pretrained_path=str(checkpoint),
        preprocessor_overrides={"device_processor": {"device": device}},
        postprocessor_overrides={"device_processor": {"device": "cpu"}},
    )
    policy = ACTPolicy.from_pretrained(checkpoint, config=cfg, local_files_only=True)
    policy.to(device).eval()
    return policy, pre, post, cfg


def make_obs(sample: dict, device: str, cams: list[str]) -> dict:
    """Replicate the eval-time per-frame observation dict from a dataset sample.

    LeRobotDataset returns images as float32 (C,H,W) in [0,1]; that matches the
    pre-processor's expected scale (it then applies MEAN_STD normalize)."""
    obs = {"observation.state": sample["observation.state"].to(device=device, dtype=torch.float32)}
    for c in cams:
        key = f"observation.images.{c}"
        img = sample[key]
        if not torch.is_tensor(img):
            img = torch.as_tensor(img)
        # Dataset gives (C,H,W) float in [0,1]; same as eval _rgb_to_policy_tensor output.
        obs[key] = img.to(device=device, dtype=torch.float32)
    return obs


def run_episode(policy, pre, post, dataset, ep_idx_table: pd.DataFrame, ep_idx: int, cams: list[str], device: str):
    row = ep_idx_table[ep_idx_table["episode_index"] == ep_idx].iloc[0]
    ep_from = int(row["dataset_from_index"])
    ep_to = int(row["dataset_to_index"])
    T = ep_to - ep_from
    pred_actions = np.zeros((T, 7), dtype=np.float32)
    gt_actions = np.zeros((T, 7), dtype=np.float32)
    policy.reset()
    with torch.inference_mode():
        for i in range(T):
            sample = dataset[ep_from + i]
            obs = make_obs(sample, device, cams)
            processed = pre(obs)
            action = policy.select_action(processed)
            action = post(action)
            if action.ndim == 2:
                action = action[0]
            pred_actions[i] = action.cpu().numpy()
            gt_actions[i] = sample["action"].cpu().numpy()
    return pred_actions, gt_actions


def transition_stats(gt, pred):
    """Per-dim stats restricted to frames near a gripper sign-flip."""
    grip_gt = gt[:, 6]
    flips = np.where(np.sign(grip_gt[:-1]) != np.sign(grip_gt[1:]))[0]
    if flips.size == 0:
        return None
    # union of +/- 5 frames around every flip
    mask = np.zeros(len(gt), dtype=bool)
    for f in flips:
        mask[max(0, f - 5):min(len(gt), f + 6)] = True
    mae = np.mean(np.abs(pred[mask] - gt[mask]), axis=0)
    rmse = np.sqrt(np.mean((pred[mask] - gt[mask]) ** 2, axis=0))
    return {"flip_count": int(flips.size), "window_frames": int(mask.sum()),
            "mae": mae.tolist(), "rmse": rmse.tolist()}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--checkpoint", type=Path, required=True)
    p.add_argument("--dataset_root", type=Path, required=True)
    p.add_argument("--repo_id", type=str, required=True)
    p.add_argument("--cams", type=str, default="front,wrist")
    p.add_argument("--n_episodes", type=int, default=5)
    p.add_argument("--device", type=str, default="cuda:1")
    p.add_argument("--out_dir", type=Path, default=None)
    p.add_argument("--seed", type=int, default=0)
    args = p.parse_args()

    if torch.cuda.is_available() and args.device.startswith("cuda:"):
        torch.cuda.set_device(int(args.device.split(":", 1)[1]))

    cams = [c.strip() for c in args.cams.split(",") if c.strip()]
    out_dir = args.out_dir or (args.checkpoint / "diag2_offline_regression")
    out_dir.mkdir(parents=True, exist_ok=True)

    policy, pre, post, cfg = load_policy(args.checkpoint, args.device)
    print(f"[diag2] policy chunk_size={cfg.chunk_size} n_action_steps={cfg.n_action_steps}"
          f" temporal_ensemble_coeff={cfg.temporal_ensemble_coeff}")

    dataset = LeRobotDataset(repo_id=args.repo_id, root=args.dataset_root)
    ep_index_table = load_episode_index(args.dataset_root)
    n_ep = min(args.n_episodes, dataset.num_episodes)
    rng = np.random.default_rng(args.seed)
    ep_ids = sorted(rng.choice(dataset.num_episodes, n_ep, replace=False).tolist())
    print(f"[diag2] dataset episodes={dataset.num_episodes} chosen={ep_ids}")

    dim_names = ["dx", "dy", "dz", "droll", "dpitch", "dyaw", "grip"]
    all_pred = []
    all_gt = []
    per_ep_summary = []
    for ep in ep_ids:
        pred, gt = run_episode(policy, pre, post, dataset, ep_index_table, ep, cams, args.device)
        all_pred.append(pred)
        all_gt.append(gt)
        mae = np.mean(np.abs(pred - gt), axis=0)
        rmse = np.sqrt(np.mean((pred - gt) ** 2, axis=0))
        ts = transition_stats(gt, pred)
        per_ep_summary.append({"episode": int(ep), "T": int(len(pred)),
                               "mae": mae.tolist(), "rmse": rmse.tolist(),
                               "transition": ts})
        print(f"[diag2] ep {ep:03d} T={len(pred):3d} "
              f"per-dim MAE=[{', '.join(f'{x:.3f}' for x in mae)}]")
        if ts is not None:
            print(f"        gripper-flip window ({ts['window_frames']} frames over "
                  f"{ts['flip_count']} flips): gripRMSE={ts['rmse'][6]:.3f}")
        # Per-episode CSV with full traces
        csv_path = out_dir / f"ep_{ep:03d}_trace.csv"
        with csv_path.open("w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["t"] + [f"gt_{n}" for n in dim_names]
                       + [f"pred_{n}" for n in dim_names])
            for t in range(len(pred)):
                w.writerow([t] + gt[t].tolist() + pred[t].tolist())
        print(f"        wrote {csv_path}")

    all_pred = np.concatenate(all_pred, axis=0)
    all_gt = np.concatenate(all_gt, axis=0)
    mae = np.mean(np.abs(all_pred - all_gt), axis=0)
    rmse = np.sqrt(np.mean((all_pred - all_gt) ** 2, axis=0))
    print("\n[diag2] === OVERALL ===")
    for n, m, r in zip(dim_names, mae, rmse):
        print(f"  {n:6s}  MAE={m:.4f}  RMSE={r:.4f}")

    # Special focus: gripper sign agreement and lag
    grip_gt = all_gt[:, 6]
    grip_pred = all_pred[:, 6]
    sign_match = float(np.mean(np.sign(grip_gt) == np.sign(grip_pred)))
    close_when_gt_close = float(np.mean(grip_pred[grip_gt < 0] < 0))
    print(f"\n[diag2] gripper sign agreement = {sign_match * 100:.1f}%")
    print(f"[diag2] grip predicted < 0 when GT < 0 = {close_when_gt_close * 100:.1f}%")
    print(f"[diag2] grip pred range = [{grip_pred.min():.3f}, {grip_pred.max():.3f}]"
          f"   (GT range = [{grip_gt.min():.3f}, {grip_gt.max():.3f}])")

    summary = {
        "checkpoint": str(args.checkpoint),
        "dataset_root": str(args.dataset_root),
        "cams": cams,
        "ep_ids": ep_ids,
        "dim_names": dim_names,
        "overall_mae": mae.tolist(),
        "overall_rmse": rmse.tolist(),
        "gripper_sign_agreement": sign_match,
        "gripper_close_recall": close_when_gt_close,
        "gripper_pred_range": [float(grip_pred.min()), float(grip_pred.max())],
        "gripper_gt_range": [float(grip_gt.min()), float(grip_gt.max())],
        "per_episode": per_ep_summary,
        "policy_cfg": {
            "chunk_size": cfg.chunk_size,
            "n_action_steps": cfg.n_action_steps,
            "temporal_ensemble_coeff": cfg.temporal_ensemble_coeff,
        },
    }
    summary_path = out_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2))
    print(f"\n[diag2] wrote {summary_path}")


if __name__ == "__main__":
    main()
