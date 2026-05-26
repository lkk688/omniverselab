"""Diagnostic: does the ACT-BEV aux head actually predict cube location?

Loads the trained checkpoint, runs it on a few training batches, and prints:
  - aux loss value
  - peak of predicted heatmap (predicted cube voxel)
  - true cube voxel (from GT cube_pose)
  - L2 error between peak-voxel and GT-voxel in metres

If the predicted-peak tracks the GT-peak, the BEV+aux is learning correctly,
and the failure is at the transformer/action-head level. If not, the BEV
encoder isn't extracting cube position from the cameras.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, "/Developer/omniverselab")
sys.path.insert(0, "/Developer/omniverselab/mylerobot")
sys.path.insert(0, "/Developer/lerobot/src")

import argparse

import numpy as np
import pandas as pd
import torch

from lerobot.configs.policies import PreTrainedConfig
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.policies.factory import make_pre_post_processors

from mylerobot.policies.act_bev.aux_head import build_target_heatmap, cube_heatmap_loss
from mylerobot.policies.act_bev.modeling_act_bev import ACTBEVPolicy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", type=Path, required=True)
    ap.add_argument("--dataset_root", type=Path, required=True)
    ap.add_argument("--repo_id", type=str, required=True)
    ap.add_argument("--device", type=str, default="cuda:1")
    ap.add_argument("--n_samples", type=int, default=10)
    args = ap.parse_args()

    torch.cuda.set_device(int(args.device.split(":")[1]))
    cfg = PreTrainedConfig.from_pretrained(args.checkpoint)
    cfg.device = args.device

    pre, _ = make_pre_post_processors(
        cfg,
        pretrained_path=str(args.checkpoint),
        preprocessor_overrides={"device_processor": {"device": args.device}},
        postprocessor_overrides={"device_processor": {"device": "cpu"}},
    )
    policy = ACTBEVPolicy.from_pretrained(args.checkpoint, config=cfg, local_files_only=True)
    policy.to(args.device).eval()
    print(f"[diag] cube key = {cfg.aux_cube_key}")
    print(f"[diag] voxel ranges x={cfg.voxel_x_range} y={cfg.voxel_y_range} step={cfg.voxel_size}")

    dataset = LeRobotDataset(repo_id=args.repo_id, root=args.dataset_root)

    rng = np.random.default_rng(0)
    idx_list = rng.choice(len(dataset), args.n_samples, replace=False).tolist()
    print(f"[diag] sampling frames: {idx_list[:6]}…")

    nx = int(round((cfg.voxel_x_range[1] - cfg.voxel_x_range[0]) / cfg.voxel_size))
    ny = int(round((cfg.voxel_y_range[1] - cfg.voxel_y_range[0]) / cfg.voxel_size))

    aux_losses = []
    peak_errors_m = []
    for idx in idx_list:
        sample = dataset[idx]
        # Build a batch dict
        batch = {}
        for k in [
            "observation.state",
            "observation.images.top",
            "observation.images.front",
            "observation.images.wrist",
            "observation.cube_pose",
        ]:
            if k in sample:
                v = sample[k]
                if not torch.is_tensor(v):
                    v = torch.as_tensor(v)
                batch[k] = v.unsqueeze(0).to(args.device)
        batch = pre(batch)

        with torch.inference_mode():
            # Inspect each stage of the BEV encoder explicitly
            enc = policy.model.bev_encoder
            imgs = torch.stack([batch[k] for k in policy.config.bev_cam_keys], dim=1)
            B, N = imgs.shape[:2]
            K = policy.model.bev_K.unsqueeze(0).expand(B, -1, -1, -1).to(imgs.device)
            E = policy.model.bev_E.unsqueeze(0).expand(B, -1, -1, -1).to(imgs.device)
            print(f"    img stats: mean={imgs.mean():.4f} std={imgs.std():.4f}")
            feat = enc.backbone(imgs.reshape(B * N, *imgs.shape[2:]))
            feat = enc.feat_proj(feat)
            print(f"    backbone feat stats: shape={tuple(feat.shape)} mean={feat.mean():.4f} std={feat.std():.4f}")
            uv, vis = enc._project_voxels(K, E)
            print(f"    visibility: total_vis_voxels={vis.sum().item():.0f} "
                  f"(per cam {[float(vis[0,i].sum()) for i in range(N)]})")
            _, _, bev_feat = policy.model(batch)
            pred_logits = policy.model.aux_head(bev_feat)
            pred = torch.sigmoid(pred_logits)[0].cpu().numpy()
            bf = bev_feat[0]
            print(f"    BEV feat stats: shape={tuple(bf.shape)} mean={bf.mean():.4f} "
                  f"std={bf.std():.4f} max={bf.max():.4f}")

        cube_pose = batch[cfg.aux_cube_key][0].cpu().numpy()
        tgt = build_target_heatmap(
            batch[cfg.aux_cube_key],
            cfg.voxel_x_range, cfg.voxel_y_range, cfg.voxel_size, cfg.aux_heatmap_sigma_m,
        )[0].cpu().numpy()
        aux = cube_heatmap_loss(pred_logits.cpu(), torch.from_numpy(tgt[None])).item()

        # Peak of predicted heatmap → voxel coord → world XY
        pi, pj = np.unravel_index(pred.argmax(), pred.shape)
        pred_x = cfg.voxel_x_range[0] + (pi + 0.5) * cfg.voxel_size
        pred_y = cfg.voxel_y_range[0] + (pj + 0.5) * cfg.voxel_size
        err = float(np.hypot(pred_x - cube_pose[0], pred_y - cube_pose[1]))
        aux_losses.append(aux)
        peak_errors_m.append(err)
        print(f"  idx={idx:4d}  cube=({cube_pose[0]:+.3f},{cube_pose[1]:+.3f}) "
              f"pred_peak=({pred_x:+.3f},{pred_y:+.3f}) "
              f"err={err*100:5.2f} cm   aux_loss={aux:.4f}   "
              f"pred_max={pred.max():.3f} pred_mean={pred.mean():.3f}")

    print(f"\n[diag] mean aux loss = {np.mean(aux_losses):.4f}")
    print(f"[diag] mean peak-XY error = {np.mean(peak_errors_m)*100:.2f} cm")
    print(f"[diag] median peak-XY error = {np.median(peak_errors_m)*100:.2f} cm")


if __name__ == "__main__":
    main()
