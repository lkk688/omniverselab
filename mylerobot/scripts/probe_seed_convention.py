#!/usr/bin/env python
"""Probe whether `aloha_sim_insertion_scripted` episode N was recorded with seed=N.

Logic: the only random variable per-episode is the initial BOX_POSE (set
inside `AlohaEnv.reset()` from `sample_insertion_pose(seed)`). If we reset
the env with the right seed, the rendered first frame should be
pixel-identical (or very nearly so) to the dataset's first frame.

We probe by:
  1) Loading the dataset and pulling the first `observation.images.top`
     for a few sampled episodes (default: 0, 5, 25, 49).
  2) For each probe episode, trying seeds 0..149 in our multicam env,
     rendering the `top` camera, and computing per-pixel MSE.
  3) Reporting the best-match seed per episode and whether the
     "seed == episode_index" hypothesis holds.

If the hypothesis holds, we can replay scripted actions through the
multi-cam env to manufacture a 4-cam version of the scripted dataset
without porting any policy code.
"""

from __future__ import annotations

# Shim must come first.
import mylerobot  # noqa: F401

import argparse
import os
from pathlib import Path

import numpy as np
import torch

os.environ.setdefault("MUJOCO_GL", "egl")

from lerobot.datasets.lerobot_dataset import LeRobotDataset
from mylerobot.sim.aloha_multicam import AlohaMultiCamEnv


def _frame0_top(dataset: LeRobotDataset, episode_idx: int) -> np.ndarray:
    """Get observation.images.top of the first frame of `episode_idx`."""
    # Current lerobot API: ds.meta.episodes[ep] has dataset_from_index/to_index.
    ep_meta = dataset.meta.episodes[episode_idx]
    start = int(ep_meta["dataset_from_index"])
    item = dataset[start]
    img = item["observation.images.top"]
    if isinstance(img, torch.Tensor):
        img = img.detach().cpu().numpy()
    # The dataset returns float32 in [0,1], CHW. Convert to HWC uint8.
    if img.dtype != np.uint8:
        if img.max() <= 1.0 + 1e-6:
            img = (img * 255.0).clip(0, 255).astype(np.uint8)
        else:
            img = img.astype(np.uint8)
    if img.shape[0] in (1, 3) and img.shape[0] < img.shape[-1]:
        img = np.transpose(img, (1, 2, 0))  # CHW -> HWC
    return img


def _env_frame0_top(env: AlohaMultiCamEnv, seed: int) -> np.ndarray:
    env.reset(seed=seed)
    return env.render_camera("top")


def mse(a: np.ndarray, b: np.ndarray) -> float:
    a = a.astype(np.float32)
    b = b.astype(np.float32)
    if a.shape != b.shape:
        return float("inf")
    return float(np.mean((a - b) ** 2))


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--dataset", default="lerobot/aloha_sim_insertion_scripted")
    p.add_argument("--probe_episodes", type=int, nargs="+", default=[0, 5, 25, 49])
    p.add_argument("--max_seed", type=int, default=149)
    p.add_argument("--obs_h", type=int, default=480)
    p.add_argument("--obs_w", type=int, default=640)
    p.add_argument("--save_compare", type=Path, default=None,
                   help="If set, save side-by-side {dataset, best-seed-env} png per probe to this dir.")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    print(f"[probe] dataset       = {args.dataset}")
    print(f"[probe] probe episodes = {args.probe_episodes}")
    print(f"[probe] seeds tried   = 0..{args.max_seed}")

    print("[probe] loading dataset ...")
    ds = LeRobotDataset(args.dataset, video_backend="pyav")
    print(f"[probe] dataset has {ds.num_episodes} episodes, {len(ds)} frames")

    print("[probe] creating multicam env ...")
    env = AlohaMultiCamEnv(
        task="insertion",
        observation_height=args.obs_h,
        observation_width=args.obs_w,
    )

    # Cache env renders for each seed once (reused across probe episodes).
    print(f"[probe] rendering env at seeds 0..{args.max_seed} ...")
    seed_imgs: dict[int, np.ndarray] = {}
    for s in range(args.max_seed + 1):
        seed_imgs[s] = _env_frame0_top(env, s)
        if (s + 1) % 25 == 0:
            print(f"  ... rendered seed {s+1}/{args.max_seed+1}", flush=True)

    print("\n[probe] === RESULTS ===")
    hypothesis_holds = True
    for ep in args.probe_episodes:
        ref_img = _frame0_top(ds, ep)
        scores = [(s, mse(ref_img, img)) for s, img in seed_imgs.items()]
        scores.sort(key=lambda x: x[1])
        best5 = scores[:5]
        best_seed, best_mse = best5[0]
        seed_eq_ep_mse = dict(scores)[ep] if ep <= args.max_seed else float("inf")

        match_flag = "✓" if best_seed == ep else "✗"
        print(f"  ep={ep:3d}  best_seed={best_seed:3d}  best_mse={best_mse:8.2f}  "
              f"mse@seed={ep:3d}: {seed_eq_ep_mse:8.2f}  hypothesis={match_flag}")
        # show next 4 close matches for context
        for s, m in best5[1:]:
            print(f"           runner-up seed={s:3d}  mse={m:8.2f}")
        if best_seed != ep:
            hypothesis_holds = False

        if args.save_compare is not None:
            args.save_compare.mkdir(parents=True, exist_ok=True)
            try:
                from PIL import Image
                side = np.concatenate([ref_img, seed_imgs[best_seed]], axis=1)
                Image.fromarray(side).save(args.save_compare / f"ep{ep:03d}_vs_seed{best_seed:03d}.png")
            except ImportError:
                print("  (PIL not installed, skipping image save)")

    env.close()

    print("\n[probe] === CONCLUSION ===")
    if hypothesis_holds:
        print("  ✓✓✓  seed == episode_index hypothesis holds for ALL probe episodes.")
        print("       Safe to replay scripted actions in multicam env using seed=ep_idx.")
    else:
        print("  ✗✗✗  hypothesis FAILED on at least one probe.")
        print("       Need a different data-generation strategy (e.g. port scripted policy).")
        # Heuristic: if best seeds form an offset, report it
        offsets = []
        for ep in args.probe_episodes:
            ref_img = _frame0_top(ds, ep)
            scores = [(s, mse(ref_img, img)) for s, img in seed_imgs.items()]
            scores.sort(key=lambda x: x[1])
            offsets.append(scores[0][0] - ep)
        if len(set(offsets)) == 1:
            print(f"       But all probes match with constant offset = {offsets[0]}. "
                  f"Try seed = episode_index + {offsets[0]}.")


if __name__ == "__main__":
    main()
