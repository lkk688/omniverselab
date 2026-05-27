#!/usr/bin/env python
"""Generate side-by-side sample video clips from the collected HDF5 demos.

Reads an `isaac_auto_collector_v6` HDF5 directory, picks a few episodes,
stitches all cameras horizontally per frame (in the order given by --cams),
overlays metadata (episode #, frame index, action, cube xyz, robot joint_pos
summary), and writes per-episode MP4s.

Usage:
  conda activate isaac_lerobot
  python /Developer/omniverselab/IsaacSim/make_sample_clips.py \\
    --input_dir /Developer/IsaacLab/logs/demos_lift_tfw_cube_n200 \\
    --output_dir /tmp/sample_clips_tfw \\
    --cams top,front,wrist \\
    --n_episodes 5 \\
    --fps 50
"""

from __future__ import annotations

import argparse
import glob
import json
import os
from pathlib import Path

import h5py
import imageio.v3 as iio
import numpy as np
from PIL import Image, ImageDraw, ImageFont


def _try_load_font(size: int = 14) -> ImageFont.ImageFont:
    """Try to grab a TrueType font that's likely installed; fall back to default."""
    for candidate in (
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/TTF/DejaVuSansMono.ttf",
    ):
        if os.path.exists(candidate):
            try:
                return ImageFont.truetype(candidate, size=size)
            except Exception:
                pass
    return ImageFont.load_default()


def _episode_files(input_dir: Path) -> list[Path]:
    files = sorted(glob.glob(str(input_dir / "*.hdf5"))) + sorted(
        glob.glob(str(input_dir / "*.h5"))
    )
    if not files:
        raise FileNotFoundError(f"No .hdf5/.h5 files under {input_dir}")
    return [Path(f) for f in files]


def _load_metadata(h5: h5py.File) -> dict:
    raw = h5.attrs.get("metadata")
    if raw is None:
        return {}
    if isinstance(raw, bytes):
        raw = raw.decode("utf-8")
    try:
        return json.loads(str(raw))
    except Exception:
        return {}


def _stitch_row(frames: list[np.ndarray], pad: int = 4) -> np.ndarray:
    """Stitch a list of HxWx3 uint8 frames horizontally with a thin black gap."""
    H = max(f.shape[0] for f in frames)
    parts = []
    for i, f in enumerate(frames):
        if f.shape[0] != H:
            # pad to common height
            pad_top = (H - f.shape[0]) // 2
            pad_bot = H - f.shape[0] - pad_top
            f = np.pad(f, ((pad_top, pad_bot), (0, 0), (0, 0)), mode="constant")
        parts.append(f)
        if i != len(frames) - 1:
            parts.append(np.zeros((H, pad, 3), dtype=np.uint8))
    return np.concatenate(parts, axis=1)


def _overlay_text(rgb: np.ndarray, lines: list[str], font) -> np.ndarray:
    """Burn small left-aligned text lines into the top-left of the frame."""
    img = Image.fromarray(rgb)
    draw = ImageDraw.Draw(img)
    y = 4
    for line in lines:
        # background pad for readability
        bbox = draw.textbbox((4, y), line, font=font)
        draw.rectangle((bbox[0] - 2, bbox[1] - 1, bbox[2] + 2, bbox[3] + 1), fill=(0, 0, 0))
        draw.text((4, y), line, fill=(255, 255, 255), font=font)
        y += (bbox[3] - bbox[1]) + 2
    return np.asarray(img)


def _cam_label_strip(cam_names: list[str], widths: list[int], H: int, font) -> np.ndarray:
    """Build a thin label strip naming each camera above the stitched row."""
    strip = np.zeros((H, sum(widths) + 4 * (len(widths) - 1), 3), dtype=np.uint8)
    img = Image.fromarray(strip)
    draw = ImageDraw.Draw(img)
    x = 0
    for name, w in zip(cam_names, widths):
        bbox = draw.textbbox((x + 4, 2), name, font=font)
        draw.text((x + 4, 2), name, fill=(255, 255, 0), font=font)
        x += w + 4
    return np.asarray(img)


def render_clip(h5_path: Path, cam_names: list[str], out_path: Path, fps: int = 50,
                step_stride: int = 1, max_frames: int | None = None) -> None:
    with h5py.File(h5_path, "r") as h5:
        meta = _load_metadata(h5)
        available_cams = list(h5["obs/images"].keys()) if "obs/images" in h5 else []
        cams = [c for c in cam_names if c in available_cams]
        if not cams:
            raise RuntimeError(
                f"None of requested cams {cam_names} present in {h5_path.name} "
                f"(available: {available_cams})"
            )
        T = int(h5["actions"].shape[0])
        if max_frames:
            T = min(T, max_frames)
        actions = h5["actions"][:T]
        state = h5["obs/state"][:T] if "obs/state" in h5 else None
        cube_pose = h5["obs/cube_pose"][:T] if "obs/cube_pose" in h5 else None

        # Preload cam frame datasets (lazy access)
        cam_dsets = {c: h5[f"obs/images/{c}"] for c in cams}

        font_meta = _try_load_font(14)
        font_label = _try_load_font(16)

        frames = []
        for t in range(0, T, step_stride):
            cam_frames = [np.asarray(cam_dsets[c][t], dtype=np.uint8) for c in cams]
            row = _stitch_row(cam_frames, pad=4)
            label_strip = _cam_label_strip(
                cams,
                [f.shape[1] for f in cam_frames],
                H=22, font=font_label,
            )
            row_with_labels = np.concatenate([label_strip, row], axis=0)

            a = actions[t]
            lines = [
                f"file: {h5_path.name}",
                f"t={t:03d}/{T:03d}  task={meta.get('task','?')}",
                f"action: dx={a[0]:+.2f} dy={a[1]:+.2f} dz={a[2]:+.2f}  "
                f"droll={a[3]:+.2f} dpitch={a[4]:+.2f} dyaw={a[5]:+.2f}  grip={a[6]:+.1f}",
            ]
            if cube_pose is not None:
                cp = cube_pose[t]
                lines.append(f"cube_pose: ({cp[0]:+.3f}, {cp[1]:+.3f}, {cp[2]:+.3f})")
            if state is not None:
                s = state[t]
                # Last 7 dims of a 16-D state are EE pos+quat; first 9 are joint_pos.
                if s.shape[0] >= 16:
                    lines.append(
                        f"ee_xyz: ({s[9]:+.3f}, {s[10]:+.3f}, {s[11]:+.3f})  "
                        f"fingers: ({s[7]:.3f}, {s[8]:.3f})"
                    )
                else:
                    lines.append(f"fingers: ({s[7]:.3f}, {s[8]:.3f})")

            frame = _overlay_text(row_with_labels, lines, font_meta)
            frames.append(frame)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    iio.imwrite(out_path, np.stack(frames), fps=fps, codec="libx264", quality=8)
    print(f"  wrote {out_path}  ({len(frames)} frames @ {fps}fps, {out_path.stat().st_size/1e6:.1f} MB)")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--input_dir", type=Path, required=True,
                    help="Directory of HDF5 demos (isaac_auto_collector_v6 output).")
    ap.add_argument("--output_dir", type=Path, required=True)
    ap.add_argument("--cams", type=str, default="top,front,wrist")
    ap.add_argument("--n_episodes", type=int, default=5)
    ap.add_argument("--episode_indices", type=str, default="",
                    help="Comma-separated specific episode indices to render. "
                         "Overrides --n_episodes if set.")
    ap.add_argument("--fps", type=int, default=50)
    ap.add_argument("--stride", type=int, default=1,
                    help="Sub-sample every Nth frame to make smaller clips.")
    ap.add_argument("--max_frames", type=int, default=0,
                    help="Optional cap on frames per episode (0 = all).")
    args = ap.parse_args()

    cams = [c.strip() for c in args.cams.split(",") if c.strip()]
    files = _episode_files(args.input_dir)
    if args.episode_indices:
        idxs = [int(x.strip()) for x in args.episode_indices.split(",") if x.strip()]
    else:
        rng = np.random.default_rng(0)
        idxs = sorted(rng.choice(len(files), size=min(args.n_episodes, len(files)),
                                 replace=False).tolist())
    print(f"[clips] {len(files)} episodes found in {args.input_dir}")
    print(f"[clips] rendering indices: {idxs}")

    for i, idx in enumerate(idxs):
        h5_path = files[idx]
        out_path = args.output_dir / f"ep_{idx:04d}_{h5_path.stem}.mp4"
        print(f"[clips] ({i+1}/{len(idxs)}) ep {idx:04d}: {h5_path.name}")
        render_clip(
            h5_path, cams, out_path, fps=args.fps, step_stride=args.stride,
            max_frames=args.max_frames or None,
        )

    print(f"\n[clips] done. Wrote {len(idxs)} clips to {args.output_dir}")


if __name__ == "__main__":
    main()
