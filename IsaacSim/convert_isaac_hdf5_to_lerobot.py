#!/usr/bin/env python
"""Convert Isaac auto-collector HDF5 demos to a native LeRobotDataset.

The v6 collector writes one HDF5 file per episode:

    obs/state
    actions
    obs/images/<cam>

This script maps those fields to LeRobot's training keys:

    observation.state
    action
    observation.images.<cam>

Run from an environment that can import LeRobot, for example:

    conda activate isaac_lerobot
    PYTHONPATH=/Developer/lerobot/src python \
      /Developer/omniverselab/IsaacSim/convert_isaac_hdf5_to_lerobot.py \
      /Developer/IsaacLab/logs/demos_multicam_lift \
      --repo_id local/isaac_multicam_lift \
      --root /Developer/IsaacLab/logs/lerobot_multicam_lift \
      --task "lift the cube" \
      --image_dtype video \
      --overwrite
"""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path
from typing import Any

import h5py
import numpy as np


def _load_lerobot_dataset_class():
    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "Could not import LeRobotDataset. Run in the LeRobot environment, e.g.\n"
            "  conda activate isaac_lerobot\n"
            "  PYTHONPATH=/Developer/lerobot/src python IsaacSim/convert_isaac_hdf5_to_lerobot.py ...\n"
        ) from exc
    return LeRobotDataset


def _episode_files(inputs: list[str]) -> list[Path]:
    files: list[Path] = []
    for raw in inputs:
        path = Path(raw).expanduser()
        if path.is_dir():
            files.extend(sorted(path.glob("*.hdf5")))
            files.extend(sorted(path.glob("*.h5")))
        elif path.is_file():
            files.append(path)
        else:
            raise FileNotFoundError(path)
    unique = sorted(dict.fromkeys(p.resolve() for p in files))
    if not unique:
        raise FileNotFoundError("No .hdf5/.h5 episode files found.")
    return unique


def _metadata(h5: h5py.File) -> dict[str, Any]:
    raw = h5.attrs.get("metadata")
    if raw is None:
        return {}
    if isinstance(raw, bytes):
        raw = raw.decode("utf-8")
    try:
        return json.loads(str(raw))
    except json.JSONDecodeError:
        return {}


def _camera_names(h5: h5py.File, requested: list[str] | None) -> list[str]:
    if requested:
        return requested
    meta = _metadata(h5)
    cams = meta.get("cameras")
    if isinstance(cams, list) and cams:
        return [str(c) for c in cams]
    if "obs/images" not in h5:
        return []
    return sorted(h5["obs/images"].keys())


def _feature_names(prefix: str, dim: int) -> list[str]:
    if prefix == "action" and dim == 7:
        return ["dx", "dy", "dz", "droll", "dpitch", "dyaw", "gripper"]
    return [f"{prefix}_{i}" for i in range(dim)]


def _build_features(first_file: Path, cams: list[str], image_dtype: str) -> dict[str, Any]:
    with h5py.File(first_file, "r") as h5:
        state_dim = int(h5["obs/state"].shape[1])
        action_dim = int(h5["actions"].shape[1])
        features: dict[str, Any] = {
            "observation.state": {
                "dtype": "float32",
                "shape": (state_dim,),
                "names": _feature_names("joint", state_dim),
            },
            "action": {
                "dtype": "float32",
                "shape": (action_dim,),
                "names": _feature_names("action", action_dim),
            },
        }
        for cam in cams:
            if f"obs/images/{cam}" not in h5:
                raise KeyError(f"{first_file} is missing obs/images/{cam}")
            _, height, width, channels = h5[f"obs/images/{cam}"].shape
            features[f"observation.images.{cam}"] = {
                "dtype": image_dtype,
                "shape": (int(height), int(width), int(channels)),
                "names": ["height", "width", "channel"],
            }
    return features


def _validate_episode(h5: h5py.File, cams: list[str]) -> int:
    if "obs/state" not in h5 or "actions" not in h5:
        raise KeyError("Episode must contain obs/state and actions.")
    steps = int(h5["actions"].shape[0])
    if int(h5["obs/state"].shape[0]) != steps:
        raise ValueError("obs/state and actions have different lengths.")
    for cam in cams:
        key = f"obs/images/{cam}"
        if key not in h5:
            raise KeyError(f"Missing {key}")
        if int(h5[key].shape[0]) != steps:
            raise ValueError(f"{key} length differs from actions length.")
    return steps


def convert(args: argparse.Namespace) -> Path:
    LeRobotDataset = _load_lerobot_dataset_class()
    files = _episode_files(args.inputs)
    root = Path(args.root).expanduser().resolve()
    if root.exists():
        if not args.overwrite:
            raise FileExistsError(f"{root} already exists; pass --overwrite to replace it.")
        shutil.rmtree(root)

    with h5py.File(files[0], "r") as h5:
        cams = _camera_names(h5, args.cams)
    if not cams:
        raise ValueError("No cameras found. Pass --cams or check obs/images/<cam> in the HDF5.")

    features = _build_features(files[0], cams, args.image_dtype)
    dataset = LeRobotDataset.create(
        repo_id=args.repo_id,
        fps=args.fps,
        features=features,
        root=root,
        robot_type=args.robot_type,
        use_videos=args.image_dtype == "video",
    )

    total_frames = 0
    for file_index, file_path in enumerate(files):
        with h5py.File(file_path, "r") as h5:
            steps = _validate_episode(h5, cams)
            meta = _metadata(h5)
            task = args.task or str(meta.get("task") or meta.get("env") or "isaac manipulation")
            for t in range(steps):
                frame: dict[str, Any] = {
                    "observation.state": np.asarray(h5["obs/state"][t], dtype=np.float32),
                    "action": np.asarray(h5["actions"][t], dtype=np.float32),
                    "task": task,
                }
                for cam in cams:
                    frame[f"observation.images.{cam}"] = np.asarray(
                        h5[f"obs/images/{cam}"][t], dtype=np.uint8
                    )
                dataset.add_frame(frame)
            dataset.save_episode()
            total_frames += steps
            print(f"[convert] episode {file_index:04d}: {file_path.name} ({steps} frames)")

    dataset.finalize()
    print(f"[convert] wrote {len(files)} episodes, {total_frames} frames -> {root}")
    return root


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", nargs="+", help="HDF5 episode files or directories.")
    parser.add_argument("--repo_id", required=True, help="LeRobot repo id, e.g. local/isaac_lift.")
    parser.add_argument("--root", required=True, help="Output LeRobotDataset root directory.")
    parser.add_argument("--task", default="", help="Task string stored in each LeRobot frame.")
    parser.add_argument("--robot_type", default="franka_panda")
    parser.add_argument("--fps", type=int, default=50, help="Isaac IK-Rel collector runs at 50 Hz.")
    parser.add_argument("--cams", default="", help="Comma-separated camera list. Defaults to HDF5 metadata.")
    parser.add_argument("--image_dtype", choices=["video", "image"], default="video")
    parser.add_argument("--overwrite", action="store_true", help="Replace --root if it already exists.")
    args = parser.parse_args()
    args.cams = [c.strip() for c in args.cams.split(",") if c.strip()] or None
    convert(args)


if __name__ == "__main__":
    main()
