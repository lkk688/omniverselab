#!/usr/bin/env python
"""Geometry-repair test (no retraining): does canonicalizing the perturbed scene
camera recover camera-viewpoint robustness?

Mechanism (verified): LIBERO-Plus "Camera Viewpoints" perturbs the agentview by
ROTATING it (fixed position [0.8966,0,0.65], varying quat). The policy's obs comes
from this rotated `agentview`. The REPAIR renders the scene from the CANONICAL
agentview pose (derived at runtime from a non-camera-perturbed task of the same
suite) and substitutes it for the perturbed image — a geometry-stabilized, in-
distribution view, with NO change to the frozen policy.

Compare, on the SAME Camera-Viewpoints tasks:
  - baseline : policy sees the perturbed agentview (normal LIBERO-Plus eval)
  - repair   : policy sees the canonical agentview render instead

Reading:
  - repair ≫ baseline → the perturbation hurt via the image; a geometry-stabilized
    view recovers it → the policy DOES use the scene cam → a viewpoint-invariant
    readout is motivated.
  - repair ≈ baseline → consistent with the policy under-using the scene cam
    (wrist-centric): canonicalizing the view it ignores changes nothing.

This is a no-retrain PROXY for the readout fix; it does not inject explicit
geometry into the policy (the frozen policy has no pathway for that — that needs
the readout-fix retraining).
"""
from __future__ import annotations

import mylerobot  # noqa: F401  # shim before lerobot

import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np
import torch

os.environ.setdefault("MUJOCO_GL", "egl")

from lerobot.envs.configs import LiberoEnv as LiberoEnvConfig
from lerobot.envs.factory import make_env_pre_post_processors
from lerobot.envs.libero import LiberoEnv
from lerobot.envs.utils import preprocess_observation
from lerobot.utils.constants import ACTION
from libero.libero import benchmark

sys.path.insert(0, str(Path(__file__).resolve().parent))
from eval_libero_plus_fragility import _batched, load_policy_polymorphic  # noqa: E402


def _derive_canonical_pose(suite, suite_name, name_to_idx, classif):
    """Read the agentview pose from a non-camera-perturbed task (= canonical)."""
    ref = [t for t in classif.get(suite_name, [])
           if t["category"] != "Camera Viewpoints" and t["name"] in name_to_idx]
    if not ref:
        raise RuntimeError(f"no non-camera reference task for {suite_name}")
    tid = name_to_idx[ref[0]["name"]]
    env = LiberoEnv(task_suite=suite, task_id=tid, task_suite_name=suite_name,
                    observation_height=256, observation_width=256,
                    obs_type="pixels_agent_pos", episode_index=100, is_libero_plus=True)
    env.reset(seed=None)
    sim = env._env.env.sim
    aid = sim.model.camera_name2id("agentview")
    pos, quat = sim.model.cam_pos[aid].copy(), sim.model.cam_quat[aid].copy()
    env.close()
    return pos, quat


def _render_canonical(env, canon_pos, canon_quat) -> np.ndarray:
    """Render agentview from the canonical pose; restore after. (H,W,3) uint8."""
    sim = env._env.env.sim
    aid = sim.model.camera_name2id("agentview")
    old_pos, old_quat = sim.model.cam_pos[aid].copy(), sim.model.cam_quat[aid].copy()
    sim.model.cam_pos[aid] = canon_pos
    sim.model.cam_quat[aid] = canon_quat
    sim.forward()
    # [::-1] vertical flip to match the env obs orientation; ascontiguousarray so
    # the (negative-stride) view becomes a real array torch.from_numpy accepts.
    img = np.ascontiguousarray(sim.render(width=256, height=256, camera_name="agentview")[::-1],
                               dtype=np.uint8)
    sim.model.cam_pos[aid], sim.model.cam_quat[aid] = old_pos, old_quat
    sim.forward()
    return img


def rollout(env, policy, env_pre, env_post, pre, post, max_steps, device, task_desc,
            expected_keys, repair=False, canon=None) -> bool:
    obs, info = env.reset(seed=None)
    policy.reset()
    success = False
    for _ in range(max_steps):
        if repair:  # substitute canonical agentview render before preprocessing
            obs["pixels"]["image"] = _render_canonical(env, canon[0], canon[1])
        observation = preprocess_observation(_batched(obs))
        observation["task"] = [task_desc]
        observation = env_pre(observation)
        if "observation.images.front" in expected_keys and "observation.images.image" in observation:
            observation["observation.images.front"] = observation.pop("observation.images.image")
        if "observation.images.wrist" in expected_keys and "observation.images.image2" in observation:
            observation["observation.images.wrist"] = observation.pop("observation.images.image2")
        observation = pre(observation)
        observation = {k: (v.to(device, non_blocking=True) if torch.is_tensor(v) else v)
                       for k, v in observation.items()}
        with torch.inference_mode():
            action = policy.select_action(observation)
        action = post(action)
        transition = env_post({ACTION: action})
        action_np = transition[ACTION].detach().cpu().numpy().reshape(-1).astype(np.float32)
        obs, _, terminated, truncated, info = env.step(action_np)
        if info.get("is_success", False):
            success = True
        if terminated or truncated or success:
            break
    return success


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy_path", default="lerobot/pi0_libero_finetuned_v044")
    ap.add_argument("--suite", default="libero_object")
    ap.add_argument("--max_tasks", type=int, default=20)
    ap.add_argument("--max_steps", type=int, default=220)
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    ap.add_argument("--classification_json", type=Path,
                    default=Path("/data/rnd-liu/aiprojects/LIBERO-plus/libero/libero/benchmark/task_classification.json"))
    ap.add_argument("--out_json", type=Path,
                    default=Path("/data/rnd-liu/aiprojects/lerobot/outputs/eval/camera_repair.json"))
    args = ap.parse_args()

    classif = json.loads(args.classification_json.read_text())
    suite = benchmark.get_benchmark_dict()[args.suite]()
    name_to_idx = {t.name: i for i, t in enumerate(suite.tasks)}

    canon_pos, canon_quat = _derive_canonical_pose(suite, args.suite, name_to_idx, classif)
    print(f"[repair] canonical agentview pos={np.round(canon_pos,4)} "
          f"quat={np.round(canon_quat,4)}", flush=True)

    policy, pre, post, cfg = load_policy_polymorphic(args.policy_path, args.device)
    expected = set(getattr(cfg, "image_features", {}).keys())
    env_cfg = LiberoEnvConfig(task=args.suite)
    env_pre, env_post = make_env_pre_post_processors(env_cfg=env_cfg, policy_cfg=cfg)

    cam_tasks = [t for t in classif.get(args.suite, []) if t["category"] == "Camera Viewpoints"]
    cam_tasks = [t for t in cam_tasks if t["name"] in name_to_idx][: args.max_tasks]
    print(f"[repair] {len(cam_tasks)} Camera-Viewpoints tasks", flush=True)

    res = {"baseline": 0, "repair": 0, "n": 0}
    t0 = time.time()
    for i, te in enumerate(cam_tasks):
        tid = name_to_idx[te["name"]]
        for cond in ("baseline", "repair"):
            env = LiberoEnv(task_suite=suite, task_id=tid, task_suite_name=args.suite,
                            observation_height=256, observation_width=256,
                            obs_type="pixels_agent_pos", episode_index=100,
                            is_libero_plus=True)
            ok = rollout(env, policy, env_pre, env_post, pre, post, args.max_steps,
                         args.device, suite.tasks[tid].language, expected,
                         repair=(cond == "repair"), canon=(canon_pos, canon_quat))
            env.close()
            res[cond] += int(ok)
        res["n"] += 1
        print(f"[repair] task {i+1}/{len(cam_tasks)}  baseline={res['baseline']}/{res['n']}  "
              f"repair={res['repair']}/{res['n']}  ({time.time()-t0:.0f}s)", flush=True)

    n = res["n"]
    b = 100.0 * res["baseline"] / n
    r = 100.0 * res["repair"] / n
    print("\n" + "=" * 60)
    print(f"GEOMETRY-REPAIR (canonical agentview) | {args.suite} | Camera Viewpoints | n={n}")
    print(f"  baseline (perturbed agentview) : {b:.1f}%  ({res['baseline']}/{n})")
    print(f"  repair   (canonical agentview) : {r:.1f}%  ({res['repair']}/{n})")
    print(f"  recovery                       : {r-b:+.1f} pp")
    print("=" * 60)
    print("repair≫baseline -> policy uses scene cam, canonicalization recovers it;")
    print("repair≈baseline -> scene cam under-used (wrist-centric); view fix changes little.")

    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps({
        "suite": args.suite, "n": n, "baseline_pc": b, "repair_pc": r,
        "recovery_pp": r - b, "raw": res,
        "canonical_pos": canon_pos.tolist(), "canonical_quat": canon_quat.tolist(),
    }, indent=2))
    print(f"[repair] wrote {args.out_json}")


if __name__ == "__main__":
    main()
