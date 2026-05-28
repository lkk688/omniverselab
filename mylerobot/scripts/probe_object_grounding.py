#!/usr/bin/env python
"""Real-system PROBE: is object 3D position linearly decodable from pi0's
visual features — and does a location-preserving readout recover more than a
mean-pool? This is the real-system analog of the causal-shortcut toy's
`perception_probe`, run on the EXISTING trained pi0 with NO retraining.

Why this exists (causal_shortcut_study finding): a mean-pool readout destroys
spatial location, capping decode accuracy regardless of how geometric the
encoder is; a location-preserving (soft-argmax/keypoint) readout fixes it. This
script tests whether that mechanism is the live bottleneck on pi0+LIBERO:

  - POOLED probe : ridge regression from the MEAN-POOLED SigLIP tokens -> object xyz
                   ("what a pooled readout sees")
  - SPATIAL probe: ridge from the SAME tokens kept per-position (location preserved
                   via a fixed per-token random projection) -> object xyz
  - chance       : predict the train-set mean object position
  - proprio      : ridge from the robot EEF position (is object pos trivially in
                   proprio? it should NOT be)

Reading:
  * POOLED ≪ chance            -> object geometry is present even after pooling;
                                  readout is NOT the presence bottleneck (consumption is).
  * POOLED ≈ chance, SPATIAL ≪  -> location IS in the features but pooling destroys it;
                                  the toy's readout diagnosis holds on the real system.
  * both ≈ chance              -> pi0's vision encoder doesn't linearly encode object
                                  geometry at all; a readout fix on top won't help.

No training of the policy. Captures features over policy-driven rollouts.
"""
from __future__ import annotations

import mylerobot  # noqa: F401  # shim MUST precede lerobot imports

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


# ---- feature capture --------------------------------------------------------
def _agentview_tokens(policy, observation) -> torch.Tensor:
    """Exact agentview SigLIP tokens for this obs.

    Reuses the policy's own image preprocessing (resize_with_pad + [-1,1] norm)
    so the tokens are identical to what the action expert consumes. images[0] is
    the agentview cam (index 0 of config.image_features). Returns (T, H) on CPU.
    """
    images, _ = policy._preprocess_images(observation)
    with torch.inference_mode():
        tok = policy.model.paligemma_with_expert.embed_image(images[0])
    return tok.detach().float().cpu()[0]  # (T, H)


def _obj_body_id(sim, name: str) -> int | None:
    """Find the MuJoCo body id for an object by name prefix (no render)."""
    for bn in sim.model.body_names:
        if bn == name or bn == f"{name}_main" or bn.startswith(f"{name}_"):
            return sim.model.body_name2id(bn)
    return None


# ---- ridge probe (closed form, GPU) -----------------------------------------
def _ridge_fit_eval(Xtr, Ytr, Xte, Yte, lam):
    """Standardize on train, fit ridge, return test euclidean RMSE + per-axis RMSE."""
    mu, sd = Xtr.mean(0, keepdim=True), Xtr.std(0, keepdim=True) + 1e-6
    Xtr = (Xtr - mu) / sd
    Xte = (Xte - mu) / sd
    n, d = Xtr.shape
    Xtr1 = torch.cat([Xtr, torch.ones(n, 1, device=Xtr.device)], 1)
    Xte1 = torch.cat([Xte, torch.ones(Xte.shape[0], 1, device=Xte.device)], 1)
    A = Xtr1.T @ Xtr1
    reg = lam * torch.eye(A.shape[0], device=A.device)
    reg[-1, -1] = 0.0  # don't regularize bias
    b = Xtr1.T @ Ytr
    try:
        W = torch.linalg.solve(A + reg, b)
    except torch._C._LinAlgError:
        W = torch.linalg.lstsq(A + reg, b).solution
    pred = Xte1 @ W
    err = pred - Yte
    rmse = torch.sqrt((err ** 2).sum(1).mean()).item()          # euclidean
    per_axis = torch.sqrt((err ** 2).mean(0)).tolist()
    return rmse, per_axis


def _probe(name, X, Y, ep_ids, device, lam_grid=(1e-1, 1.0, 10.0, 100.0)):
    """Episode-grouped 70/30 split; pick lam on a train-internal val; eval test."""
    X = torch.as_tensor(X, dtype=torch.float32, device=device)
    Y = torch.as_tensor(Y, dtype=torch.float32, device=device)
    ep_ids = np.asarray(ep_ids)
    uniq = np.unique(ep_ids)
    rng = np.random.default_rng(0)
    rng.shuffle(uniq)
    if len(uniq) < 5:  # too few episodes to carve a val set; fixed lam, 70/30
        n_te = max(1, int(0.3 * len(uniq)))
        te_eps = set(uniq[:n_te].tolist())
        te = np.array([e in te_eps for e in ep_ids]); tr = ~te
        rmse, per_axis = _ridge_fit_eval(X[tr], Y[tr], X[te], Y[te], 10.0)
        return {"name": name, "test_rmse": rmse, "per_axis_rmse": per_axis,
                "lambda": 10.0, "n_train": int(tr.sum()), "n_test": int(te.sum()),
                "feat_dim": int(X.shape[1])}
    n_te = max(1, int(0.3 * len(uniq)))
    te_eps = set(uniq[:n_te].tolist())
    va_eps = set(uniq[n_te:2 * n_te].tolist())  # val carved from remaining
    te = np.array([e in te_eps for e in ep_ids])
    va = np.array([e in va_eps for e in ep_ids])
    tr = ~(te | va)
    Xtr, Ytr = X[tr], Y[tr]
    # pick lambda on val
    best = (None, float("inf"))
    for lam in lam_grid:
        r, _ = _ridge_fit_eval(Xtr, Ytr, X[va], Y[va], lam)
        if r < best[1]:
            best = (lam, r)
    lam = best[0]
    # refit on train+val, eval on test
    trva = tr | va
    rmse, per_axis = _ridge_fit_eval(X[trva], Y[trva], X[te], Y[te], lam)
    return {"name": name, "test_rmse": rmse, "per_axis_rmse": per_axis,
            "lambda": lam, "n_train": int(trva.sum()), "n_test": int(te.sum()),
            "feat_dim": int(X.shape[1])}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy_path", default="lerobot/pi0_libero_finetuned_v044")
    ap.add_argument("--suite", default="libero_object")
    ap.add_argument("--max_tasks", type=int, default=10)
    ap.add_argument("--episodes_per_task", type=int, default=6)
    ap.add_argument("--max_steps", type=int, default=180)
    ap.add_argument("--target_samples", type=int, default=1500)
    ap.add_argument("--capture_stride", type=int, default=1,
                    help="capture a probe sample every N rollout steps")
    ap.add_argument("--proj_dim", type=int, default=32,
                    help="per-token random projection dim for the spatial probe")
    ap.add_argument("--libero_plus", type=int, default=1,
                    help="1 = suffix-stripping init-state loader (required: the installed "
                         "benchmark is the LIBERO-Plus variant whose task files carry "
                         "perturbation suffixes). 0 only works on vanilla LIBERO.")
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    ap.add_argument("--out_json", type=Path,
                    default=Path("/data/rnd-liu/aiprojects/lerobot/outputs/eval/probe_object_grounding.json"))
    args = ap.parse_args()

    print(f"[probe] policy={args.policy_path} suite={args.suite} "
          f"target_samples={args.target_samples} device={args.device}", flush=True)

    policy, pre, post, cfg = load_policy_polymorphic(args.policy_path, args.device)
    img_feats = list(getattr(cfg, "image_features", {}).keys())
    print(f"[probe] image_features order = {img_feats}  (index 0 = agentview)", flush=True)
    expected = set(img_feats)

    suite = benchmark.get_benchmark_dict()[args.suite]()
    env_cfg = LiberoEnvConfig(task=args.suite)
    env_pre, env_post = make_env_pre_post_processors(env_cfg=env_cfg, policy_cfg=cfg)

    pooled, spatial, obj_xyz, eef_xyz, ep_ids = [], [], [], [], []
    proj = None  # lazy fixed per-token random projection (hidden -> proj_dim)
    gen = torch.Generator().manual_seed(0)
    ep_counter = 0
    t0 = time.time()
    n_tasks = min(args.max_tasks, len(suite.tasks))

    for task_id in range(n_tasks):
        if len(obj_xyz) >= args.target_samples:
            break
        task = suite.tasks[task_id]
        for ep in range(args.episodes_per_task):
            if len(obj_xyz) >= args.target_samples:
                break
            env = LiberoEnv(task_suite=suite, task_id=task_id, task_suite_name=args.suite,
                            observation_height=256, observation_width=256,
                            obs_type="pixels_agent_pos", episode_index=100 + ep,
                            is_libero_plus=bool(args.libero_plus))
            obs, info = env.reset(seed=None)
            policy.reset()
            ep_counter += 1
            inner = env._env.env
            sim = inner.sim
            ooi = list(getattr(inner, "obj_of_interest", []) or [])
            bid = _obj_body_id(sim, ooi[0]) if ooi else None
            if bid is None:
                print(f"[probe] task {task_id} ep {ep}: no obj_of_interest body found "
                      f"(ooi={ooi}); skipping episode", flush=True)
                if hasattr(env, "close"):
                    env.close()
                continue
            for step in range(args.max_steps):
                observation = preprocess_observation(_batched(obs))
                observation["task"] = [task.language]
                observation = env_pre(observation)
                if "observation.images.front" in expected and "observation.images.image" in observation:
                    observation["observation.images.front"] = observation.pop("observation.images.image")
                if "observation.images.wrist" in expected and "observation.images.image2" in observation:
                    observation["observation.images.wrist"] = observation.pop("observation.images.image2")
                observation = pre(observation)
                observation = {k: (v.to(args.device, non_blocking=True) if torch.is_tensor(v) else v)
                               for k, v in observation.items()}

                # Capture a probe sample every `capture_stride` steps (no extra render:
                # object pos from sim.body_xpos, eef from the env obs already rendered).
                if (len(obj_xyz) < args.target_samples
                        and step % args.capture_stride == 0):
                    oxyz = sim.data.body_xpos[bid].astype(np.float32).copy()
                    exyz = np.asarray(obs["robot_state"]["eef"]["pos"], np.float32)
                    t = _agentview_tokens(policy, observation)        # (T, H)
                    if proj is None:
                        proj = torch.randn(t.shape[1], args.proj_dim, generator=gen)
                    pooled.append(t.mean(0).numpy())                  # (H,)
                    spatial.append((t @ proj).reshape(-1).numpy())    # (T*proj_dim,)
                    obj_xyz.append(oxyz)
                    eef_xyz.append(exyz)
                    ep_ids.append(ep_counter)

                with torch.inference_mode():
                    action = policy.select_action(observation)
                action = post(action)
                transition = env_post({ACTION: action})
                action_np = transition[ACTION].detach().cpu().numpy().reshape(-1).astype(np.float32)
                obs, reward, terminated, truncated, info = env.step(action_np)
                if terminated or truncated or info.get("is_success", False):
                    break
            if hasattr(env, "close"):
                env.close()
            print(f"[probe] task {task_id} ep {ep} -> {len(obj_xyz)} samples "
                  f"({time.time()-t0:.0f}s)", flush=True)

    N = len(obj_xyz)
    print(f"\n[probe] collected {N} samples from {ep_counter} episodes in {(time.time()-t0)/60:.1f} min",
          flush=True)
    if N < 50:
        print("[probe] too few samples; aborting.")
        return

    Y = np.stack(obj_xyz)
    Xp = np.stack(pooled)
    Xs = np.stack(spatial)
    Xe = np.stack(eef_xyz)
    dev = args.device

    # chance = predict train mean (uses same split as probes for comparability)
    ep_arr = np.asarray(ep_ids)
    uniq = np.unique(ep_arr); rng = np.random.default_rng(0); rng.shuffle(uniq)
    n_te = max(1, int(0.3 * len(uniq))); te_eps = set(uniq[:n_te].tolist())
    te = np.array([e in te_eps for e in ep_arr]); tr = ~te
    mean_pred = Y[tr].mean(0, keepdims=True)
    chance_rmse = float(np.sqrt(((Y[te] - mean_pred) ** 2).sum(1).mean()))
    obj_spread = float(np.sqrt(Y.var(0).sum()))  # std magnitude of object pos

    res_pool = _probe("pooled (mean over tokens)", Xp, Y, ep_ids, dev)
    res_spat = _probe("spatial (location-preserving)", Xs, Y, ep_ids, dev)
    res_eef = _probe("proprio (EEF pos)", Xe, Y, ep_ids, dev)

    print("\n" + "=" * 78)
    print(f"OBJECT-GROUNDING PROBE  |  suite={args.suite}  N={N}  "
          f"target=obj_of_interest[0] xyz (meters)")
    print("-" * 78)
    print(f"{'probe':32s} {'test_RMSE(m)':>12} {'feat_dim':>9} {'lambda':>8}")
    print("-" * 78)
    print(f"{'chance (predict mean)':32s} {chance_rmse:>12.4f} {'-':>9} {'-':>8}")
    for r in (res_eef, res_pool, res_spat):
        print(f"{r['name']:32s} {r['test_rmse']:>12.4f} {r['feat_dim']:>9d} {r['lambda']:>8.1f}")
    print("-" * 78)
    print(f"object position spread (||std||) = {obj_spread:.4f} m")
    pool_gain = chance_rmse - res_pool["test_rmse"]
    spat_gain = res_spat["test_rmse"] - res_pool["test_rmse"]
    print(f"pooled beats chance by: {pool_gain:+.4f} m  |  "
          f"spatial beats pooled by: {-spat_gain:+.4f} m")
    print("=" * 78)
    print("READING: pooled≪chance -> geometry present post-pool (consumption is the issue);")
    print("         pooled≈chance & spatial≪pooled -> pooling destroys location (readout issue);")
    print("         both≈chance -> encoder doesn't linearly encode object geometry.")

    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps({
        "policy": args.policy_path, "suite": args.suite, "n_samples": N,
        "n_episodes": ep_counter, "is_libero_plus": bool(args.libero_plus),
        "chance_rmse": chance_rmse, "object_spread": obj_spread,
        "probes": {"proprio": res_eef, "pooled": res_pool, "spatial": res_spat},
    }, indent=2))
    print(f"\n[probe] wrote {args.out_json}")


if __name__ == "__main__":
    main()
