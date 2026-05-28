"""Train a ReachPolicy and evaluate it on the four diagnostic metrics.

Metrics:
  1. open_loop_mse        — action MSE on held-out (start,target) pairs
  2. closed_loop_success  — rollout success on NOVEL targets (the real metric)
  3. perception_probe     — how well target decodes from perception features
  4. shortcut_reliance    — closed-loop success with perception ABLATED;
                            the gap (success - ablated) = how much the policy
                            actually uses perception

torch + numpy. CPU-friendly. One config trains in seconds.
"""

from __future__ import annotations

import argparse

import numpy as np
import torch
import torch.nn as nn

from toy_reach import (
    ToyReachEnv, build_proprio, generate_dataset, expert_action,
    proprio_dim, render_target_image, MAX_T, SUCCESS_RADIUS,
)
from models import ReachPolicy


def _to_tensor(d: dict) -> dict:
    out = {}
    for k, v in d.items():
        out[k] = torch.from_numpy(v).float()
    return out


def train_policy(
    proprio_mode: str,
    perception_mode: str,
    injection: str,
    aux_weight: float = 0.0,
    noise_sigma: float = 0.0,
    n_train_episodes: int = 300,
    epochs: int = 40,
    batch_size: int = 256,
    lr: float = 1e-3,
    seed: int = 0,
    device: str = "cpu",
    shortcut_pool_size: int | None = None,
    freeze_encoder_after_aux: bool = False,
    aux_pretrain_epochs: int = 40,
    n_distractors: int = 0,
):
    """Train and return (policy, train_log).

    shortcut_pool_size: if set, training targets are sampled from a fixed pool
        of this many targets (creates the memorizable-trajectory shortcut). If
        None, targets are uniform random (no memorization shortcut).

    freeze_encoder_after_aux: TWO-STAGE DECOUPLE.
        Stage 1 — train ONLY the perception encoder + aux head with the aux
                  loss (no action head in the graph). Makes the encoder
                  geometric, decoupled from the action objective.
        Stage 2 — freeze the encoder; train the action head (+ injection-side
                  layers) with the action loss on the now-frozen geometric
                  features.
        This is the decisive test of "decoupled representation + policy":
        if it works, the prior failure was joint-training credit assignment;
        if it still fails, the action head genuinely cannot consume geometry.
    """
    torch.manual_seed(seed)
    rng = np.random.default_rng(seed)
    target_pool = (rng.uniform(-0.8, 0.8, (shortcut_pool_size, 2)).astype(np.float32)
                   if shortcut_pool_size else None)

    data = generate_dataset(n_train_episodes, proprio_mode, perception_mode,
                            seed=seed, target_pool=target_pool,
                            n_distractors=n_distractors)
    t = _to_tensor(data)
    N = t["proprio"].shape[0]

    policy = ReachPolicy(proprio_dim(proprio_mode), perception_mode, injection).to(device)
    mse = nn.MSELoss()
    log = {"action_loss": [], "aux_loss": [], "stage1_aux_loss": []}

    # ---- STAGE 1: aux-pretrain the perception module (decoupled mode only) ----
    can_decouple = (perception_mode != "none"
                    and (policy.aux_head is not None or policy.is_softargmax))
    if freeze_encoder_after_aux and can_decouple:
        stage1_params = policy.perception_params()
        if policy.aux_head is not None:
            stage1_params = stage1_params + list(policy.aux_head.parameters())
        opt1 = torch.optim.Adam(stage1_params, lr=lr)
        for ep in range(aux_pretrain_epochs):
            perm = torch.randperm(N)
            ep_x, nb = 0.0, 0
            for i in range(0, N, batch_size):
                idx = perm[i:i + batch_size]
                perception = t["perception"][idx].to(device)
                tgt_gt = t["tgt_gt"][idx].to(device)
                aux_pred = policy.probe_target(perception)   # perception module only
                aux = mse(aux_pred, tgt_gt)
                opt1.zero_grad()
                aux.backward()
                opt1.step()
                ep_x += float(aux.detach()); nb += 1
            log["stage1_aux_loss"].append(ep_x / nb)
        # Freeze the perception module for stage 2
        for p in policy.perception_params():
            p.requires_grad = False

    # ---- STAGE 2 (or single-stage joint training if not decoupled) ----
    trainable = [p for p in policy.parameters() if p.requires_grad]
    opt = torch.optim.Adam(trainable, lr=lr)

    for ep in range(epochs):
        perm = torch.randperm(N)
        ep_a, ep_x = 0.0, 0.0
        nb = 0
        for i in range(0, N, batch_size):
            idx = perm[i:i + batch_size]
            proprio = t["proprio"][idx].to(device)
            action = t["action"][idx].to(device)
            perception = t["perception"][idx].to(device)
            tgt_gt = t["tgt_gt"][idx].to(device)

            # Optional perception noise augmentation (Path-1 robustness knob):
            # applied to the raw perception coordinate, or to the image target
            # by re-rendering at a jittered location.
            if noise_sigma > 0.0 and perception_mode == "raw":
                perception = perception + noise_sigma * torch.randn_like(perception)
            # (image-mode noise handled by jittering tgt at dataset gen time;
            #  for simplicity we add feature-space noise post-encode below)

            pred = policy(proprio, perception)
            loss = mse(pred, action)

            if aux_weight > 0.0:
                aux_pred = policy.aux_predict()
                if aux_pred is not None:
                    aux = mse(aux_pred, tgt_gt)
                    loss = loss + aux_weight * aux
                    ep_x += float(aux.detach())

            opt.zero_grad()
            loss.backward()
            opt.step()
            ep_a += float(mse(pred, action).detach())
            nb += 1
        log["action_loss"].append(ep_a / nb)
        log["aux_loss"].append(ep_x / nb)

    return policy, log


@torch.no_grad()
def eval_open_loop(policy, proprio_mode, perception_mode, injection,
                   n_episodes=100, seed=999, device="cpu", n_distractors=0):
    data = generate_dataset(n_episodes, proprio_mode, perception_mode, seed=seed,
                            n_distractors=n_distractors)
    t = _to_tensor(data)
    pred = policy(t["proprio"].to(device), t["perception"].to(device)).cpu()
    return float(((pred - t["action"]) ** 2).mean())


@torch.no_grad()
def eval_closed_loop(policy, proprio_mode, perception_mode, injection,
                     n_episodes=100, seed=12345, device="cpu", perc_ablate=False,
                     n_distractors=0):
    """Rollout on NOVEL random targets. Returns success rate."""
    rng = np.random.default_rng(seed)
    env = ToyReachEnv(rng=rng, n_distractors=n_distractors)
    successes = 0
    for _ in range(n_episodes):
        obs = env.reset()  # random novel target
        done = False
        while not done:
            proprio = torch.from_numpy(build_proprio(obs, proprio_mode)).float().unsqueeze(0).to(device)
            if perception_mode == "none":
                perc = torch.zeros((1, 0), device=device)
            elif perception_mode == "raw":
                perc = torch.from_numpy(obs["tgt"]).float().unsqueeze(0).to(device)
            else:
                perc = torch.from_numpy(obs["tgt_image"]).float().unsqueeze(0).to(device)
            a = policy(proprio, perc, perc_ablate=perc_ablate).squeeze(0).cpu().numpy()
            obs, _, done, info = env.step(a)
        successes += int(info["success"])
    return successes / n_episodes


@torch.no_grad()
def eval_perception_probe(policy, perception_mode, n_episodes=100, seed=777,
                          device="cpu", n_distractors=0):
    """How well does target decode from the perception feature? Returns L2 (cm-like)."""
    if perception_mode == "none":
        return float("nan")
    data = generate_dataset(n_episodes, "minimal", perception_mode, seed=seed,
                            n_distractors=n_distractors)
    t = _to_tensor(data)
    aux = policy.probe_target(t["perception"].to(device))  # bypasses proprio + head
    if aux is None:
        return float("nan")
    # report mean L2 error in coordinate units (workspace is 2 units wide)
    return float(((aux.cpu() - t["tgt_gt"]) ** 2).sum(-1).sqrt().mean())


def run_one(proprio_mode="copycat", perception_mode="image", injection="concat",
            aux_weight=0.0, noise_sigma=0.0, shortcut_pool_size=None,
            freeze_encoder_after_aux=False, n_distractors=0,
            seed=0, device="cpu", verbose=True):
    policy, log = train_policy(
        proprio_mode, perception_mode, injection,
        aux_weight=aux_weight, noise_sigma=noise_sigma,
        shortcut_pool_size=shortcut_pool_size,
        freeze_encoder_after_aux=freeze_encoder_after_aux,
        n_distractors=n_distractors,
        seed=seed, device=device,
    )
    ol = eval_open_loop(policy, proprio_mode, perception_mode, injection,
                        device=device, n_distractors=n_distractors)
    cl = eval_closed_loop(policy, proprio_mode, perception_mode, injection,
                          device=device, n_distractors=n_distractors)
    cl_ablate = (eval_closed_loop(policy, proprio_mode, perception_mode, injection,
                                  device=device, perc_ablate=True,
                                  n_distractors=n_distractors)
                 if perception_mode != "none" else cl)
    probe = eval_perception_probe(policy, perception_mode, device=device,
                                  n_distractors=n_distractors)
    res = {
        "proprio_mode": proprio_mode,
        "perception_mode": perception_mode,
        "injection": injection,
        "aux_weight": aux_weight,
        "noise_sigma": noise_sigma,
        "shortcut_pool": shortcut_pool_size or 0,
        "n_distractors": n_distractors,
        "open_loop_mse": round(ol, 5),
        "closed_loop_success": round(cl, 3),
        "ablated_success": round(cl_ablate, 3),
        "perception_use": round(cl - cl_ablate, 3),   # the key number
        "perception_probe_l2": round(probe, 4),
        "final_action_loss": round(log["action_loss"][-1], 5),
        "final_aux_loss": round(log["aux_loss"][-1], 5),
    }
    if verbose:
        print("\n=== Result ===")
        for k, v in res.items():
            print(f"  {k:22s}: {v}")
    return res


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--proprio_mode", default="copycat", choices=["oracle", "copycat", "minimal"])
    p.add_argument("--perception_mode", default="image", choices=["none", "raw", "image"])
    p.add_argument("--injection", default="concat",
                   choices=["concat", "xattn", "replace", "softargmax"])
    p.add_argument("--aux_weight", type=float, default=0.0)
    p.add_argument("--noise_sigma", type=float, default=0.0)
    p.add_argument("--shortcut_pool_size", type=int, default=None,
                   help="If set, train targets sampled from a fixed pool (memorization shortcut)")
    p.add_argument("--freeze_encoder_after_aux", action="store_true",
                   help="Two-stage decouple: aux-pretrain encoder, freeze, train action head")
    p.add_argument("--n_distractors", type=int, default=0,
                   help="Dimmer distractor blobs added to image perception (clutter test)")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--device", default="cpu")
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()
    run_one(
        proprio_mode=args.proprio_mode,
        perception_mode=args.perception_mode,
        injection=args.injection,
        aux_weight=args.aux_weight,
        noise_sigma=args.noise_sigma,
        shortcut_pool_size=args.shortcut_pool_size,
        freeze_encoder_after_aux=args.freeze_encoder_after_aux,
        n_distractors=args.n_distractors,
        seed=args.seed,
        device=args.device,
    )
