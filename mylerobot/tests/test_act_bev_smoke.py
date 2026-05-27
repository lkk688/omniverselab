"""Smoke test: build ACT-BEV from a synthetic ACTBEVConfig and forward a dummy
batch. Checks shapes & that aux loss runs."""

from __future__ import annotations

import sys

sys.path.insert(0, "/Developer/lerobot/src")
sys.path.insert(0, "/Developer/omniverselab/mylerobot")

import torch

from lerobot.configs.types import FeatureType, PolicyFeature
from mylerobot.policies.act_bev.configuration_act_bev import ACTBEVConfig
from mylerobot.policies.act_bev.modeling_act_bev import ACTBEVPolicy


def main():
    device = "cuda" if torch.cuda.is_available() else "cpu"
    H, W = 240, 320
    state_dim = 16
    action_dim = 7
    B = 2
    chunk = 8

    cfg = ACTBEVConfig(
        chunk_size=chunk,
        n_action_steps=chunk,
        temporal_ensemble_coeff=None,
        device=device,
    )
    cfg.input_features = {
        "observation.state": PolicyFeature(type=FeatureType.STATE, shape=(state_dim,)),
        "observation.images.top": PolicyFeature(type=FeatureType.VISUAL, shape=(3, H, W)),
        "observation.images.front": PolicyFeature(type=FeatureType.VISUAL, shape=(3, H, W)),
        "observation.images.wrist": PolicyFeature(type=FeatureType.VISUAL, shape=(3, H, W)),
    }
    cfg.output_features = {
        "action": PolicyFeature(type=FeatureType.ACTION, shape=(action_dim,)),
    }

    policy = ACTBEVPolicy(cfg).to(device)
    policy.train()

    batch = {
        "observation.state": torch.randn(B, state_dim, device=device),
        "observation.images.top": torch.rand(B, 3, H, W, device=device),
        "observation.images.front": torch.rand(B, 3, H, W, device=device),
        "observation.images.wrist": torch.rand(B, 3, H, W, device=device),
        "observation.cube_pose": torch.tensor(
            [[0.5, 0.0, 0.02], [0.4, 0.1, 0.02]], device=device
        ),
        "action": torch.randn(B, chunk, action_dim, device=device),
        "action_is_pad": torch.zeros(B, chunk, dtype=torch.bool, device=device),
    }

    loss, loss_dict = policy(batch)
    print(f"[smoke] loss = {loss.item():.4f}")
    for k, v in loss_dict.items():
        print(f"[smoke]   {k} = {v:.4f}")
    assert torch.isfinite(loss), "loss is not finite"
    assert "aux_cube_heatmap_loss" in loss_dict, "aux loss missing"
    print("[smoke] OK — train forward pass works")

    # Inference path
    policy.eval()
    policy.reset()
    obs = {
        "observation.state": torch.randn(1, state_dim, device=device),
        "observation.images.top": torch.rand(1, 3, H, W, device=device),
        "observation.images.front": torch.rand(1, 3, H, W, device=device),
        "observation.images.wrist": torch.rand(1, 3, H, W, device=device),
    }
    with torch.inference_mode():
        a = policy.select_action(obs)
    print(f"[smoke] inference action shape = {tuple(a.shape)}")
    print("[smoke] OK — inference forward pass works")


if __name__ == "__main__":
    main()
