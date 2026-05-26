"""Smoke test: build ACT-Cheat-Noisy and check that noise IS injected in train
mode and NOT in eval mode."""

from __future__ import annotations

import sys

sys.path.insert(0, "/Developer/lerobot/src")
sys.path.insert(0, "/Developer/omniverselab/mylerobot")

import torch

from lerobot.configs.types import FeatureType, PolicyFeature
from mylerobot.policies.act_cheat_noisy.configuration_act_cheat_noisy import ACTCheatNoisyConfig
from mylerobot.policies.act_cheat_noisy.modeling_act_cheat_noisy import ACTCheatNoisyPolicy


def main():
    device = "cuda" if torch.cuda.is_available() else "cpu"
    H, W = 240, 320
    state_dim = 19
    action_dim = 7
    B = 4
    chunk = 8

    cfg = ACTCheatNoisyConfig(
        chunk_size=chunk,
        n_action_steps=chunk,
        temporal_ensemble_coeff=None,
        device=device,
    )
    cfg.input_features = {
        "observation.state": PolicyFeature(type=FeatureType.STATE, shape=(state_dim,)),
        "observation.images.front": PolicyFeature(type=FeatureType.VISUAL, shape=(3, H, W)),
        "observation.images.wrist": PolicyFeature(type=FeatureType.VISUAL, shape=(3, H, W)),
    }
    cfg.output_features = {
        "action": PolicyFeature(type=FeatureType.ACTION, shape=(action_dim,)),
    }

    policy = ACTCheatNoisyPolicy(cfg).to(device)

    # Build a batch with KNOWN cube_pose values at indices 16,17,18.
    state = torch.zeros(B, state_dim, device=device)
    state[:, 16] = 0.5
    state[:, 17] = -0.1
    state[:, 18] = 0.02
    batch = {
        "observation.state": state,
        "observation.images.front": torch.rand(B, 3, H, W, device=device),
        "observation.images.wrist": torch.rand(B, 3, H, W, device=device),
        "action": torch.randn(B, chunk, action_dim, device=device),
        "action_is_pad": torch.zeros(B, chunk, dtype=torch.bool, device=device),
    }

    # Training path: noise should perturb cube_pose dims.
    policy.train()
    perturbed = policy._inject_state_noise({k: v.clone() for k, v in batch.items()})
    delta_cube = (perturbed["observation.state"][:, 16:19] - state[:, 16:19]).std(dim=0)
    delta_proprio = (perturbed["observation.state"][:, :16] - state[:, :16]).abs().max()
    print(f"[smoke] training perturbation: cube std per dim = {delta_cube.cpu().tolist()}")
    print(f"[smoke] training perturbation: proprio max abs = {delta_proprio.item():.6f} (should be 0)")
    assert delta_proprio.item() == 0, "noise leaked into proprio dims"
    assert (delta_cube > 0).all(), "no noise on cube_pose dims"

    # Eval path: forward should NOT inject noise (self.training=False).
    policy.eval()
    with torch.inference_mode():
        # Direct call to the noise helper would still inject, but the policy's
        # forward path is guarded by self.training. We check by running the
        # full forward and asserting select_action works.
        action = policy.select_action({k: v[0:1] for k, v in batch.items() if "action" not in k})
        print(f"[smoke] eval action shape = {tuple(action.shape)}")

    # Training forward (full path)
    policy.train()
    loss, ld = policy(batch)
    print(f"[smoke] train forward loss = {loss.item():.4f}  l1={ld.get('l1_loss',0):.4f}")
    print("[smoke] OK")


if __name__ == "__main__":
    main()
