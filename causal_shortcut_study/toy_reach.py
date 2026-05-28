"""2D reaching toy environment + expert + dataset generation.

Self-contained: numpy only (torch tensors built in train_eval). No external
robotics deps. The whole point is a controllable proprioceptive shortcut so we
can study whether an action head learns to consume perception over the shortcut.

Coordinate conventions:
- ee, tgt in [-1, 1]^2
- action = delta-position, clipped to +/- MAX_STEP
- expert = proportional controller toward target
"""

from __future__ import annotations

import numpy as np

MAX_STEP = 0.1          # max delta-position per step
SUCCESS_RADIUS = 0.05   # closed-loop success threshold
MAX_T = 60              # max steps per episode
IMG_SIZE = 16           # perception "image" resolution
BLOB_SIGMA = 0.08       # Gaussian blob std (in [-1,1] coords) for the target render


def expert_action(ee: np.ndarray, tgt: np.ndarray) -> np.ndarray:
    """Proportional controller toward target, clipped. (2,) -> (2,)."""
    delta = tgt - ee
    return np.clip(delta, -MAX_STEP, MAX_STEP)


def render_target_image(tgt: np.ndarray, size: int = IMG_SIZE,
                        sigma: float = BLOB_SIGMA) -> np.ndarray:
    """Render the target as a single-channel Gaussian blob on a size×size grid.

    This is the "perception that must be decoded" — analogous to a BEV/voxel
    feature map where the object location is implicit, not handed over as a
    coordinate. Returns (1, size, size) float32 in [0, 1].
    """
    coords = np.linspace(-1.0, 1.0, size, dtype=np.float32)
    gx, gy = np.meshgrid(coords, coords, indexing="ij")  # (size, size)
    sq = (gx - tgt[0]) ** 2 + (gy - tgt[1]) ** 2
    blob = np.exp(-0.5 * sq / (sigma ** 2)).astype(np.float32)
    return blob[None]  # (1, size, size)


class ToyReachEnv:
    """2D reaching. Stateless w.r.t. dataset gen; used for both rollout + data."""

    def __init__(self, rng: np.random.Generator | None = None):
        self.rng = rng or np.random.default_rng(0)
        self.ee = np.zeros(2, dtype=np.float32)
        self.tgt = np.zeros(2, dtype=np.float32)
        self.prev_action = np.zeros(2, dtype=np.float32)
        self.t = 0

    def reset(self, ee: np.ndarray | None = None, tgt: np.ndarray | None = None):
        self.ee = (ee if ee is not None
                   else self.rng.uniform(-0.8, 0.8, 2).astype(np.float32)).copy()
        self.tgt = (tgt if tgt is not None
                    else self.rng.uniform(-0.8, 0.8, 2).astype(np.float32)).copy()
        self.prev_action = np.zeros(2, dtype=np.float32)
        self.t = 0
        return self._obs_dict()

    def step(self, action: np.ndarray):
        action = np.clip(action, -MAX_STEP, MAX_STEP).astype(np.float32)
        self.ee = np.clip(self.ee + action, -1.0, 1.0)
        self.prev_action = action
        self.t += 1
        dist = float(np.linalg.norm(self.tgt - self.ee))
        success = dist < SUCCESS_RADIUS
        done = success or self.t >= MAX_T
        return self._obs_dict(), -dist, done, {"success": success, "dist": dist}

    def _obs_dict(self) -> dict:
        return {
            "ee": self.ee.copy(),
            "tgt": self.tgt.copy(),
            "prev_action": self.prev_action.copy(),
            "tgt_image": render_target_image(self.tgt),
        }


def build_proprio(obs: dict, proprio_mode: str) -> np.ndarray:
    """Assemble the proprio vector. The TARGET is never in proprio for the
    shortcut-study modes — it is only available through perception. proprio
    carries only the shortcut signal (or nothing).

    oracle  : [ee(2), tgt(2)]        → target handed over directly. UPPER BOUND
                                       / sanity check. Perception is irrelevant
                                       here; the policy should hit ~100%.
    copycat : [ee(2), prev_action(2)] → the copycat shortcut (a_t ≈ a_{t-1})
                                       is available; target only via perception.
    minimal : [ee(2)]                → no shortcut at all; target only via
                                       perception. The policy MUST decode
                                       perception to generalize.
    """
    if proprio_mode == "oracle":
        return np.concatenate([obs["ee"], obs["tgt"]]).astype(np.float32)
    if proprio_mode == "copycat":
        return np.concatenate([obs["ee"], obs["prev_action"]]).astype(np.float32)
    if proprio_mode == "minimal":
        return obs["ee"].astype(np.float32)
    raise ValueError(f"unknown proprio_mode: {proprio_mode}")


def proprio_dim(proprio_mode: str) -> int:
    return {"oracle": 4, "copycat": 4, "minimal": 2}[proprio_mode]


def generate_dataset(
    n_episodes: int,
    proprio_mode: str,
    perception_mode: str,
    seed: int = 0,
    target_pool: np.ndarray | None = None,
):
    """Roll out the expert and collect (proprio, perception, action) tuples.

    target_pool: if given, targets are sampled from this fixed (K, 2) pool —
        used to create the "memorizable trajectory" shortcut. If None, targets
        are drawn uniformly at random (breaks the memorization shortcut).

    Returns dict of np arrays:
        proprio:    (N, proprio_dim)
        perception: (N, ...) depending on perception_mode
        action:     (N, 2)
    """
    rng = np.random.default_rng(seed)
    env = ToyReachEnv(rng=rng)

    proprios, perceptions, actions, tgt_gts = [], [], [], []
    for _ in range(n_episodes):
        if target_pool is not None:
            tgt = target_pool[rng.integers(len(target_pool))]
        else:
            tgt = None
        obs = env.reset(tgt=tgt)
        done = False
        while not done:
            a = expert_action(obs["ee"], obs["tgt"])
            proprios.append(build_proprio(obs, proprio_mode))
            perceptions.append(_perception(obs, perception_mode))
            actions.append(a.astype(np.float32))
            tgt_gts.append(obs["tgt"].astype(np.float32))  # GT target for probe + aux
            obs, _, done, _ = env.step(a)

    out = {
        "proprio": np.stack(proprios),
        "action": np.stack(actions),
        "tgt_gt": np.stack(tgt_gts),  # (N, 2) ground-truth target per sample
    }
    if perception_mode == "none":
        out["perception"] = np.zeros((len(proprios), 0), dtype=np.float32)
    else:
        out["perception"] = np.stack(perceptions)
    return out


def _perception(obs: dict, perception_mode: str) -> np.ndarray:
    if perception_mode == "none":
        return np.zeros(0, dtype=np.float32)
    if perception_mode == "raw":
        return obs["tgt"].astype(np.float32)
    if perception_mode == "image":
        return obs["tgt_image"].astype(np.float32)  # (1, IMG, IMG)
    raise ValueError(f"unknown perception_mode: {perception_mode}")
