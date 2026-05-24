# mylerobot

External research extensions on top of [lerobot](https://github.com/huggingface/lerobot):

- Language-conditioned ACT (`mylerobot.policies.act_lang`)
- 3D-2D heterogeneous vision fusion (`mylerobot.policies.vision_fusion`) — *planned*
- Cloud-LLM agentic layer + scene-graph grounding (`mylerobot.agent`, `mylerobot.perception`) — *planned*
- Closed-loop reflection / replan (`mylerobot.execution`) — *planned*

Designed so that upstream lerobot changes have **minimum** impact: we *subclass* and
*adapt* rather than fork. Coupling to lerobot is intentionally limited to:

1. `lerobot.policies.act.modeling_act.{ACT, ACTPolicy}` — subclassed
2. `lerobot.policies.act.configuration_act.ACTConfig` — subclassed
3. `lerobot.datasets.lerobot_dataset.LeRobotDataset` — wrapped (not subclassed)

## Targeted upstream commit

`8194897994be649cc85405fe54eb9ede751886b1` (2026-05-22).

`pyproject.toml` does not pin lerobot itself because we work against an editable
install at `/fs/atipa/data/rnd-liu/aiprojects/lerobot`. To reproduce on a fresh
machine, check out the commit above before `pip install -e ../lerobot`.

## Install (development)

```bash
# from the omniverselab repo root
conda activate py312          # torch 2.10 + lerobot 0.4.4 editable
pip install -e mylerobot
pytest mylerobot/tests -q
```

## Layout

```
mylerobot/
├── mylerobot/
│   ├── policies/
│   │   └── act_lang/         # language-conditioned ACT (Phase 1)
│   ├── policies/vision_fusion/   # TODO Phase 2
│   ├── perception/               # TODO Phase 3 (scene graph)
│   ├── agent/                    # TODO Phase 4 (LLM planner + reflector)
│   ├── execution/                # TODO Phase 5 (runtime + evaluator)
│   ├── data/                     # dataset wrapper, language annotator
│   └── training/                 # custom train loop
├── scripts/                  # entry points (train, eval, annotate, deploy)
├── configs/                  # yaml configs per experiment
├── tests/                    # pytest
└── pyproject.toml
```

## Roadmap

See top-level discussion notes (or `docs/ROADMAP.md` once written) for the
6-phase plan. Phase 0 = scaffolding (this commit). Phase 1 = language
conditioning smoke test on existing aloha checkpoints.
