"""Sweep the causal-shortcut matrix and tabulate results.

Each cell is averaged over multiple seeds (closed-loop success at n=100
episodes has a wide CI — we learned the small-n variance lesson the hard way
on LIBERO-Plus, so we average seeds here by default).

Usage:
    python run_matrix.py --seeds 3 --out results.csv
    python run_matrix.py --plot results.csv     # render summary plot
"""

from __future__ import annotations

import argparse
import csv
import statistics

from train_eval import run_one


# Curated, informative cells (not the full cartesian product — many
# combinations are redundant). Each tuple:
#   (label, proprio_mode, perception_mode, injection, aux_weight,
#    noise_sigma, shortcut_pool_size, freeze_encoder_after_aux)
CELLS = [
    # --- Upper bound: target handed over directly in proprio ---
    ("oracle",                 "oracle",  "raw",   "concat", 0.0, 0.0, None, False),

    # --- Consumption study (raw = trivial decode; isolates shortcut effect) ---
    ("raw/minimal/concat",     "minimal", "raw",   "concat", 0.0, 0.0, None, False),
    ("raw/copycat/concat",     "copycat", "raw",   "concat", 0.0, 0.0, None, False),
    ("raw/copycat/concat/pool8","copycat","raw",   "concat", 0.0, 0.0, 8, False),
    ("raw/copycat/xattn/pool8","copycat", "raw",   "xattn",  0.0, 0.0, 8, False),
    ("raw/copycat/replace/pool8","copycat","raw",  "replace",0.0, 0.0, 8, False),

    # --- Decode study (image = hard CNN decode; the realistic case) ---
    ("img/minimal/concat",     "minimal", "image", "concat", 0.0, 0.0, None, False),
    ("img/minimal/concat/aux", "minimal", "image", "concat", 0.5, 0.0, None, False),
    ("img/minimal/xattn",      "minimal", "image", "xattn",  0.0, 0.0, None, False),
    ("img/minimal/xattn/aux",  "minimal", "image", "xattn",  0.5, 0.0, None, False),
    ("img/minimal/replace",    "minimal", "image", "replace",0.0, 0.0, None, False),
    ("img/minimal/replace/aux","minimal", "image", "replace",0.5, 0.0, None, False),

    # --- Hardest: image decode AND shortcut available (full causal confusion) ---
    ("img/copycat/concat/pool8","copycat","image","concat", 0.0, 0.0, 8, False),
    ("img/copycat/concat/pool8/aux","copycat","image","concat",0.5,0.0,8, False),
    ("img/copycat/xattn/pool8/aux","copycat","image","xattn",0.5,0.0,8, False),

    # --- SOFT-ARGMAX READOUT (the breakthrough: location-preserving readout) ---
    # vs the mean-pool concat/xattn/replace above, which destroy location.
    ("img/minimal/softargmax",      "minimal","image","softargmax",0.0, 0.0, None, False),
    ("img/minimal/softargmax/aux",  "minimal","image","softargmax",0.5, 0.0, None, False),
    ("img/minimal/softargmax/decouple","minimal","image","softargmax",0.5,0.0,None, True),
    # Shortcut present: does the location-preserving readout survive memorization?
    ("img/copycat/softargmax/pool8","copycat","image","softargmax",0.0, 0.0, 8, False),
    ("img/copycat/softargmax/pool8/decouple","copycat","image","softargmax",0.5,0.0,8, True),

    # --- Noise-tolerance sweet spot (raw, minimal, vary noise) ---
    ("raw/min/noise0.02",      "minimal", "raw",   "replace",0.0, 0.02, None, False),
    ("raw/min/noise0.05",      "minimal", "raw",   "replace",0.0, 0.05, None, False),
    ("raw/min/noise0.10",      "minimal", "raw",   "replace",0.0, 0.10, None, False),
]

FIELDS = ["label", "closed_loop_success", "ablated_success", "perception_use",
          "perception_probe_l2", "open_loop_mse"]


def run_matrix(seeds: int, out_csv: str | None):
    rows = []
    for cell in CELLS:
        label, pm, perc, inj, auxw, noise, pool, freeze = cell
        per_seed = []
        for s in range(seeds):
            res = run_one(proprio_mode=pm, perception_mode=perc, injection=inj,
                          aux_weight=auxw, noise_sigma=noise, shortcut_pool_size=pool,
                          freeze_encoder_after_aux=freeze,
                          seed=s, verbose=False)
            per_seed.append(res)
        # average the scalar metrics across seeds
        agg = {"label": label}
        for k in ["closed_loop_success", "ablated_success", "perception_use",
                  "perception_probe_l2", "open_loop_mse"]:
            vals = [r[k] for r in per_seed]
            agg[k] = round(statistics.mean(vals), 3)
            agg[k + "_std"] = round(statistics.pstdev(vals), 3) if seeds > 1 else 0.0
        agg["full_cfg"] = f"proprio={pm} perc={perc} inj={inj} aux={auxw} noise={noise} pool={pool}"
        rows.append(agg)
        print(f"  done: {label:32s} "
              f"CL={agg['closed_loop_success']:.2f}±{agg['closed_loop_success_std']:.2f}  "
              f"perc_use={agg['perception_use']:.2f}  "
              f"probe={agg['perception_probe_l2']:.3f}")

    # Pretty table
    print("\n" + "=" * 100)
    print(f"{'cell':34s} {'CL_succ':>9} {'ablated':>9} {'perc_use':>9} {'probe_L2':>9} {'OL_mse':>9}")
    print("-" * 100)
    for r in rows:
        print(f"{r['label']:34s} "
              f"{r['closed_loop_success']:>9.2f} "
              f"{r['ablated_success']:>9.2f} "
              f"{r['perception_use']:>9.2f} "
              f"{r['perception_probe_l2']:>9.3f} "
              f"{r['open_loop_mse']:>9.4f}")
    print("=" * 100)
    print("CL_succ   = closed-loop success on NOVEL targets (the real metric)")
    print("ablated   = same with perception zeroed at test")
    print("perc_use  = CL_succ - ablated = how much the policy actually uses perception")
    print("probe_L2  = target-decode L2 from perception features (lower=more geometric; needs aux)")
    print("OL_mse    = open-loop action MSE (can be low via shortcut — not a success signal)")

    if out_csv:
        all_keys = sorted({k for r in rows for k in r})
        with open(out_csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=all_keys)
            w.writeheader()
            w.writerows(rows)
        print(f"\nwrote {out_csv}")
    return rows


def plot(csv_path: str):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rows = list(csv.DictReader(open(csv_path)))
    labels = [r["label"] for r in rows]
    cl = [float(r["closed_loop_success"]) for r in rows]
    use = [float(r["perception_use"]) for r in rows]

    fig, ax = plt.subplots(figsize=(12, 7))
    y = range(len(labels))
    ax.barh(y, cl, height=0.4, label="closed-loop success", align="edge")
    ax.barh([v + 0.4 for v in y], use, height=0.4, label="perception use", align="edge")
    ax.set_yticks([v + 0.4 for v in y])
    ax.set_yticklabels(labels, fontsize=8)
    ax.set_xlabel("rate")
    ax.legend()
    ax.set_title("Causal-shortcut study: closed-loop success vs perception use")
    plt.tight_layout()
    out = csv_path.replace(".csv", ".png")
    plt.savefig(out, dpi=120)
    print(f"wrote {out}")


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--seeds", type=int, default=3)
    p.add_argument("--out", default="results.csv")
    p.add_argument("--plot", default=None, help="path to an existing results.csv to plot")
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()
    if args.plot:
        plot(args.plot)
    else:
        run_matrix(args.seeds, args.out)
