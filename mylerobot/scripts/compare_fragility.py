#!/usr/bin/env python
"""Compare two LIBERO-Plus fragility eval JSONs and print a delta markdown table.

Use to compare baseline vs voxel-augmented π0 (or any two checkpoints):

    python scripts/compare_fragility.py \\
        --baseline /data/.../pi0_libero_plus_fragility_full.json \\
        --variant  /data/.../pi0voxel_libero_plus_fragility_full.json

Output: markdown table with `variant% (baseline%) Δ+/-pp` cells, plus summary
row/col means and overall mean delta.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--baseline", type=Path, required=True)
    p.add_argument("--variant", type=Path, required=True)
    p.add_argument("--label_baseline", default="baseline")
    p.add_argument("--label_variant", default="variant")
    return p.parse_args()


def cell_pc(per_suite_cell):
    if not per_suite_cell or per_suite_cell.get("total", 0) == 0:
        return None
    return 100.0 * per_suite_cell["successes"] / per_suite_cell["total"]


def main():
    args = parse_args()
    base = json.loads(args.baseline.read_text())
    var = json.loads(args.variant.read_text())

    # Sanity check: same suites + categories
    if set(base["suites"]) != set(var["suites"]):
        print(f"⚠ suites mismatch: {base['suites']} vs {var['suites']}")
    if set(base["categories"]) != set(var["categories"]):
        print(f"⚠ categories mismatch")

    suites = base["suites"]
    cats = base["categories"]

    print(f"# Fragility delta: **{args.label_variant}** vs **{args.label_baseline}**\n")
    print(f"_Baseline file_: `{args.baseline}`")
    print(f"_Variant file_:  `{args.variant}`\n")

    # Header
    print("| Suite | " + " | ".join(cats) + " | row Δ |")
    print("|" + "|".join(["---"] * (len(cats) + 2)) + "|")

    all_deltas: list[float] = []
    col_deltas: dict[str, list[float]] = {c: [] for c in cats}
    for suite in suites:
        row = [suite]
        suite_deltas = []
        for cat in cats:
            bc = base["per_suite"].get(suite, {}).get(cat, {})
            vc = var["per_suite"].get(suite, {}).get(cat, {})
            bp = cell_pc(bc)
            vp = cell_pc(vc)
            if bp is None or vp is None:
                row.append("—")
            else:
                d = vp - bp
                sign = "+" if d >= 0 else ""
                row.append(f"{vp:.1f} ({bp:.1f}) {sign}{d:+.1f}")
                suite_deltas.append(d)
                col_deltas[cat].append(d)
                all_deltas.append(d)
        row_d = float(np.mean(suite_deltas)) if suite_deltas else 0.0
        sign = "+" if row_d >= 0 else ""
        row.append(f"{sign}{row_d:+.1f}")
        print("| " + " | ".join(row) + " |")

    # Column means delta
    col_row = ["**col Δ**"]
    for cat in cats:
        if col_deltas[cat]:
            m = float(np.mean(col_deltas[cat]))
            col_row.append(f"**{'+' if m >= 0 else ''}{m:+.1f}**")
        else:
            col_row.append("—")
    overall = float(np.mean(all_deltas)) if all_deltas else 0.0
    col_row.append(f"**{'+' if overall >= 0 else ''}{overall:+.1f}**")
    print("| " + " | ".join(col_row) + " |")

    print(f"\n**Overall mean delta: {overall:+.2f} pp** "
          f"({args.label_variant} {var.get('overall_mean_pc', '?'):.1f}% vs "
          f"{args.label_baseline} {base.get('overall_mean_pc', '?'):.1f}%)")


if __name__ == "__main__":
    main()
