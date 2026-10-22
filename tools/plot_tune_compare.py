#!/usr/bin/env python3
"""对比两份 tune_result.json（例如 2cm vs 1cm 噪声）的平均 RMSE。"""

import json
import pathlib
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

COLORS = ["#4c78a8", "#e45756", "#54a24b", "#b279a2"]


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    labels = []
    for a in sys.argv[1:]:
        if a.startswith("--labels="):
            labels = a.split("=", 1)[1].split(",")
    if len(args) < 2:
        print("usage: plot_tune_compare.py a.json b.json [--labels=2cm,1cm]")
        return 1

    runs = [json.loads(pathlib.Path(p).read_text()) for p in args]
    if not labels:
        labels = [f"noise {r.get('measurement_sigma', '?')}m" for r in runs]

    models = [r["model"] for r in runs[0]["results"]]
    params = [r["params"] for r in runs[0]["results"]]
    n = len(models)
    width = 0.8 / len(runs)

    fig, ax = plt.subplots(figsize=(12, 5.8))
    for k, (run, lab) in enumerate(zip(runs, labels)):
        xs = [i + (k - (len(runs) - 1) / 2) * width for i in range(n)]
        vals = [r["mean"] for r in run["results"]]
        ax.bar(xs, vals, width, label=lab, color=COLORS[k % len(COLORS)])

    ax.set_xticks(range(n))
    ax.set_xticklabels([m + "\n" + p.replace(", ", "\n") for m, p in zip(models, params)], fontsize=6.5)
    ax.set_ylabel("mean RMSE [cm]")
    ax.set_title("best common params: noise level comparison")
    ax.grid(alpha=0.3, axis="y")
    ax.legend(fontsize=8)
    fig.tight_layout()

    out = pathlib.Path(args[0]).with_name("tune_compare.png")
    fig.savefig(out, dpi=150)
    print("wrote", out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
