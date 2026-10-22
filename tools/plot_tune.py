#!/usr/bin/env python3
"""统一参数调参结果：每个模型用一组参数跑全部工况的 RMSE 对比。"""

import json
import pathlib
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

COLORS = ["#4c78a8", "#e45756", "#54a24b", "#b279a2", "#f58518", "#72b7b2"]


def main():
    if len(sys.argv) < 2:
        print("usage: plot_tune.py <tune_result.json>")
        return 1

    meta = json.loads(pathlib.Path(sys.argv[1]).read_text())
    scenarios = meta["scenarios"]
    results = meta["results"]

    fig, ax = plt.subplots(figsize=(12.5, 6.0))
    n_models = len(results)
    n_sc = len(scenarios)
    width = 0.8 / n_sc

    for k, sc in enumerate(scenarios):
        xs = [i + (k - (n_sc - 1) / 2) * width for i in range(n_models)]
        vals = [r["per_scenario"][k] for r in results]
        ax.bar(xs, vals, width, label=sc, color=COLORS[k % len(COLORS)])

    # 逐场景各自最优：小黑点
    for i, r in enumerate(results):
        for k in range(n_sc):
            x = i + (k - (n_sc - 1) / 2) * width
            ax.plot([x], [r["per_scenario_best"][k]], marker="_", color="#111", ms=6, mew=1.2)

    labels = [r["model"] + "\n" + r["params"].replace(", ", "\n") for r in results]
    ax.set_xticks(range(n_models))
    ax.set_xticklabels(labels, fontsize=7)
    ax.set_ylabel("RMSE [cm]")
    ax.set_title(f"best common params across scenarios (objective={meta['objective']})", pad=34)
    ax.grid(alpha=0.3, axis="y")
    ax.legend(fontsize=8, ncol=n_sc, loc="lower center", bbox_to_anchor=(0.5, 1.0), frameon=False)

    fig.tight_layout()
    out = pathlib.Path(meta.get("png", pathlib.Path(sys.argv[1]).with_suffix(".png")))
    fig.savefig(out, dpi=150)
    print("wrote", out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
