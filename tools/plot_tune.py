#!/usr/bin/env python3
"""八工况统一参数精度：模型 × 工况热力图 + 平均 RMSE 条形图。"""

import json
import pathlib
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.gridspec import GridSpec


def main():
    if len(sys.argv) < 2:
        print("usage: plot_tune.py <tune_result.json>")
        return 1

    meta = json.loads(pathlib.Path(sys.argv[1]).read_text())
    scenarios = meta["scenarios"]
    results = meta["results"]
    models = [r["model"] for r in results]
    mat = np.array([r["per_scenario"] for r in results])
    mean = np.array([r["mean"] for r in results])

    fig = plt.figure(figsize=(12.5, 6.8))
    gs = GridSpec(2, 2, width_ratios=[3.2, 1.0], height_ratios=[1, 0.05], wspace=0.08, hspace=0.55)

    ax = fig.add_subplot(gs[0, 0])
    im = ax.imshow(mat, aspect="auto", cmap="YlOrRd", vmin=1.5, vmax=max(3.3, mat.max()))
    ax.set_xticks(range(len(scenarios)))
    ax.set_xticklabels(scenarios, rotation=35, ha="right", fontsize=8)
    ax.set_yticks(range(len(models)))
    ax.set_yticklabels(models, fontsize=8)
    for i in range(len(models)):
        for j in range(len(scenarios)):
            ax.text(j, i, f"{mat[i, j]:.2f}", ha="center", va="center", fontsize=6.5,
                    color="black" if mat[i, j] < 2.8 else "white")
    ax.set_title("RMSE [cm] · 8 scenarios × models (best common params)", fontsize=10)
    cax = fig.add_subplot(gs[1, 0])
    fig.colorbar(im, cax=cax, orientation="horizontal", label="RMSE [cm]")

    ax = fig.add_subplot(gs[0, 1])
    y = np.arange(len(models))
    ax.barh(y, mean, color="#4c78a8")
    ax.set_yticks(y)
    ax.set_yticklabels([])
    ax.invert_yaxis()
    ax.set_xlabel("mean RMSE [cm]")
    ax.grid(alpha=0.3, axis="x")
    for i, v in enumerate(mean):
        ax.text(v + 0.02, i, f"{v:.2f}", va="center", fontsize=7)
    ax.set_xlim(0, max(3.6, mean.max() * 1.1))
    ax.set_title("mean", fontsize=9)

    fig.tight_layout()
    out = pathlib.Path(meta.get("png", pathlib.Path(sys.argv[1]).with_suffix(".png")))
    fig.savefig(out, dpi=150)
    print("wrote", out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
