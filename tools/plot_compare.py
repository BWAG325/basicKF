#!/usr/bin/env python3
"""读 kf_compare 的 meta JSON + CSV，画误差对比 / RMSE / 轨迹 / 模型概率。"""

import csv
import json
import pathlib
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

COLORS = ["#f58518", "#54a24b", "#4c78a8", "#e45756", "#b279a2", "#72b7b2", "#eeca3b"]


def rmse(rows, key, warmup):
    vals = [float(r[key]) for r in rows[warmup:]]
    return (sum(v * v for v in vals) / max(1, len(vals))) ** 0.5 * 100.0


def main():
    if len(sys.argv) < 2:
        print("usage: plot_compare.py <meta.json>")
        return 1

    meta = json.loads(pathlib.Path(sys.argv[1]).read_text())
    rows = list(csv.DictReader(open(meta["csv"])))
    t = [float(r["t"]) for r in rows]
    models = meta["models"]
    warmup = int(meta.get("warmup", 0))

    n_imm = sum(1 for m in models if m["imm"])
    n_rows = 3 if n_imm else 2
    fig = plt.figure(figsize=(13, 6.5 if n_imm == 0 else 10))
    gs = GridSpec(n_rows, 3, figure=fig, height_ratios=[1.05] + [1] * (n_rows - 1))

    ax = fig.add_subplot(gs[0, :])
    for i, m in enumerate(models):
        ax.plot(t, [float(r[f"err_{i}"]) * 100 for r in rows], lw=1.2,
                color=COLORS[i % len(COLORS)], label=m["name"])
    ax.set_title("position error")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("|est - truth| [cm]")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, ncol=2)

    ax = fig.add_subplot(gs[1, 0])
    vals = [rmse(rows, f"err_{i}", warmup) for i in range(len(models))]
    bars = ax.bar(range(len(models)), vals,
                  color=[COLORS[i % len(COLORS)] for i in range(len(models))])
    for b, v in zip(bars, vals):
        ax.text(b.get_x() + b.get_width() / 2, v + 0.05, f"{v:.2f}", ha="center", fontsize=8)
    ax.set_xticks(range(len(models)))
    ax.set_xticklabels([m["name"] for m in models], rotation=30, ha="right", fontsize=8)
    ax.set_title("RMSE")
    ax.set_ylabel("cm")
    ax.grid(alpha=0.3, axis="y")

    ax = fig.add_subplot(gs[1, 1:])
    ax.plot([float(r["truth_x"]) for r in rows], [float(r["truth_y"]) for r in rows],
            color="#4c78a8", lw=2, label="truth")
    ax.scatter([float(r["obs_x"]) for r in rows], [float(r["obs_y"]) for r in rows],
               s=4, color="#9aa0a6", alpha=0.4, label="obs")
    for i, m in enumerate(models):
        ax.plot([float(r[f"r{i}_x"]) for r in rows], [float(r[f"r{i}_y"]) for r in rows],
                color=COLORS[i % len(COLORS)], lw=1.2, label=m["name"])
    ax.set_title("x-y trajectory (y exaggerated)")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="upper left")

    imm_models = [(i, m) for i, m in enumerate(models) if m["imm"]]
    for k, (i, m) in enumerate(imm_models[:3]):
        ax = fig.add_subplot(gs[2, k])
        for j, sub in enumerate(m["subs"]):
            ax.plot(t, [float(r[f"mu_{i}_{j}"]) for r in rows], lw=1.3,
                    color=COLORS[j % len(COLORS)], label=sub)
        ax.axhline(0.5, color="#8b949e", ls=":", lw=1)
        ax.set_ylim(-0.02, 1.02)
        ax.set_title(m["name"], fontsize=9)
        ax.set_xlabel("t [s]")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7, loc="center right")

    sim_name = pathlib.Path(meta["sim"]).name
    fig.suptitle(f"basicKF compare · {sim_name}")
    fig.tight_layout()
    out = pathlib.Path(meta["png"])
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=140)
    print("wrote", out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
