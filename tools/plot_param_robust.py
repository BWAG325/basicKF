#!/usr/bin/env python3
import csv, pathlib
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

rows = list(csv.DictReader(open("build/kf/param_robust.csv")))
models = []
for r in rows:
    if r["model"] not in models: models.append(r["model"])
colors = ["#4c78a8", "#f58518", "#54a24b", "#e45756"]

fig, axes = plt.subplots(1, 2, figsize=(12, 4.6), sharey=True)
for ax, axis, title in [(axes[0], "r", "R mismatch (assumed / true)"),
                        (axes[1], "q", "Q mismatch (assumed / true)")]:
    for i, m in enumerate(models):
        xs, ys = [], []
        for r in rows:
            if r["model"] == m and r["axis"] == axis:
                xs.append(float(r["scale"])); ys.append(float(r["rmse_cm"]))
        order = sorted(range(len(xs)), key=lambda k: xs[k])
        ax.plot([xs[k] for k in order], [ys[k] for k in order], marker="o", ms=3,
                lw=1.5, color=colors[i % len(colors)], label=m)
    ax.set_xscale("log")
    ax.set_xlabel(title)
    ax.set_ylabel("RMSE [cm]")
    ax.grid(alpha=0.3, which="both")
    ax.legend(fontsize=8)
axes[0].set_title("observation noise")
axes[1].set_title("process noise")
fig.suptitle("parameter robustness (4 scenarios, 80 Hz)")
fig.tight_layout()
fig.savefig("build/kf/param_robust.png", dpi=150)
print("wrote build/kf/param_robust.png")
