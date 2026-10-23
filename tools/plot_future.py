#!/usr/bin/env python3
"""提前量：各模型误差曲线 + 80ms 时模型×工况热力图。"""
import csv
import pathlib
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.gridspec import GridSpec

path = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else "build/kf/future.csv")
rows = list(csv.DictReader(path.open()))

scenarios, models, horizons = [], [], []
data = {}
for r in rows:
    sc, m, h, v = r["scenario"], r["model"], float(r["horizon_ms"]), float(r["rmse_cm"])
    data[(sc, m, h)] = v
    if sc not in scenarios: scenarios.append(sc)
    if m not in models: models.append(m)
    if h not in horizons: horizons.append(h)
horizons.sort()

fig = plt.figure(figsize=(13, 4.8))
gs = GridSpec(1, 2, width_ratios=[1, 1.5], wspace=0.18)

ax = fig.add_subplot(gs[0, 0])
colors = ["#f58518", "#54a24b", "#4c78a8", "#e45756", "#b279a2"]
for m, c in zip(models, colors):
    ys = [np.mean([data[(s, m, h)] for s in scenarios]) for h in horizons]
    ax.plot(horizons, ys, marker="o", ms=3, lw=1.5, color=c, label=m)
ax.set_xlabel("prediction horizon [ms]")
ax.set_ylabel("mean RMSE [cm]")
ax.set_title("prediction error (8 scenarios)")
ax.grid(alpha=0.3)
ax.legend(fontsize=8)

ax = fig.add_subplot(gs[0, 1])
h80 = min(horizons, key=lambda x: abs(x - 80))
mat = np.array([[data[(s, m, h80)] for s in scenarios] for m in models])
im = ax.imshow(mat, aspect="auto", cmap="YlOrRd")
ax.set_xticks(range(len(scenarios)))
ax.set_xticklabels(scenarios, rotation=35, ha="right", fontsize=7)
ax.set_yticks(range(len(models)))
ax.set_yticklabels(models, fontsize=8)
for i in range(len(models)):
    for j in range(len(scenarios)):
        ax.text(j, i, f"{mat[i, j]:.1f}", ha="center", va="center", fontsize=6.5)
ax.set_title(f"RMSE @ {h80:.0f} ms (cm)")
fig.colorbar(im, ax=ax, fraction=0.03, pad=0.02)

fig.tight_layout()
out = path.with_name("future.png")
fig.savefig(out, dpi=150)
print("wrote", out)
