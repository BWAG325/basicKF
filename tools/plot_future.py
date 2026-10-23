#!/usr/bin/env python3
"""提前量预测误差曲线：RMSE vs 预测时长。"""
import csv
import pathlib
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

path = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else "build/kf/future.csv")
rows = list(csv.DictReader(path.open()))
models = []
data = {}
for r in rows:
    m = r["model"]
    if m not in data:
        data[m] = []
        models.append(m)
    data[m].append((float(r["horizon_ms"]), float(r["rmse_cm"])))

fig, ax = plt.subplots(figsize=(8, 4.8))
colors = ["#f58518", "#54a24b", "#4c78a8", "#e45756"]
for m, c in zip(models, colors):
    xs = [p[0] for p in data[m]]
    ys = [p[1] for p in data[m]]
    ax.plot(xs, ys, marker="o", ms=3, lw=1.5, color=c, label=m)

ax.axhspan(0, 6, color="#54a24b", alpha=0.08)
ax.set_xlabel("prediction horizon [ms]")
ax.set_ylabel("position RMSE [cm]")
ax.set_title("future armor-position prediction error")
ax.grid(alpha=0.3)
ax.legend()
fig.tight_layout()
out = path.with_name("future.png")
fig.savefig(out, dpi=150)
print("wrote", out)
