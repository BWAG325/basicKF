#!/usr/bin/env python3
"""R 估错鲁棒性：RMSE vs 假设的 R 倍数。"""
import csv, pathlib, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

path = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else "build/kf/robust.csv")
rows = list(csv.DictReader(path.open()))
xs = [float(r["r_scale"]) for r in rows]
ys = [float(r["rmse_cm"]) for r in rows]
fig, ax = plt.subplots(figsize=(6.5, 4.2))
ax.plot(xs, ys, marker="o", color="#4c78a8", lw=1.6)
ax.set_xscale("log")
ax.set_xlabel("assumed R / true R")
ax.set_ylabel("RMSE [cm]")
ax.set_title("CV: robustness to R mismatch")
ax.grid(alpha=0.3, which="both")
fig.tight_layout()
out = path.with_name("robust.png")
fig.savefig(out, dpi=150)
print("wrote", out)
