#!/usr/bin/env python3
"""折返跑（匀速小陀螺）：按位置分箱和运动状态的误差。"""
import csv, json, pathlib, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

meta_path = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else "build/kf/compare_shuttle_spin.csv.json")
meta = json.loads(meta_path.read_text())
rows = list(csv.DictReader(open(meta["csv"])))
warmup = int(meta.get("warmup", 0))
models = meta["models"]
names = [m["name"] for m in models]

t = np.array([float(r["t"]) for r in rows])
x = np.array([float(r["truth_x"]) for r in rows])
err = np.array([[float(r[f"err_{i}"]) * 100 for r in rows] for i in range(len(models))])

# 工况段：2 cruise,1 brake,1 turn,1 accel,2 cruise,1 brake,1 turn,1 accel,2 cruise,1 brake,1 turn,1 accel,1 cruise
segs = [(2.0, "cruise"), (1.0, "brake"), (1.0, "turn"), (1.0, "accel"),
        (2.0, "cruise"), (1.0, "brake"), (1.0, "turn"), (1.0, "accel"),
        (2.0, "cruise"), (1.0, "brake"), (1.0, "turn"), (1.0, "accel"),
        (1.0, "cruise")]
bounds, acc_t = [], 0.0
for dur, label in segs:
    bounds.append((acc_t, acc_t + dur, label))
    acc_t += dur
def state_of(tt):
    for a, b, lab in bounds:
        if a <= tt < b:
            return lab
    return "cruise"
states = [state_of(v) for v in t]

fig, axes = plt.subplots(1, 2, figsize=(13, 4.8))

# 位置分箱
ax = axes[0]
bins = np.linspace(x.min(), x.max(), 13)
centers = 0.5 * (bins[:-1] + bins[1:])
for mi, name in enumerate(names):
    vals = []
    for k in range(len(bins) - 1):
        mask = (x >= bins[k]) & (x < bins[k + 1]) & (np.arange(len(x)) >= warmup)
        vals.append(np.sqrt(np.mean(err[mi][mask] ** 2)) if mask.any() else np.nan)
    ax.plot(centers, vals, marker="o", ms=3, lw=1.4, label=name)
ax.set_xlabel("truth x [m]")
ax.set_ylabel("RMSE [cm]")
ax.set_title("error vs position")
ax.grid(alpha=0.3)
ax.legend(fontsize=8)

# 状态分箱
ax = axes[1]
order = ["cruise", "accel", "brake", "turn"]
width = 0.8 / len(models)
for mi, name in enumerate(names):
    vals = []
    for st in order:
        mask = np.array([s == st for s in states]) & (np.arange(len(t)) >= warmup)
        vals.append(np.sqrt(np.mean(err[mi][mask] ** 2)) if mask.any() else 0.0)
    xs = np.arange(len(order)) + (mi - (len(models) - 1) / 2) * width
    ax.bar(xs, vals, width, label=name)
    for xx, vv in zip(xs, vals):
        ax.text(xx, vv + 0.05, f"{vv:.2f}", ha="center", fontsize=7)
ax.set_xticks(range(len(order)))
ax.set_xticklabels(["cruise", "accel", "brake", "turn"])
ax.set_ylabel("RMSE [cm]")
ax.set_title("error by motion state")
ax.grid(alpha=0.3, axis="y")
ax.legend(fontsize=8)

fig.suptitle(f"shuttle + spin · {pathlib.Path(meta['sim']).name}")
fig.tight_layout()
out = pathlib.Path("build/kf/error_state.png")
fig.savefig(out, dpi=150)
print("wrote", out)
for mi, name in enumerate(names):
    print(name, "total", round(float(np.sqrt(np.mean(err[mi][warmup:] ** 2))), 2))
