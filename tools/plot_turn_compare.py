#!/usr/bin/env python3
"""只对比折返段（刹停+掉头+反向加速）与其余段。"""
import csv, json, pathlib, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

meta = json.loads(pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else
                               "build/kf/compare_shuttle_spin.csv.json").read_text())
rows = list(csv.DictReader(open(meta["csv"])))
warmup = int(meta.get("warmup", 0))
models = meta["models"]
names = [m["name"] for m in models]
t = np.array([float(r["t"]) for r in rows])
err = np.array([[float(r[f"err_{i}"]) * 100 for r in rows] for i in range(len(models))])

# 每个端点的折返窗口：刹车开始 → 反向加速结束（3-5, 8-10, 13-15）
windows = [(2.5, 5.0), (7.5, 10.0), (12.5, 15.0)]
turn = np.zeros(len(t), dtype=bool)
for a, b in windows:
    turn |= (t >= a) & (t <= b)
rest = (~turn) & (np.arange(len(t)) >= warmup)

def rmse(mask, mi):
    return float(np.sqrt(np.mean(err[mi][mask] ** 2)))

turn_vals = [rmse(turn, i) for i in range(len(models))]
rest_vals = [rmse(rest, i) for i in range(len(models))]

fig, ax = plt.subplots(figsize=(8, 4.6))
x = np.arange(len(models)); w = 0.36
b1 = ax.bar(x - w/2, turn_vals, w, label="turn region (brake+turn+accel)")
b2 = ax.bar(x + w/2, rest_vals, w, label="rest")
for bars in (b1, b2):
    for b in bars:
        ax.text(b.get_x()+b.get_width()/2, b.get_height()+0.03, f"{b.get_height():.2f}",
                ha="center", fontsize=8)
ax.set_xticks(x); ax.set_xticklabels(names, fontsize=8)
ax.set_ylabel("RMSE [cm]"); ax.set_ylim(0, max(turn_vals+rest_vals)*1.2)
ax.set_title("turn region vs rest (80 Hz, shuttle_spin)")
ax.grid(alpha=0.3, axis="y"); ax.legend(fontsize=8)
fig.tight_layout()
out = pathlib.Path("build/kf/turn_compare.png")
fig.savefig(out, dpi=150)
print("wrote", out)
for n, a, b in zip(names, turn_vals, rest_vals):
    print(f"{n:12s} turn {a:.2f}  rest {b:.2f}")
