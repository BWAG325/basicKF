#!/usr/bin/env python3
"""RTS 终值冻住 r/l/h 之后，实时卡尔曼有没有变好。"""
import csv
import math
import pathlib

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams["font.family"] = "Noto Sans CJK JP"
plt.rcParams["axes.unicode_minus"] = False

ROOT = pathlib.Path(__file__).resolve().parents[1]
CSV = ROOT / "build/kf/rts_feasibility.csv"
OUT = ROOT / "build/kf/rts_feasibility.png"

SERIES = [
    ("causal", "实时联合", "#4c78a8"),
    ("replay", "终值重跑", "#54a24b"),
    ("oracle", "真值几何", "#111111"),
    ("rts", "RTS 平滑", "#e45756"),
    ("nominal", "错误标称", "#f2cf5b"),
    ("freeze", "1.5 s 冻住", "#b279a2"),
]


def rmse(rows, kind):
    c = np.array([float(r[f"center_{kind}"]) for r in rows])
    return math.sqrt(np.mean(c ** 2)) * 1000.0


def main():
    rows = list(csv.DictReader(CSV.open()))
    t = np.array([float(r["t"]) for r in rows])
    fig, axes = plt.subplots(2, 2, figsize=(14, 8.2))

    ax = axes[0, 0]
    for key, color in (("r", "#4c78a8"), ("l", "#54a24b"), ("h", "#e45756")):
        true = np.array([float(r[f"{key}_true"]) for r in rows])
        causal = np.array([float(r[f"{key}_causal"]) for r in rows])
        smooth = np.array([float(r[f"{key}_rts"]) for r in rows])
        ax.plot(t, true, color=color, lw=1.6, label=f"{key} 真值")
        ax.plot(t, causal, color=color, lw=1.0, alpha=0.85, label=f"{key} 实时")
        ax.plot(t, smooth, color=color, lw=1.2, ls="--", label=f"{key} RTS")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("m")
    ax.set_title("r / l / h：RTS 整段等于实时终点")
    ax.grid(alpha=0.25)
    ax.legend(fontsize=6, ncol=3)

    ax = axes[0, 1]
    for key, label, color in (
        ("causal", "实时联合", "#4c78a8"),
        ("replay", "用 RTS/实时终值重跑", "#54a24b"),
        ("rts", "RTS 平滑轨迹", "#e45756"),
    ):
        y = np.array([float(r[f"center_{key}"]) for r in rows]) * 1000.0
        ax.plot(t, y, color=color, lw=1.15, label=label)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("中心误差 [mm]")
    ax.set_title("车体中心误差")
    ax.set_ylim(0, 80)
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8)

    ax = axes[1, 0]
    labels = [s[1] for s in SERIES]
    vals = [rmse(rows, s[0]) for s in SERIES]
    colors = [s[2] for s in SERIES]
    x = np.arange(len(SERIES))
    ax.bar(x, vals, color=colors, width=0.72)
    for xi, v in zip(x, vals):
        ax.text(xi, v + 0.6, f"{v:.1f}", ha="center", fontsize=8)
    ax.set_xticks(x, labels, rotation=18, ha="right", fontsize=8)
    ax.set_ylabel("中心 RMSE [mm]")
    ax.set_title("整段 6 s 的中心 RMSE")
    ax.set_ylim(0, max(vals) * 1.18)
    ax.grid(alpha=0.25, axis="y")

    axes[1, 1].axis("off")
    axes[1, 1].text(
        0.0,
        0.98,
        "不可行：RTS 优化出的恒定量，喂不回实时滤波。\n\n"
        "r/l/h 的过程噪声是 0。RTS 终点就是实时终点，\n"
        "差别只有 1e-13 m。整段历史被回填成这个终点，\n"
        "并没有一个更准的常数。\n\n"
        "用这个终点冻住 r/l/h 再从头跑实时滤波，\n"
        "中心 RMSE 19.0 → 18.7 mm，和已知真值几何一样。\n"
        "1.5 s 后在线冻住也是 19.0 mm。\n"
        "错误标称 0.22/0/0 冻住则是 49.4 mm：\n"
        "常数错了会伤实时滤波，但正确的常数\n"
        "实时联合估计自己就会收敛到。\n\n"
        "RTS 把中心 RMSE 降到 10.1 mm，是因为平滑了\n"
        "过去的位置和 yaw，用了未来量测。\n"
        "最后一拍与实时滤波相同，不能拿来当当前输出。",
        va="top",
        fontsize=9.0,
        linespacing=1.45,
    )
    fig.suptitle(
        "原地小陀螺：用 RTS 的 r/l/h 重跑实时卡尔曼（6 s，位置噪声 20 mm）",
        fontsize=13,
        fontweight="bold",
    )
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT, dpi=150)
    print("wrote", OUT)


if __name__ == "__main__":
    main()
