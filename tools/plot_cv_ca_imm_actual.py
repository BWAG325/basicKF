#!/usr/bin/env python3
"""实际工况：无小陀螺加减速 / 原地匀速小陀螺 / 原地变速小陀螺。"""
import pathlib
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams["font.family"] = "Noto Sans CJK JP"
plt.rcParams["axes.unicode_minus"] = False

ROOT = pathlib.Path(__file__).resolve().parents[1]
CASES = [
    ("加减速\n无小陀螺", ROOT / "build/kf/compare_cv_ca_imm_accel_no_spin.csv"),
    ("原地\n匀速小陀螺", ROOT / "build/kf/compare_cv_ca_imm_spin_const.csv"),
    ("原地\n变速小陀螺", ROOT / "build/kf/compare_cv_ca_imm_spin_var.csv"),
]
COLORS = {"CV": "#f58518", "CA": "#54a24b", "IMM": "#4c78a8"}
WARM = 25


def load(p):
    return list(csv.DictReader(open(p)))


def rmse(rows, key):
    v = np.array([float(r[key]) for r in rows[WARM:]])
    return float(np.sqrt((v * v).mean()) * 100)


def main():
    data = {name: load(p) for name, p in CASES}
    fig = plt.figure(figsize=(13.5, 8.0))
    gs = fig.add_gridspec(2, 3, hspace=0.40, wspace=0.28)

    ax = fig.add_subplot(gs[0, 0])
    x = np.arange(len(CASES))
    w = 0.25
    for k, (key, label) in enumerate([("err_0", "CV"), ("err_1", "CA"), ("err_2", "IMM")]):
        vals = [rmse(data[name], key) for name, _ in CASES]
        ax.bar(x + (k - 1) * w, vals, width=w, color=COLORS[label], label=label)
        for xi, v in zip(x + (k - 1) * w, vals):
            ax.text(xi, v + 0.05, f"{v:.2f}", ha="center", fontsize=8)
    ax.set_xticks(x, [n for n, _ in CASES], fontsize=8)
    ax.set_ylabel("RMSE [cm]")
    ax.set_title("当前板位置 RMSE")
    ax.grid(alpha=0.25, axis="y")
    ax.legend(fontsize=8)

    ax = fig.add_subplot(gs[0, 1])
    for name, _ in CASES:
        rows = data[name]
        ax.plot([float(r["t"]) for r in rows], [float(r["mu_2_1"]) for r in rows], lw=1.4, label=name)
    ax.axhline(0.5, color="#8b949e", ls=":", lw=1)
    ax.set_ylim(-0.02, 1.02)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("P(CA)")
    ax.set_title("IMM 的 CA 概率")
    ax.grid(alpha=0.25)
    ax.legend(fontsize=7)

    ax = fig.add_subplot(gs[0, 2])
    rows = data["加减速\n无小陀螺"]
    t = [float(r["t"]) for r in rows]
    for key, label in [("err_0", "CV"), ("err_1", "CA"), ("err_2", "IMM")]:
        ax.plot(t, [float(r[key]) * 100 for r in rows], color=COLORS[label], lw=1.0, label=label)
    ax.set_ylim(0, 6)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("|err| [cm]")
    ax.set_title("加减速/无小陀螺 error")
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8)

    for col, name in enumerate(["原地\n匀速小陀螺", "原地\n变速小陀螺"]):
        ax = fig.add_subplot(gs[1, col])
        rows = data[name]
        t = [float(r["t"]) for r in rows]
        for key, label in [("err_0", "CV"), ("err_1", "CA"), ("err_2", "IMM")]:
            ax.plot(t, [float(r[key]) * 100 for r in rows], color=COLORS[label], lw=1.0, label=label)
        ax.set_ylim(0, 6)
        ax.set_xlabel("t [s]")
        ax.set_ylabel("|err| [cm]")
        ax.set_title(name.replace("\n", " ") + " error")
        ax.grid(alpha=0.25)
        ax.legend(fontsize=8)

    ax = fig.add_subplot(gs[1, 2])
    ax.axis("off")
    ax.text(
        0.02, 0.95,
        "结论（3D 位置观测，当前板位置 RMSE）\n"
        "加减速无小陀螺：CA 1.63 < IMM 1.72 < CV 1.85\n"
        "原地匀速小陀螺：IMM 1.95 < CV 1.97 << CA 2.44\n"
        "原地变速小陀螺：IMM 2.20 = CV 2.20 << CA 3.58\n\n"
        "q_jerk=10 更贴合实际：CA 在无小陀螺加速段\n"
        "明显好，但在原地小陀螺段会发散；IMM 能退回 CV。\n"
        "IMM 在变速小陀螺没有收益：它只有平移 CA，\n"
        "yaw/w 仍是常值模型；需要单独的 yaw CA/Singer。",
        va="top", fontsize=9.0, linespacing=1.55)
    fig.suptitle("basicKF · 实际工况：无小陀螺加减速 / 原地匀速/变速小陀螺",
                 fontsize=13, fontweight="bold", y=1.0)
    out = ROOT / "build/kf/cv_ca_imm_actual_summary.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    print("wrote", out)


if __name__ == "__main__":
    main()
