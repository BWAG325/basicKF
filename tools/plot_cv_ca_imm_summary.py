#!/usr/bin/env python3
"""CV/CA/CV+CA IMM 在 shuttle / ellipse / hard_rev 三个 sim 场景的汇总图。"""
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
    ("shuttle", ROOT / "build/kf/compare_cv_ca_imm.csv"),
    ("ellipse", ROOT / "build/kf/compare_cv_ca_imm_ellipse.csv"),
    ("hard rev", ROOT / "build/kf/compare_cv_ca_imm_hard.csv"),
]
COLORS = {"CV": "#f58518", "CA": "#54a24b", "IMM": "#4c78a8"}
WARM = 25


def load(p):
    rows = list(csv.DictReader(open(p)))
    return rows


def rmse(rows, key):
    v = np.array([float(r[key]) for r in rows[WARM:]])
    return float(np.sqrt((v * v).mean()) * 100)


def main():
    data = {name: load(p) for name, p in CASES}
    fig = plt.figure(figsize=(13.5, 8.2))
    gs = fig.add_gridspec(2, 3, hspace=0.35, wspace=0.28)

    # grouped RMSE
    ax = fig.add_subplot(gs[0, 0])
    x = np.arange(len(CASES))
    w = 0.25
    for k, (key, label) in enumerate([("err_0", "CV"), ("err_1", "CA"), ("err_2", "IMM")]):
        vals = [rmse(data[name], key) for name, _ in CASES]
        ax.bar(x + (k - 1) * w, vals, width=w, color=COLORS[label], label=label)
        for xi, v in zip(x + (k - 1) * w, vals):
            ax.text(xi, v + 0.05, f"{v:.2f}", ha="center", fontsize=7)
    ax.set_xticks(x, [n for n, _ in CASES])
    ax.set_ylabel("RMSE [cm]")
    ax.set_title("current plate-position RMSE")
    ax.grid(alpha=0.25, axis="y")
    ax.legend(fontsize=8)

    # CA probability
    ax = fig.add_subplot(gs[0, 1])
    for name, _ in CASES:
        rows = data[name]
        t = [float(r["t"]) for r in rows]
        mu = [float(r["mu_2_1"]) for r in rows]
        ax.plot(t, mu, lw=1.4, label=name)
    ax.axhline(0.5, color="#8b949e", ls=":", lw=1)
    ax.set_ylim(-0.02, 1.02)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("P(CA)")
    ax.set_title("IMM 的 CA 模型概率")
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8)

    # error shuttle
    ax = fig.add_subplot(gs[0, 2])
    rows = data["shuttle"]
    t = [float(r["t"]) for r in rows]
    for key, label in [("err_0", "CV"), ("err_1", "CA"), ("err_2", "IMM")]:
        ax.plot(t, [float(r[key]) * 100 for r in rows], color=COLORS[label], lw=1.0, label=label)
    ax.set_ylim(0, 7)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("|err| [cm]")
    ax.set_title("shuttle error")
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8)

    # error hard
    ax = fig.add_subplot(gs[1, 0])
    rows = data["hard rev"]
    t = [float(r["t"]) for r in rows]
    for key, label in [("err_0", "CV"), ("err_1", "CA"), ("err_2", "IMM")]:
        ax.plot(t, [float(r[key]) * 100 for r in rows], color=COLORS[label], lw=1.0, label=label)
    ax.set_ylim(0, 7)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("|err| [cm]")
    ax.set_title("hard reversal error")
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8)

    # error ellipse
    ax = fig.add_subplot(gs[1, 1])
    rows = data["ellipse"]
    t = [float(r["t"]) for r in rows]
    for key, label in [("err_0", "CV"), ("err_1", "CA"), ("err_2", "IMM")]:
        ax.plot(t, [float(r[key]) * 100 for r in rows], color=COLORS[label], lw=1.0, label=label)
    ax.set_ylim(0, 7)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("|err| [cm]")
    ax.set_title("ellipse error")
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8)

    # note
    ax = fig.add_subplot(gs[1, 2])
    ax.axis("off")
    ax.text(
        0.02, 0.95,
        "结论（3D 位置观测，当前板位置 RMSE）\n"
        "shuttle: IMM 2.14 < CA 2.27 < CV 2.53\n"
        "ellipse: CA 2.14 < IMM 2.23 < CV 2.44\n"
        "hard rev: CV 2.02 ≈ IMM 2.02 < CA 2.27\n\n"
        "IMM 只在 CA 概率有意义时占优；\n"
        "hard_rev 的 P(CA) 只有 ~0.18，基本退回 CV。\n"
        "当前板位置误差 ≠ 100ms 提前量。",
        va="top", fontsize=9.5, family="Noto Sans CJK JP", linespacing=1.55)
    fig.suptitle("basicKF: CV / CA / CV+CA IMM（q_jerk=800, ca_tau=0）", fontsize=13, fontweight="bold", y=1.0)
    out = ROOT / "build/kf/cv_ca_imm_summary.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    print("wrote", out)


if __name__ == "__main__":
    main()
