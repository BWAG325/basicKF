#!/usr/bin/env python3
"""折返跑：无小陀螺 vs 小陀螺，按刹停/掉头/再加速分段看 IMM。"""
import pathlib
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams["font.family"] = "Noto Sans CJK JP"
plt.rcParams["axes.unicode_minus"] = False

ROOT = pathlib.Path(__file__).resolve().parents[1]
COLORS = {"CV": "#f58518", "CA": "#54a24b", "IMM": "#4c78a8"}
WARM = 25
CASES = [
    ("无小陀螺折返", ROOT / "build/kf/compare_cv_ca_imm_fold_no_spin.csv"),
    ("小陀螺折返", ROOT / "build/kf/compare_cv_ca_imm_fold_spin.csv"),
]
PHASES = [
    ("刹停", [2, 7, 12], [3, 8, 13]),
    ("掉头", [3, 8, 13], [4, 9, 14]),
    ("再加速", [4, 9, 14], [5, 10, 15]),
]


def load(p):
    return list(csv.DictReader(open(p)))


def rmse(rows, key):
    v = np.array([float(r[key]) for r in rows[WARM:]])
    return float(np.sqrt((v * v).mean()) * 100)


def phase_rows(rows, starts, ends):
    out = []
    for r in rows:
        t = float(r["t"])
        if any(s <= t < e for s, e in zip(starts, ends)):
            out.append(r)
    return out


def main():
    data = {name: load(p) for name, p in CASES}
    fig = plt.figure(figsize=(14, 8.8))
    gs = fig.add_gridspec(2, 3, hspace=0.38, wspace=0.26)

    for col, (name, _) in enumerate(CASES):
        rows = data[name]
        t = [float(r["t"]) for r in rows]

        ax = fig.add_subplot(gs[0, col])
        for key, label in [("err_0", "CV"), ("err_1", "CA"), ("err_2", "IMM")]:
            ax.plot(t, [float(r[key]) * 100 for r in rows], color=COLORS[label], lw=0.9, label=label)
        for s, e in [(2, 5), (7, 10), (12, 15)]:
            ax.axvspan(s, e, color="#e45756", alpha=0.07)
        for s, e in [(2, 3), (7, 8), (12, 13)]:
            ax.axvspan(s, e, color="#f58518", alpha=0.10)
        for s, e in [(3, 4), (8, 9), (13, 14)]:
            ax.axvspan(s, e, color="#b279a2", alpha=0.10)
        ax.set_ylim(0, 7)
        ax.set_xlabel("t [s]")
        ax.set_ylabel("|err| [cm]")
        ax.set_title(f"{name} · 误差")
        ax.grid(alpha=0.25)
        ax.legend(fontsize=8)

        ax = fig.add_subplot(gs[1, col])
        labels = []
        for pname, starts, ends in PHASES:
            x = phase_rows(rows, starts, ends)
            vals = [rmse(x, k) for k in ["err_0", "err_1", "err_2"]]
            labels.append(pname)
            ax.bar(np.arange(3) + len(labels) * 0.0, [0, 0, 0], alpha=0)  # keep
        # grouped by phase
        width = 0.22
        xpos = np.arange(len(PHASES))
        for k, (key, label) in enumerate([("err_0", "CV"), ("err_1", "CA"), ("err_2", "IMM")]):
            vals = []
            for pname, starts, ends in PHASES:
                vals.append(rmse(phase_rows(rows, starts, ends), key))
            ax.bar(xpos + (k - 1) * width, vals, width=width, color=COLORS[label], label=label)
            for xi, v in zip(xpos + (k - 1) * width, vals):
                ax.text(xi, v + 0.03, f"{v:.2f}", ha="center", fontsize=6.5)
        ax.set_xticks(xpos, [p[0] for p in PHASES])
        ax.set_ylabel("RMSE [cm]")
        ax.set_title(f"{name} · 分阶段 RMSE")
        ax.grid(alpha=0.25, axis="y")
        ax.legend(fontsize=8)

    ax = fig.add_subplot(gs[1, 2])
    ax.axis("off")
    ax.text(
        0.0, 0.95,
        "折返段结论（当前板位置 RMSE，cm）\n\n"
        "无小陀螺折返（q_jerk=10）：\n"
        "  刹停 CV1.89 / CA1.74 / IMM1.79\n"
        "  掉头 CV1.89 / CA1.97 / IMM1.87\n"
        "  再加速 CV1.89 / CA1.94 / IMM1.90\n"
        "  整段 CV1.87 / CA1.80 / IMM1.82\n\n"
        "小陀螺折返（q_jerk=800）：\n"
        "  刹停 CV2.24 / CA2.28 / IMM2.24\n"
        "  掉头 CV3.13 / CA2.28 / IMM2.38\n"
        "  再加速 CV2.47 / CA2.32 / IMM2.31\n"
        "  整段 CV2.53 / CA2.27 / IMM2.29\n\n"
        "yaw 按角度混合、加速度不再被 CV 补 0 之后，\n"
        "IMM 仍贴着当前更好的那个单模型，\n"
        "没有单独拉开折返段。",
        va="top", fontsize=8.4, linespacing=1.45)
    fig.suptitle("basicKF · 折返段：无小陀螺 vs 小陀螺（CV / CA / CV+CA IMM）",
                 fontsize=13, fontweight="bold", y=1.0)
    out = ROOT / "build/kf/cv_ca_imm_fold_summary.png"
    fig.savefig(out, dpi=150, bbox_inches="tight")
    print("wrote", out)


if __name__ == "__main__":
    main()
