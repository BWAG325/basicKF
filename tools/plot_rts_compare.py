#!/usr/bin/env python3
"""在 kf_compare 的折返/小陀螺场景上比较 causal vs RTS 的 r/l/h。"""
import pathlib
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams["font.family"] = "Noto Sans CJK JP"
plt.rcParams["axes.unicode_minus"] = False

ROOT = pathlib.Path(__file__).resolve().parents[1]
SC = [
    ("无spin折返", "rm_fold_no_spin"),
    ("小陀螺折返", "rm_shuttle_spin"),
    ("实际加减速", "rm_actual_accel_no_spin"),
    ("原地匀速spin", "rm_inplace_spin_const"),
    ("原地变速spin", "rm_inplace_spin_var"),
]


def load(name):
    return list(csv.DictReader(open(ROOT / f"build/kf/rts_{name}.csv")))


def final(rows, key):
    return float(rows[-1][key])


def main():
    data = {n: load(n) for _, n in SC}
    fig, axes = plt.subplots(2, 3, figsize=(14, 8.6))
    for col, (key, title) in enumerate([("r", "r [m]"), ("l", "l [m]"), ("h", "h [m]")]):
        ax = axes[0, col]
        x = np.arange(len(SC))
        ref = [float(data[n][-1][f"{key}_ref"]) for _, n in SC]
        causal = [final(data[n], f"{key}_causal") for _, n in SC]
        smooth = [final(data[n], f"{key}_smooth") for _, n in SC]
        ax.bar(x - 0.22, ref, 0.22, color="#111111", label="reference")
        ax.bar(x, causal, 0.22, color="#4c78a8", label="causal")
        ax.bar(x + 0.22, smooth, 0.22, color="#e45756", hatch="//", alpha=0.7, label="RTS")
        ax.set_xticks(x, [s[0] for s in SC], rotation=20, ha="right", fontsize=7)
        ax.set_ylabel(title)
        ax.set_title(f"final {key}（causal = RTS）")
        ax.grid(alpha=0.25, axis="y")
        ax.legend(fontsize=7)
        for xi, v in zip(x, causal):
            ax.text(xi, v + (0.005 if v >= 0 else -0.01), f"{v:.3f}", ha="center", fontsize=6)

    # time series for two representative scenarios
    for col, (label, name) in enumerate([SC[0], SC[1]]):
        ax = axes[1, col]
        rows = data[name]
        t = np.array([float(r["t"]) for r in rows])
        for key, color in [("r", "#4c78a8"), ("l", "#54a24b"), ("h", "#e45756")]:
            c = np.array([float(r[f"{key}_causal"]) for r in rows])
            s = np.array([float(r[f"{key}_smooth"]) for r in rows])
            ax.plot(t, c, color=color, lw=1.1, label=f"{key} causal")
            ax.plot(t, s, "--", color=color, lw=1.0, alpha=0.8, label=f"{key} RTS")
        ax.set_xlabel("t [s]")
        ax.set_ylabel("m")
        ax.set_title(f"{label}：causal / RTS")
        ax.grid(alpha=0.25)
        ax.legend(fontsize=6, ncol=2)

    axes[1, 2].axis("off")
    axes[1, 2].text(
        0.0, 0.95,
        "结论（用前面对比 IMM 的同一批场景）\n\n"
        "RTS 的最终 r/l/h 与 causal 完全相同：\n"
        "  无spin折返、 小陀螺折返、\n"
        "  实际加减速、 原地匀速/变速 spin\n"
        "  五组 final 全部一一相等。\n\n"
        "RTS 只把历史回填成最终值；\n"
        "对严格恒定 r/l/h（Q=0）不会提高最终精度。\n\n"
        "另外：这些场景锁定单块板，\n"
        "r/l/h 本身可观测性很弱，causal 估计也会漂；\n"
        "RTS 不能创造可观测性，也不能修模型/关联误差。\n\n"
        "要改善 r/l/h：用多板/多角度、batch LS/EM、\n"
        "多 track 融合；不是靠 RTS。",
        va="top", fontsize=8.6, linespacing=1.5)
    fig.suptitle("RTS 在折返/小陀螺场景上的 r/l/h 效果（与 IMM 对比同一批场景）",
                 fontsize=13, fontweight="bold")
    fig.subplots_adjust(left=0.07, right=0.98, top=0.88, bottom=0.08, hspace=0.55, wspace=0.28)
    out = ROOT / "build/kf/rts_compare.png"
    fig.savefig(out, dpi=150)
    print("wrote", out)


if __name__ == "__main__":
    main()
