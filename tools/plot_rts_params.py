#!/usr/bin/env python3
"""RTS 对恒定 r/l/h 的估计：静态、全板观测、truth center/yaw 固定。"""
import pathlib
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams["font.family"] = "Noto Sans CJK JP"
plt.rcParams["axes.unicode_minus"] = False

ROOT = pathlib.Path(__file__).resolve().parents[1]
CSV = ROOT / "build/kf/rts_params.csv"
OUT = ROOT / "build/kf/rts_params.png"


def main():
    rows = list(csv.DictReader(CSV.open()))
    t = np.array([float(r["t"]) for r in rows])
    fig, axes = plt.subplots(2, 3, figsize=(14, 7.6))
    names = [("r", "r [m]", "短轴半径 r"), ("l", "l [m]", "长短轴差 l"), ("h", "h [m]", "高低差 h")]
    for col, (key, ylabel, title) in enumerate(names):
        ax = axes[0, col]
        true = np.array([float(r[f"{key}_true"]) for r in rows])
        causal = np.array([float(r[f"{key}_causal"]) for r in rows])
        smooth = np.array([float(r[f"{key}_smooth"]) for r in rows])
        ax.plot(t, true, color="#111111", lw=2.0, label="true")
        ax.plot(t, causal, color="#4c78a8", lw=1.4, label="causal")
        ax.plot(t, smooth, "--", color="#e45756", lw=1.4, label="RTS")
        ax.set_xlabel("t [s]")
        ax.set_ylabel(ylabel)
        ax.set_title(f"{title}（最终值相同）")
        ax.grid(alpha=0.25)
        ax.legend(fontsize=7)

    ax = axes[1, 0]
    labels = ["r", "l", "h"]
    causal_err = []
    smooth_err = []
    for key in labels:
        true = np.array([float(r[f"{key}_true"]) for r in rows])
        c = np.array([float(r[f"{key}_causal"]) for r in rows])
        s = np.array([float(r[f"{key}_smooth"]) for r in rows])
        causal_err.append(abs(c[-1] - true[-1]) * 1000)
        smooth_err.append(abs(s[-1] - true[-1]) * 1000)
    x = np.arange(3)
    ax.bar(x - 0.18, causal_err, 0.36, color="#4c78a8", label="causal")
    ax.bar(x + 0.18, smooth_err, 0.36, color="#e45756", label="RTS")
    for xi, v in zip(x - 0.18, causal_err):
        ax.text(xi, v + 0.2, f"{v:.1f}", ha="center", fontsize=7)
    for xi, v in zip(x + 0.18, smooth_err):
        ax.text(xi, v + 0.2, f"{v:.1f}", ha="center", fontsize=7)
    ax.set_xticks(x, labels)
    ax.set_ylabel("final |error| [mm]")
    ax.set_title("最终误差（Q=0 时相同）")
    ax.grid(alpha=0.25, axis="y")
    ax.legend(fontsize=8)

    for ax in [axes[1, 1], axes[1, 2]]:
        ax.axis("off")
    axes[1, 1].text(
        0.0, 0.95,
        "结论（全板、center/yaw 固定为真值，只估 r/l/h）\n\n"
        "causal 终点 = RTS 终点。\n"
        "这次 h 收到约 -0.060 m；\n"
        "r 停在约 0.220 m，l 停在约 0。\n"
        "仿真装甲在椭圆上，半径随转角变，\n"
        "不是滤波器里「奇偶板各一个常数半径」。\n\n"
        "RTS（F=I, Q=0）把整段历史回填成终点，\n"
        "不会比 causal 终点更接近真值。",
        va="top", fontsize=8.7, linespacing=1.5)
    axes[1, 2].text(
        0.0, 0.95,
        "若要让终点比 causal 更准：\n"
        "  - 允许参数慢变（Q>0），或\n"
        "  - 用 batch least squares / EM / 因子图。\n\n"
        "实际建议：\n"
        "  - 恒定尺寸用 track 的 causal 终点做 profile；\n"
        "  - RTS 只回填历史，不提升终点。\n\n"
        "本图固定 center/yaw 为真值，观测全部四块板。\n"
        "仿真 armor_a/b 是椭圆，板半径随转角变化；\n"
        "滤波器的 l 却是奇偶板的固定半径差。\n"
        "所以 l 收不到 0.04 m，r 会停在平均半径附近。\n"
        "h 与奇偶板高度差一致，可以收到 -0.06 m。",
        va="top", fontsize=8.7, linespacing=1.5)

    fig.suptitle("基本实验：RTS 对恒定 r/l/h 的估计（严格恒定量 Q=0）",
                 fontsize=13, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT, dpi=150)
    print("wrote", OUT)


if __name__ == "__main__":
    main()
