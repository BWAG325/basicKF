#!/usr/bin/env python3
"""折返跑：CV / CA / 三种 IMM 的误差、模型概率与 RMSE 对比。"""

import csv
import pathlib

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = pathlib.Path(__file__).resolve().parents[1]
CSV = ROOT / "build" / "kf" / "imm_demo.csv"
OUT = ROOT / "build" / "kf" / "imm_demo.png"

WARMUP = 25


def col(rows, key):
    return [float(r[key]) for r in rows]


def rmse(rows, key):
    vals = [float(r[key]) for r in rows[WARMUP:]]
    return (sum(v * v for v in vals) / max(1, len(vals))) ** 0.5 * 100.0


def main():
    rows = list(csv.DictReader(CSV.open()))
    t = col(rows, "t")

    fig = plt.figure(figsize=(13, 8.5))

    ax = fig.add_subplot(2, 3, 1)
    ax.plot(col(rows, "truth_x"), col(rows, "truth_y"), color="#4c78a8", lw=2, label="truth")
    ax.scatter(col(rows, "obs_x"), col(rows, "obs_y"), s=5, color="#9aa0a6", alpha=0.45, label="obs")
    ax.plot(col(rows, "ca9_x"), col(rows, "ca9_y"), color="#e45756", lw=1.3, label="IMM CV9+CA")
    ax.set_title("x-y trajectory (y exaggerated)")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper left")

    ax = fig.add_subplot(2, 3, 2)
    ax.plot(t, [v * 100 for v in col(rows, "err_cv")], color="#f58518", lw=1.0, ls="--", label="CV")
    ax.plot(t, [v * 100 for v in col(rows, "err_ca")], color="#54a24b", lw=1.0, ls="-.", label="CA")
    ax.plot(t, [v * 100 for v in col(rows, "err_cc")], color="#4c78a8", lw=1.2, label="IMM CV+CV")
    ax.plot(t, [v * 100 for v in col(rows, "err_ca9")], color="#e45756", lw=1.2, label="IMM CV9+CA")
    ax.plot(t, [v * 100 for v in col(rows, "err_caca")], color="#b279a2", lw=1.2, label="IMM CA+CA")
    ax.set_title("position error")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("|est - truth| [cm]")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    ax = fig.add_subplot(2, 3, 3)
    ax.plot(t, col(rows, "mu_cc_slow"), color="#54a24b", lw=1.4, label="CV slow")
    ax.plot(t, col(rows, "mu_cc_fast"), color="#b279a2", lw=1.4, label="CV fast")
    ax.axhline(0.5, color="#8b949e", ls=":", lw=1)
    ax.set_ylim(-0.02, 1.02)
    ax.set_title("IMM CV+CV")
    ax.set_xlabel("t [s]")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="center right")

    ax = fig.add_subplot(2, 3, 4)
    ax.plot(t, col(rows, "mu_ca9_cv"), color="#54a24b", lw=1.4, label="CV9 (virtual a=0)")
    ax.plot(t, col(rows, "mu_ca9_ca"), color="#b279a2", lw=1.4, label="CA")
    ax.axhline(0.5, color="#8b949e", ls=":", lw=1)
    ax.set_ylim(-0.02, 1.02)
    ax.set_title("IMM CV9+CA")
    ax.set_xlabel("t [s]")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="center right")

    ax = fig.add_subplot(2, 3, 5)
    ax.plot(t, col(rows, "mu_caca_slow"), color="#54a24b", lw=1.4, label="CA slow")
    ax.plot(t, col(rows, "mu_caca_fast"), color="#b279a2", lw=1.4, label="CA fast")
    ax.axhline(0.5, color="#8b949e", ls=":", lw=1)
    ax.set_ylim(-0.02, 1.02)
    ax.set_title("IMM CA+CA")
    ax.set_xlabel("t [s]")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc="center right")

    ax = fig.add_subplot(2, 3, 6)
    labels = ["CV", "CA", "CV+CV", "CV9+CA", "CA+CA"]
    keys = ["err_cv", "err_ca", "err_cc", "err_ca9", "err_caca"]
    vals = [rmse(rows, k) for k in keys]
    colors = ["#f58518", "#54a24b", "#4c78a8", "#e45756", "#b279a2"]
    bars = ax.bar(labels, vals, color=colors)
    for b, v in zip(bars, vals):
        ax.text(b.get_x() + b.get_width() / 2, v + 0.1, f"{v:.2f}", ha="center", fontsize=9)
    ax.set_title("RMSE (warmup skipped)")
    ax.set_ylabel("cm")
    ax.grid(alpha=0.3, axis="y")

    fig.suptitle("basicKF / shuttle run: CV vs CA vs IMM")
    fig.tight_layout()
    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT, dpi=140)
    print("wrote", OUT)
    for label, v in zip(labels, vals):
        print(f"{label:>7}: {v:6.2f} cm")


if __name__ == "__main__":
    main()
