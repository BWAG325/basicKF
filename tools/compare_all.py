#!/usr/bin/env python3
"""跑多套对比配置，汇总成一张分组柱状图（模型 × 工况）。"""

import csv
import json
import pathlib
import subprocess
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_CONFIGS = [
    "configs/kf/shuttle.yaml",
    "configs/kf/shuttle_spin.yaml",
    "configs/kf/ellipse_spin.yaml",
    "configs/kf/ellipse_var_spin.yaml",
    "configs/kf/cruise_spin.yaml",
    "configs/kf/cruise_var_spin.yaml",
]
OUT = ROOT / "build" / "kf" / "compare_all.png"


def rmse(rows, key, warmup):
    vals = [float(r[key]) for r in rows[warmup:]]
    return (sum(v * v for v in vals) / max(1, len(vals))) ** 0.5 * 100


def run(config):
    exe = ROOT / "build" / "kf_compare"
    res = subprocess.run([str(exe), f"-c={config}"], capture_output=True, text=True)
    if res.returncode != 0:
        sys.stderr.write(res.stderr)
        raise SystemExit(res.returncode)

    meta_path = None
    for line in res.stdout.splitlines():
        if line.startswith("meta:"):
            meta_path = line.split("meta:", 1)[1].strip()
    meta = json.loads(pathlib.Path(meta_path).read_text())
    rows = list(csv.DictReader(open(meta["csv"])))
    warmup = int(meta.get("warmup", 0))

    return {
        "label": pathlib.Path(meta["sim"]).stem.replace("rm_", ""),
        "models": [m["name"] for m in meta["models"]],
        "rmse": [rmse(rows, f"err_{i}", warmup) for i in range(len(meta["models"]))],
    }


def main():
    configs = sys.argv[1:] or DEFAULT_CONFIGS
    runs = [run(c) for c in configs]

    models = runs[0]["models"]
    for r in runs[1:]:
        if r["models"] != models:
            raise SystemExit("各配置的模型列表不一致，无法画分组柱状图")

    n_models = len(models)
    n_runs = len(runs)
    width = 0.8 / n_runs

    fig, ax = plt.subplots(figsize=(11, 5.9))
    colors = ["#4c78a8", "#e45756", "#54a24b", "#b279a2", "#f58518", "#72b7b2",
              "#eeca3b", "#9d755d"]
    for k, r in enumerate(runs):
        xs = [i + (k - (n_runs - 1) / 2) * width for i in range(n_models)]
        ax.bar(xs, r["rmse"], width, label=r["label"], color=colors[k % len(colors)])

    ax.set_xticks(range(n_models))
    ax.set_xticklabels(models, fontsize=9)
    ax.set_ylabel("RMSE [cm]")
    ax.set_title("RMSE across scenarios", pad=36)
    ax.grid(alpha=0.3, axis="y")
    ax.legend(fontsize=8, ncol=n_runs, loc="lower center", bbox_to_anchor=(0.5, 1.0), frameon=False)
    fig.tight_layout()
    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT, dpi=150)
    print("wrote", OUT)

    print("\n" + "model".ljust(18) + "".join(r["label"].rjust(18) for r in runs))
    for i, m in enumerate(models):
        print(m.ljust(18) + "".join(f"{r['rmse'][i]:17.2f} " for r in runs))


if __name__ == "__main__":
    main()
