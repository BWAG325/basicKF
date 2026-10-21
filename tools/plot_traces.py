#!/usr/bin/env python3
"""把 sim_demo 的 CSV 轨迹画成交互式 HTML（并用 matplotlib 出静态图）。

用法：
    python3 tools/plot_traces.py
输出：
    build/traces/traces.html   交互式（滚轮缩放 / 拖动 / 悬停 / 时间滑块）
    build/traces/traces.png    静态四宫格
"""

import csv
import json
import math
import pathlib

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = pathlib.Path(__file__).resolve().parents[1]
OUT = ROOT / "build" / "traces"

SCENES = [
    ("straight", ROOT / "build/traces/straight.csv", "#4c78a8"),
    ("circle", ROOT / "build/traces/circle.csv", "#e45756"),
    ("weave", ROOT / "build/traces/weave.csv", "#54a24b"),
    ("random", ROOT / "build/traces/random.csv", "#b279a2"),
]


def load(path):
    frames = []
    with path.open() as fh:
        for row in csv.DictReader(fh):
            t = float(row["t"])
            x, y = float(row["p_x"]), float(row["p_y"])
            vx, vy = float(row["vx"]), float(row["vy"])
            yaw = float(row["yaw"])
            spin = float(row["spin_phase"])
            valid = row["obs_valid"] == "1"
            ox, oy = float(row["obs_x"]), float(row["obs_y"])
            armor = int(row["truth_armor"])
            z = float(row["p_z"])
            oz = float(row["obs_z"])
            pitch = float(row["pitch"])
            roll = float(row["roll"])
            frames.append([t, x, y, vx, vy, yaw, spin, ox, oy, 1 if valid else 0, armor, z, oz,
                           pitch, roll])
    return frames


def main():
    data = {"scenes": []}
    for name, path, color in SCENES:
        if not path.exists():
            raise SystemExit(f"缺少 {path}，先跑 sim_demo 生成")
        data["scenes"].append({"name": name, "color": color, "frames": load(path)})

    OUT.mkdir(parents=True, exist_ok=True)

    # ---- 静态四宫格 ----
    fig = plt.figure(figsize=(11.5, 10.5))
    for i, scene in enumerate(data["scenes"]):
        ax = fig.add_subplot(2, 2, i + 1, projection="3d")
        fr = scene["frames"]
        xs = [f[1] for f in fr]
        ys = [f[2] for f in fr]
        zs = [f[11] for f in fr]
        ax.plot(xs, ys, zs, color=scene["color"], lw=1.8)
        ox = [f[7] for f in fr if f[9]]
        oy = [f[8] for f in fr if f[9]]
        oz = [f[12] for f in fr if f[9]]
        ax.scatter(ox, oy, oz, s=6, color="#9aa0a6", alpha=0.4, linewidths=0)
        ax.plot([xs[0]], [ys[0]], [zs[0]], "o", ms=6, color=scene["color"])
        ax.plot([xs[-1]], [ys[-1]], [zs[-1]], "s", ms=6, color=scene["color"])
        ax.set_title(scene["name"])
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_zlabel("z [m]")
        ax.grid(alpha=0.3)
    fig.suptitle("basicKF / sim - 3D trajectories (line = truth, dots = observations)")
    fig.tight_layout()
    fig.savefig(OUT / "traces.png", dpi=140)
    print("wrote", OUT / "traces.png")

    # ---- 交互式 HTML ----
    payload = json.dumps(data, separators=(",", ":"))
    for template_name, out_name in (("traces_template.html", "traces.html"),
                                    ("traces3d_template.html", "traces3d.html")):
        template = (ROOT / "tools" / template_name).read_text()
        (OUT / out_name).write_text(template.replace("__DATA__", payload))
        print("wrote", OUT / out_name)


if __name__ == "__main__":
    main()
