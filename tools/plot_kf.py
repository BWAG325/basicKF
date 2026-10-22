#!/usr/bin/env python3
"""把 kf_demo 的 CSV 画成四宫格：轨迹 / 位置误差 / NIS / 速度。"""

import csv
import json
import pathlib

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = pathlib.Path(__file__).resolve().parents[1]
CSV = ROOT / "build" / "kf" / "cv_demo.csv"
OUT = ROOT / "build" / "kf" / "kf_demo.png"


def main():
    rows = list(csv.DictReader(CSV.open()))
    t = [float(r["t"]) for r in rows]
    truth = [(float(r["truth_x"]), float(r["truth_y"]), float(r["truth_z"])) for r in rows]
    obs = [(float(r["obs_x"]), float(r["obs_y"]), float(r["obs_z"])) for r in rows]
    est = [(float(r["est_x"]), float(r["est_y"]), float(r["est_z"])) for r in rows]
    err = [float(r["err"]) for r in rows]
    nis = [float(r["nis"]) for r in rows]

    speed_truth = [((float(r["truth_vx"]) ** 2 + float(r["truth_vy"]) ** 2 + float(r["truth_vz"]) ** 2) ** 0.5)
                   for r in rows]
    speed_est = [((float(r["est_vx"]) ** 2 + float(r["est_vy"]) ** 2 + float(r["est_vz"]) ** 2) ** 0.5)
                 for r in rows]

    fig = plt.figure(figsize=(12, 9))

    ax = fig.add_subplot(2, 2, 1)
    ax.plot([p[0] for p in truth], [p[1] for p in truth], color="#4c78a8", lw=2, label="truth")
    ax.scatter([p[0] for p in obs], [p[1] for p in obs], s=6, color="#9aa0a6", alpha=0.5, label="obs")
    ax.plot([p[0] for p in est], [p[1] for p in est], color="#e45756", lw=1.2, label="est")
    ax.set_title("x-y path")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(loc="upper left")

    ax = fig.add_subplot(2, 2, 2)
    err_cm = [v * 100 for v in err]
    ax.plot(t, err_cm, color="#e45756", lw=1)
    ax.axhline(sum(err_cm) / len(err_cm), color="#e45756", ls="--", lw=1,
               label=f"mean {sum(err_cm)/len(err_cm):.2f} cm")
    ax.set_title("position error")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("|est - truth| [cm]")
    ax.grid(alpha=0.3)
    ax.legend()

    ax = fig.add_subplot(2, 2, 3)
    ax.plot(t, nis, color="#54a24b", lw=1)
    ax.axhline(7.81, color="#999", ls="--", lw=1, label="chi2(3) 95%")
    ax.axhline(3.0, color="#54a24b", ls=":", lw=1, label="expect 3")
    ax.set_title("NIS")
    ax.set_xlabel("t [s]")
    ax.grid(alpha=0.3)
    ax.legend()

    ax = fig.add_subplot(2, 2, 4)
    ax.plot(t, speed_truth, color="#4c78a8", lw=2, label="truth")
    ax.plot(t, speed_est, color="#e45756", lw=1.2, label="est")
    ax.set_title("speed")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("|v| [m/s]")
    ax.grid(alpha=0.3)
    ax.legend()

    fig.suptitle("basicKF / CV Kalman demo")
    fig.tight_layout()
    OUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT, dpi=140)
    print("wrote", OUT)

    # ---- 交互式 HTML ----
    payload = {
        "config": str(CSV.relative_to(ROOT)),
        "t": t,
        "truth": truth,
        "obs": obs,
        "est": est,
        "err": err,
        "nis": nis,
        "speed_truth": speed_truth,
        "speed_est": speed_est,
    }
    template = (ROOT / "tools" / "kf_demo_template.html").read_text()
    html_path = OUT.parent / "kf_demo.html"
    html_path.write_text(template.replace("__DATA__", json.dumps(payload, separators=(",", ":"))))
    print("wrote", html_path)


if __name__ == "__main__":
    main()
