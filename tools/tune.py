#!/usr/bin/env python3
"""一条命令：跨工况统一参数调参 + 出图。"""

import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]


def main():
    cfg = sys.argv[1] if len(sys.argv) > 1 else "configs/kf/tune.yaml"
    exe = ROOT / "build" / "kf_tune"
    if not exe.exists():
        print(f"error: {exe} 不存在，先 cmake --build build")
        return 1

    res = subprocess.run([str(exe), f"-c={cfg}"], capture_output=True, text=True)
    sys.stdout.write(res.stdout)
    if res.returncode != 0:
        sys.stderr.write(res.stderr)
        return res.returncode

    json_path = None
    for line in res.stdout.splitlines():
        if line.startswith("json:"):
            json_path = line.split("json:", 1)[1].strip()
    if not json_path:
        print("error: 没有找到 json 输出")
        return 1

    # png 路径从 json 同目录推出来
    png = str(pathlib.Path(json_path).with_suffix(".png"))
    return subprocess.run([sys.executable, str(ROOT / "tools" / "plot_tune.py"),
                           json_path]).returncode


if __name__ == "__main__":
    sys.exit(main())
