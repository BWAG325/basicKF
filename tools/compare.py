#!/usr/bin/env python3
"""一条命令跑对比：kf_compare（按 yaml 选模型）→ 生成 CSV/meta → 画图。"""

import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]


def main():
    if len(sys.argv) < 2:
        print("usage: python3 tools/compare.py <configs/kf/xxx.yaml>")
        return 1

    config = sys.argv[1]
    exe = ROOT / "build" / "kf_compare"
    if not exe.exists():
        print(f"error: {exe} 不存在，先 cmake --build build")
        return 1

    res = subprocess.run([str(exe), f"-c={config}"], capture_output=True, text=True)
    sys.stdout.write(res.stdout)
    sys.stdout.flush()
    if res.returncode != 0:
        sys.stderr.write(res.stderr)
        return res.returncode

    meta = None
    for line in res.stdout.splitlines():
        if line.startswith("meta:"):
            meta = line.split("meta:", 1)[1].strip()

    if not meta:
        print("error: kf_compare 没有输出 meta 路径")
        return 1

    return subprocess.run([sys.executable, str(ROOT / "tools" / "plot_compare.py"), meta]).returncode


if __name__ == "__main__":
    sys.exit(main())
