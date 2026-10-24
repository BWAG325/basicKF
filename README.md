# basicKF / sim —— RM 车辆 3D 运动仿真器 + 整车 EKF

增量式 3D 仿真器（当前状态 + 控制量 + dt），以及一套 **复刻 auto_aim 的 11 维整车 EKF**。
旧的线性/CT/SPIN/IMM 模型已移除，需要时看 git 历史。

## 模型（CV）

状态：

```text
[ cx, vx, cy, vy, cz, vz, yaw, yaw_rate, r, dr, dh ]
```

- `cx/cy/cz`：旋转中心，`vx/vy/vz`：速度
- `yaw, yaw_rate`：朝向与角速度
- `r, dr, dh`：装甲半径、长短轴半径差、高低板高度差

过程：线性匀速 + piecewise white noise（`q_xyz`、`q_yaw`），角度折回。
观测：装甲板三维位置；`armor = center − r_eff·(cos a, sin a)`，`a = yaw + id·2π/N`，
`id∈{1,3}` 用 `r+dr` 和 `cz+dh`。

实现：[cv.hpp](include/kf/models/cv.hpp)，EKF 核心 [ekf.hpp](include/kf/ekf.hpp)。

## 8 个工况

`shuttle` / `shuttle_spin` / `ellipse_spin` / `ellipse_var_spin` / `cruise_spin` /
`cruise_var_spin` / `offset_circle_spin` / `ellipse_ring_spin`

## 结果（2 cm 观测噪声）

| 指标 | 结果 |
|---|---|
| 位置 RMSE（8 工况） | 2.79–3.17 cm |
| 80 ms 提前量预测 | 8.90 cm（120 ms 13.7 cm） |
| R 估错 0.25×–10× | 2.86–3.56 cm |

## 工具

```bash
./build/kf_compare -c=configs/kf/shuttle_spin.yaml     # 单工况
python3 tools/compare_all.py                           # 8 工况汇总
./build/kf_future && python3 tools/plot_future.py      # 提前量
./build/kf_robust                                      # R 估错鲁棒性
```

## 构建 / 测试

```bash
cmake -S . -B build && cmake --build build -j4
cd build && ctest
```

依赖：C++20、Eigen3、yaml-cpp、OpenCV。

## 仿真器

- 姿态用四元数，速度沿车体 +x；
- `min_dt`：积分步长上限；观测时刻表独立（可非均匀）；
- 控制：`accel`、`jerk`、`angular_rate`、`spin_rate`、`duration`；
- 预设：`idle / straight / circle`，`plan` 支持循环（循环时重置物理状态）；
- 相对路径统一相对工程根解析（CLion / 命令行一致）。

详细仿真说明见 git 历史中的旧 README，或直接看 `include/sim/`。
