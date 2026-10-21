# basicKF / sim —— RM 车辆运动模拟器

纯数据的三维运动模拟器：给定任意时刻 `t`，直接算出车体真值 + 每块装甲板真值 + 带噪观测。
不依赖相机、不依赖图像，也不依赖卡尔曼部分。

## 构建 / 运行

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j4

./build/sim_demo --config configs/sim/rm_straight_spin.yaml --t 2.2
./build/sim_demo --config configs/sim/rm_straight_spin.yaml --dump 0 5 0.01 > run.csv
./build/sim_contract_test          # 或 cd build && ctest
```

## 对外接口

```cpp
sim::SimSetup setup = sim::loadSimSetup("configs/sim/xxx.yaml");   // 场景 + 传感器
const sim::Scene& scene = setup.scene;
sim::GaussianSensor sensor(setup.sensor);

sim::RobotTruth tr = scene.truthAt(t);                 // 纯函数、任意 t、可重入
sim::ObservationTruth anno;                            // 仅评估用，滤波器禁止读
sim::Observation obs = sensor.observeAt(tr, t, &anno);
```

- `truthAt(t)`：`t` 会被 clamp 到 `[0, duration]`；不抛异常、不改内部状态。
- `observeAt(...)`：随机数由 `(seed, 采样序号, 通道)` 哈希决定，**与求值顺序无关**。
- `obs` 里**没有**装甲板编号（真实 PnP 也拿不到）；`anno.armor_index` 只用于评估。

## 计算时机（重要）

构建期**只登记段**（时长 / jump / primitive），一次数值运算都不做，也不缓存任何入口状态。
`at(t)` 在查询时才从初值起，把 `t` 所在段之前的段逐段推过去，再解析求本段：

```text
at(t):  t ——二分——> 段 i
        s = initial
        for k < i:  s = addDelta(s, jump_k);  s = prim_k.eval(s, dur_k)
        s = addDelta(s, jump_i);  return prim_i.eval(s, t - t0_i)
```

- 复杂度 = O(`t` 之前的段数)，不是 O(`t/dt`)，也没有逐帧累加；
- 没有缓存、没有可变状态 → 乱序查询、重复查询、多线程查询都一致；
- 实测 5001 帧（含观测）14 ms；`tests` 里的 `testQueryTimeCompute` 用计数 primitive
  钉死这条契约：`add()` 阶段 eval 次数必须为 **0**，t 落在第 i 段时折叠次数必须为 **i+1**。

## 约定

| 项 | 约定 |
|---|---|
| 坐标系 | 世界系，右手，z 向上；单位 m / s / rad |
| 平移 | `chassis.p` 是车体旋转轴上的点，z 通常取装甲板中心高度 |
| 装甲板 | 中心 = `p + R_z(ring + i·2π/n)·(radius, 0, 0) + (0,0,armor_dz)` |
| 环角 | `ring = yaw + spin_phase`，两种自转语义都成立 |
| 自转语义 | `body_rotation`：整车转，`chassis.yaw` 含自转；`armor_orbit`：车体朝向独立 |
| 相位 | `spin_phase` 对外折到 (-π, π]，`spin_phase_raw` 连续累加 |

## YAML 结构

```yaml
robot:   {armors, armor_radius, armor_dz, armor_width, armor_height, body_radius, spin_mode}
initial: {position: [x,y,z], velocity: [vx,vy,vz], yaw, yaw_rate, spin_phase, spin_rate}
translation:            # 平移自由度
  - {type: hold,  duration: 0.5}
  - {type: accel, duration: 1.0, accel: 2.5}                 # 可负（减速到 0 后停住）
  - {type: line,  duration: 1.8, speed: 2.5, heading: 0.0}   # speed/heading 省略 = 继承
  - {type: arc,   duration: 1.2, speed: 2.5, omega: 1.2}     # 恒定航向角速度
  - {type: brake, duration: 0.5, decel: 5.0}                 # 沿当前速度方向刹停
yaw:                   # 车体朝向（与平移解耦）
  - {type: hold, duration: 5.0}     # 另有 rate / ramp
spin:                  # 装甲板自转
  - {type: off,  duration: 0.5}     # 另有 rate / ramp（from 省略 = 继承）
sensor: {dt, seed, position_sigma: [sx,sy,sz], yaw_sigma, dropout, max_range,
         facing_min_cos, camera: {enabled, position, target, fov_half_deg, output_in_camera_frame}}
```

分段之间默认按「上一段出口状态」续接，所以 `speed/heading/rate` 省略即 C1 连续；
需要真实存在的跳变（例如受击掉速）就在该段写 `jump_velocity: [dx, dy, dz]`。

## 契约测试（`tests/sim_contract_test.cpp`）

这些不是「跑通就算过」的测试，而是接口必须永远成立的性质：

1. 乱序求值与顺序求值逐位一致；
2. 构建期零计算：`add()` 期间 eval 次数为 0；查询时只折叠 `t` 之前的段（i+1 次）；
3. 平移/旋转 primitive 的解析式（直线、等加速、圆弧）与闭式解一致；
4. `v == dp/dt`、`a == dv/dt`（中心差分对拍）；
5. `spin_phase_raw == ∫ω dt`（数值积分对拍）；
6. C1 场景零告警；运动段后接 `hold` 必须报未声明跳变；显式 `jump` 不告警；
7. 跨 2π 时装甲板位置连续；`t` 越界 clamp；
8. 装甲板几何（半径、90° 间隔、法向与 yaw 一致）；
9. 两种自转语义下装甲环角度一致、`chassis.yaw` 语义按配置不同；
10. 观测噪声均值/标准差与配置一致、`R` 由观测层给出；
11. 观测与求值顺序无关、同 t 重复求值一致；
12. 丢帧比例 ≈ 配置值、`triggeredAt` 只在采样网格上触发；
13. 相机系输出时 `|p|` 等于到相机距离且 `z > 0`；
14. 示例 YAML 能装配出无告警场景。

## 目前故意没做的事

- **PnP 误差模型**：现在是对称高斯 + 距离无关 + 无偏。后面替换点只有 `GaussianSensor::observeAt`。
- **多板关联**：观测里没有编号，靠 `yaw` 在 `2π/n` 等价类里做关联——等滤波层再实现。
- **遮挡 / 运动模糊 / 曝光时间**：相机模型只做了 FOV + 法向朝向 + 量程门限。
- **卡尔曼层与可视化**：完全不在这一批。
