# basicKF / sim —— RM 车辆 3D 运动仿真器

按「**当前状态 + 控制量 + dt**」推进的增量式仿真器：不预算轨迹，每调用一次才算一步。
姿态是**四元数**，速度沿车体 +x，装甲板随车体姿态旋转。

## 轨迹可视化

```bash
# 先用 sim_demo 导出 CSV（每帧真值 + 观测）
./build/sim_demo -c=configs/sim/rm_circle.yaml --duration=6 --min-dt=0.0005 > build/traces/circle.csv
python3 tools/plot_traces.py
```

生成三个文件（都在 `build/traces/`）：

| 文件 | 内容 |
|---|---|
| `traces.png` | 静态四宫格 3D 轨迹 |
| `traces.html` | 交互式俯视图（缩放/悬停/时间滑块） |
| `traces3d.html` | **交互式 3D 视图**（拖动旋转、滚轮缩放、Shift+拖动平移） |

3D 页面是手写的投影 + 画家算法，无外部依赖，直接浏览器打开即可。

## CV 卡尔曼 demo

```bash
./build/kf_demo                 # 6s 匀速场景，打印汇总
./build/kf_demo --table         # 逐帧表
./build/kf_demo --csv > build/kf/cv_demo.csv
python3 tools/plot_kf.py        # 生成 build/kf/kf_demo.png + kf_demo.html（CSV 回放）
# 实时版：build/kf/kf_live.html（浏览器里跑模拟器 + 观测 + CV 滤波，可重置/换预设）
```

- 场景：[configs/sim/rm_linear.yaml](configs/sim/rm_linear.yaml)（车体不转，同一块装甲板始终可见）
- 滤波器：`kf::CvKalman`，只吃观测位置
- 参数：`--sigma-accel`（过程噪声）、`--sigma-pos`（覆盖观测噪声）、`--duration`、`--warmup`

## CA、CJ、CV9 与 IMM

### 模型

| 模型 | 文件 | 状态 | 说明 |
|---|---|---|---|
| CV | [cv.hpp](include/kf/models/cv.hpp) | `[p, v]` | 匀速 |
| CV9 | [cv9.hpp](include/kf/models/cv9.hpp) | `[p, v, a]` | CV + 恒为 0 的虚拟加速度；`setState` 强制 a 清零 |
| CA | [ca.hpp](include/kf/models/ca.hpp) | `[p, v, a]` | 匀加速，白噪声 jerk |
| CJ | [cj.hpp](include/kf/models/cj.hpp) | `[p, v, a, j]` | 匀加加速度，白噪声 snap |
| IMM | [imm.hpp](include/kf/imm.hpp) | — | 支持不同状态维度（嵌入公共空间） |

CA / CJ 的 Q（单轴，乘 σ²）：

```text
CA:  [dt⁵/20  dt⁴/8  dt³/6]          CJ:  [dt⁷/252  dt⁶/72  dt⁵/30  dt⁴/24]
     [dt⁴/8   dt³/3  dt²/2]               [dt⁶/72   dt⁵/20  dt⁴/8   dt³/6 ]
     [dt³/6   dt²/2  dt   ]               [dt⁵/30   dt⁴/8   dt³/3   dt²/2 ]
                                          [dt⁴/24   dt³/6   dt²/2   dt    ]
```

### 过程噪声：物理先验，不按数据调

- 观测噪声用真实值：`R = σz² = 0.02²`（对比配置里的 `measurement_sigma: 0.02`）；
- 小陀螺 ω = 6 rad/s、装甲半径 r = 0.22 m：
  - 向心加速度 `ω²r ≈ 7.9 m/s²`，加上线性加速度上限 2 → **CV 快模型 σa = 10**；
  - `jerk ≈ ω³r ≈ 48` → **CA 的 σj = 50**；
  - `snap ≈ ω⁴r ≈ 285` → **CJ 的 σsnap = 300**；
- IMM 转移矩阵对角 0.95。

这些值在六个工况里**保持不变**，不是逐场景调出来的。

### 六个工况

| 配置 | 场景 |
|---|---|
| [shuttle](configs/kf/shuttle.yaml) | 折返跑，不转 |
| [shuttle_spin](configs/kf/shuttle_spin.yaml) | 折返跑 + 小陀螺 |
| [ellipse_spin](configs/kf/ellipse_spin.yaml) | 椭圆 + 小陀螺，匀速 |
| [ellipse_var_spin](configs/kf/ellipse_var_spin.yaml) | 椭圆 + 小陀螺，**变速**（±40%，每圈 2 次） |
| [cruise_spin](configs/kf/cruise_spin.yaml) | 匀速直线 + 小陀螺 |
| [cruise_var_spin](configs/kf/cruise_var_spin.yaml) | 直线 + 小陀螺，**变速**（1 → 2.5 → 0.25 → 2.25 → 0） |

椭圆用反馈控制器 `EllipseController`：按 θ 的曲率给 ω，变速时用实际速度算 ω、用 accel 跟踪期望速度，
轨迹偏离 `x²/a² + y²/b² = 1` 在 4% 以内（有契约测试）。

```bash
python3 tools/compare.py configs/kf/ellipse_var_spin.yaml   # 单工况
python3 tools/compare_all.py                                # 六个工况汇总
```

### 结果（RMSE，cm，物理先验固定）

| 模型 | shuttle | shuttle_spin | ellipse_spin | ellipse_var_spin | cruise_spin | cruise_var_spin |
|---|---|---|---|---|---|---|
| CA 50 | 3.05 | 3.06 | 2.99 | 3.04 | 3.24 | 3.05 |
| CJ 300 | 3.05 | 3.07 | 2.99 | 3.02 | 3.23 | 3.03 |
| IMM CV+CV | **2.38** | 3.04 | **2.86** | 2.94 | 3.11 | **2.79** |
| IMM CV9+CA | 2.39 | 3.11 | 2.99 | 3.08 | 3.33 | 2.97 |
| IMM CA+CA | 2.67 | **3.00** | 2.93 | **2.98** | **3.16** | 2.91 |

结论：

- 固定物理先验后，**CV+CV 整体最好**：不转时 2.38，自转/变速时 2.79–3.11；
- CA / CJ 很稳（2.99–3.24），但单模型没有 IMM 的组合优势；
- CJ 和 CA 几乎打平——在这个噪声水平下，多一阶 jerk 状态带来的收益被噪声吃掉了；
- 变速与否对排名影响不大，误差都在观测噪声底（2 cm）附近。

### 统一参数调参（kf_tune）

```bash
python3 tools/tune.py                  # 默认 configs/kf/tune.yaml
```

- 六套工况的观测各采集一次（[kf_tune.cpp](apps/kf_tune.cpp)）；
- 每个模型在网格上搜一组**全工况共用**的参数，目标默认“平均 RMSE 最小”（配置里可改 `objective: max`）；
- 输出 [tune_result.json](build/kf/tune_result.json) + 图，并给出“逐工况各自最优”的对照（图里的小黑横线）。

统一参数下的结果：

| 模型 | 最优共用参数 | mean | worst |
|---|---|---|---|
| CV | σa = 10 | 2.90 | 3.08 |
| CA | jerk = 20 | 2.94 | 3.10 |
| CJ | σs = 200 | 3.03 | 3.17 |
| IMM CV+CV | 0.01 / 10 | **2.85** | 3.11 |
| IMM CV9+CA | 0.01 / 20 | 2.87 | 3.15 |
| IMM CA+CA | 0.05 / 20 | 2.89 | 3.13 |

- 统一参数下 **IMM CV+CV 仍最好**（2.85），单 CV 2.90；
- 逐工况最优和统一参数的差距主要在 `shuttle`（CV 2.41 vs 2.76，IMM 2.32 vs 2.37），
  说明“一套通用参数”的代价主要来自直线工况；
- 慢模型的参数被推到网格下界（0.01 / 0.05）：直线段希望它尽量“硬”，机动交给快模型。

### 噪声 1 cm 对照

```bash
python3 tools/tune.py configs/kf/tune_1cm.yaml
python3 tools/plot_tune_compare.py build/kf/tune_result.json build/kf/tune_result_1cm.json --labels=2cm,1cm
```

`tune_1cm.yaml` 里 `sensor_sigma: 0.01`（同时覆盖仿真观测噪声）和 `measurement_sigma: 0.01`（滤波器 R）。

| 模型 | 2 cm | 1 cm |
|---|---|---|
| CV | 2.90 | 1.53 |
| CA | 2.94 | 1.53 |
| CJ | 3.03 | 1.56 |
| IMM CV+CV | **2.85** | **1.51** |
| IMM CV9+CA | 2.87 | **1.50** |
| IMM CA+CA | 2.89 | 1.51 |

- RMSE 大约随噪声**线性减半**；
- **最优参数几乎不变**（σ=10、jerk=20、snap=200、慢模型 0.01/0.05、快模型 10/20）；
- 模型之间的差距从 0.18 cm 缩到 0.06 cm，基本进入噪声底；
- `shuttle` 的 headroom 也缩小（CV 1.49 vs 逐场景最优 1.31，IMM 1.28 vs 1.25）。

测试：`ctest` 有五项：`sim_contract`（含椭圆/变速椭圆轨迹）、`imm`、`ca`、`cj`。

## 两个 dt，别混

| 名字 | 在哪 | 作用 |
|---|---|---|
| `sim.min_dt` | `Simulator` | **积分步长上限**：`advanceTo` 按它把时间切小步，位置用 Simpson 积分 |
| `sensor.schedule` | 观测时刻表 | **两次观测的间隔**：名义间隔 + 抖动，或显式间隔列表；和 `min_dt` 完全独立 |

## 构建 / 运行

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j4

./build/sim_demo                                        # 无参数：默认配置，推进到 t=2.2 打一帧
./build/sim_demo -c=configs/sim/rm_random.yaml --t=1.0  # 换配置
./build/sim_demo --duration=2.0 --min-dt=0.001          # 按观测时刻表输出 CSV
cd build && ctest
```

参数用 `cv::CommandLineParser`（与 `auto_aim_/tests` 同一套写法），OpenCV 的约定是 `key=value`：

```bash
sim_demo -c=configs/sim/rm_straight_spin.yaml --t=2.2
sim_demo --duration=5 --min-dt=0.001
```

写错成 `-c path`（空格）会被解析成布尔 flag，程序会直接提示正确写法。

**路径解析规则**（和 `auto_aim_/tools/yaml.hpp` 的 `project_path()` 一致）：

- CMake 把工程根目录编进程序（`SIM_PROJECT_SOURCE_DIR`）；
- 相对路径一律拼到工程根，而不是当前工作目录；
- 绝对路径原样使用。

所以 CLion（工作目录通常是 `cmake-build-*`）和命令行（工作目录是工程根）启动行为一致：

```bash
./build/sim_demo -c=configs/sim/rm_circle.yaml   # 在工程根
cd build && ./sim_demo -c=configs/sim/rm_circle.yaml  # 一样能跑
```

## 运行模型

```cpp
auto setup = sim::loadSimSetup("configs/sim/xxx.yaml");
sim::Simulator sim(setup.geometry, setup.initial, std::move(setup.controller), setup.min_dt);
sim::GaussianSensor sensor(setup.sensor);

const auto times = sim::makeObservationTimes(setup.sensor, frames);
for (std::size_t k = 0; k < times.size(); ++k) {
    sim::RobotTruth tr = sim.advanceTo(times[k]);                  // 精确停在观测时刻
    sim::Observation obs = sensor.observeAt(tr, times[k], k);      // k = 第几帧
}
```

- `advanceTo(t)`：推进到绝对时刻，内部按 `min_dt` 与控制段边界切分；
- `step(dt)`：相对推进，等价于 `advanceTo(t + dt)`；
- `step(dt, control)`：显式控制，忽略 `duration`、不消耗计划队列（键盘用这个）。

## 运动模型（3D）

| 量 | 定义 |
|---|---|
| 位置 / 速度 | 世界系 `p, v`；**速度始终沿车体 +x**（无侧滑） |
| 姿态 | 单位四元数 `q`（body → world），角速度 `ω` 在 **body 系** |
| 速率 | 标量 `speed = v·(q·x̂)`，可被 accel / jerk 改变，减到 0 停住不倒车 |
| 装甲环 | 车体系内绕 z 排一圈，环角 = `spin_phase` + i·90°，再被 `q` 旋转到世界系 |

积分：

```text
q(t+τ) = q(t) ⊗ exp(ω·τ)          // body 系恒角速度，精确解
s(τ)   = s0 + accel·τ + ½·jerk·τ²
p     += ∫ s(τ)·(q(τ)·x̂) dτ      // Simpson 三点，步长 = min_dt
```

所以 `min_dt` 现在**真正影响精度**（越小越准），但同一控制段内按不同 `dt` 推进的结果仍然一致。

## 控制量（都在车体系）

| 字段 | 含义 | 取 0 时 |
|---|---|---|
| `accel` | 沿车体 +x 的加速度 [m/s²] | 匀速 |
| `jerk` | 沿车体 +x 的加加速度 [m/s³] | 匀加速 |
| `angular_rate` | body 系角速度 `[roll, pitch, yaw]` [rad/s] | 姿态不变 |
| `spin_rate` | 装甲环相对车体的自转 [rad/s] | 环不转 |
| `duration` | 本段持续多久 [s] | —— |

YAML 里 `angular_rate: [x, y, z]` 也可以拆成 `roll_rate / pitch_rate / turn_rate` 三个标量。
小陀螺就是 `angular_rate: [0, 0, 9]`（整车绕 z 转），和 `spin_rate`（环相对车体转）是两件事。

## 预设模式

```yaml
preset:
  name: circle        # idle / straight / circle
  speed: 2.0
  radius: 1.0         # 负值 = 顺时针
  accel: 2.0
  duration: 6.0       # forever=true 时只限制加速段
  forever: true       # 默认 true：加速完成后一直绕圈，不会停
```

- `forever: true`（默认）：最后一段时长是无穷，可以一直 `advanceTo` 下去，适合反复看波形；
- `forever: false`：跑完 `duration` 后回到零控制（滑行/停）；
- `random` 模式本来就是无限生成的，不受此开关影响；
- `plan` 模式默认是有限列表；加 `loop: true` 后**每圈重置回初值重放**（见下）。

加新预设：在 `src/motion/preset.cpp` 写一个生成函数，再在 `makePresetControls` 和 `presetNames()` 里各加一行。

## plan 循环与接缝检查

```yaml
loop: true
plan:
  - {duration: 1.0, accel: 2.0}
  - {duration: 2.0, turn_rate: 1.2}
  - {duration: 1.0, accel: -2.0}
```

`loop: true` 的语义是 **每圈结束把物理状态重置回初值再重放**（仿真时钟继续走），
所以波形严格重复；接缝处会真实发生速度/位置跳变。

装载时会先把整圈跑一遍，和初值比较：

| 指标 | 阈值 | 处理 |
|---|---|---|
| `|Δv|` 速度差 | > 0.05 m/s | **警告**（重放瞬间速度跳变） |
| `Δθ` 姿态差 | > 0.05 rad | **警告** |
| `|Δp|` 位置差 | —— | 仅提示 |

也可以直接调 `sim::checkPlanSeam(geom, initial, plan, min_dt)` 自己检查。

示例：[configs/sim/rm_loop.yaml](configs/sim/rm_loop.yaml)（闭合，`|Δv| ≈ 1e-12`）。

## 观测时刻表（非均匀）

```yaml
sensor:
  schedule:
    t0: 0.0
    interval: 0.04
    jitter: 0.015
    # intervals: [0.05, 0.02, 0.08]
```

噪声/丢帧的钥匙是**帧号**（不是时间轴），所以帧间隔怎么抖都不影响可复现性。

## 代码结构

```text
include/sim/
├── core/                数据契约与小工具（不依赖其他层）
│   ├── types.hpp          Control / RobotState / RobotTruth / Observation
│   ├── angle.hpp          角度归一化
│   ├── random.hpp         无状态随机数
│   └── paths.hpp          相对路径 → 工程根（CLion / 命令行一致）
├── motion/              运动与推进（只依赖 core）
│   ├── integrator.hpp     四元数姿态 + Simpson 位置积分
│   ├── controller.hpp     IController / PlanController / RandomController
│   └── preset.hpp         预设 → 控制序列
├── robot/               车体几何（只依赖 core）
│   └── robot_model.hpp    车体系装甲环 → 世界系位姿（纯函数）
├── sensing/             观测（只依赖 core）
│   └── sensor.hpp         观测时刻表 + 位置/法向噪声
├── simulator.hpp        状态机：step / advanceTo / truth
└── config.hpp           YAML 装配（最外层）

src/
├── config.cpp
├── simulator.cpp
├── motion/preset.cpp
└── sensing/sensor.cpp
```

依赖方向是单向的：`core` ← `motion/robot/sensing` ← `simulator` ← `config/apps`。
卡尔曼部分在 `include/kf/`：`filter_interface.hpp`（接口）/ `kalman_filter.hpp`（通用线性核心）/
`models/cv.hpp`（CV）/ `models/cv9.hpp`（CV+虚拟 a）/ `models/ca.hpp`（CA）/ `imm.hpp`（多模型）。

## 键盘控制（预留，未实现）

```cpp
class KeyboardController : public sim::IController {
public:
    sim::Control next(const sim::RobotState&) override {
        sim::Control c;
        c.accel = key_w ? 2.0 : key_s ? -2.0 : 0.0;
        c.angular_rate = sim::Vec3(key_q ? 1.0 : 0.0,      // roll
                                   key_r ? 1.0 : 0.0,      // pitch
                                   key_a ? 1.0 : key_d ? -1.0 : 0.0);
        c.duration = 0.05;
        return c;
    }
};
```

## 观测

`sensor.observeAt(truth, t, frame_index)`：

1. 丢帧：`hash(seed, frame_index)`；
2. 选板：量程 + FOV + 法向朝向，取 `facing_cos` 最大的一块；
3. 噪声：位置三维高斯；法向用**小角度旋转矢量**扰动（保证单位向量）。

`Observation` 里是 `p + n`（没有装甲板编号）；编号只在旁注 `ObservationTruth` 里。

## 契约测试（`tests/sim_contract_test.cpp`）

1. 匀加速 / jerk 的解析式；
2. 刹停后不再移动；
3. 圆周位置闭式解、`|a| = s·ω`；
4. 俯仰 90°：速度转到 -z，位移 `2/π`；
5. 四元数长时间积分仍是单位长度；
6. 一大步 = 50 小步（位置/速度/姿态）；
7. `min_dt` 越小结果越收敛；
8. 跨控制段的一步 = 分段调用；
9. 显式控制不消耗计划队列；
10. 同 seed 随机运动完全一致（含姿态）；
11. 装甲板几何：距离固定、法向/up 正交、姿态传导；
12. 观测时刻表：抖动范围、非均匀、显式间隔、可复现；
13. 位置/法向噪声统计与配置一致；
14. 噪声只由帧号决定；
15. 观测与求值顺序无关；
16. 相机系输出（位置和法向都旋转）；
17. 预设 circle：速度、向心加速度、三点拟合半径；
18. 四个示例 YAML 能装配并跑出有效观测。

## 目前故意没做的事

- **键盘输入**：接口留好。
- **PnP 误差模型**：现在是对称高斯 + 法向角噪声。
- **多板关联 / 卡尔曼层**：下一步。
