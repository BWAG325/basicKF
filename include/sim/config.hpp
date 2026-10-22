#pragma once

#include <memory>
#include <string>
#include <vector>

#include "sim/motion/controller.hpp"
#include "sim/sensing/sensor.hpp"
#include "sim/simulator.hpp"

namespace sim {

struct SimSetup {
    RobotGeometry geometry;
    RobotState initial;
    std::unique_ptr<IController> controller;  ///< preset / plan / random，可为空（零控制）
    SensorConfig sensor;
    double min_dt = 0.001;  ///< 积分步长上限
};

/// 从 YAML 装配「几何 + 初值 + 控制来源 + 传感器」。失败抛 std::runtime_error。
SimSetup loadSimSetup(const std::string& yaml_path);

/// 循环计划在接缝处的跳变：把计划跑一整圈，和初值比较。
/// `loop: true` 时每圈结束会把物理状态重置回初值，所以这里的差值
/// 就是重放瞬间真实发生的速度/位置跳变。
struct PlanSeam {
    double dv = 0.0;      ///< |v_end - v_init| [m/s]
    double dp = 0.0;      ///< |p_end - p_init| [m]
    double dtheta = 0.0;  ///< 姿态差 [rad]
    bool warn = false;    ///< dv 或 dtheta 超阈值
};

PlanSeam checkPlanSeam(const RobotGeometry& geom, const RobotState& initial,
                       const std::vector<Control>& plan, double min_dt);

}  // namespace sim
