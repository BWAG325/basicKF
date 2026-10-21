#pragma once

#include <memory>
#include <string>

#include "sim/controller.hpp"
#include "sim/sensor.hpp"
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

}  // namespace sim
