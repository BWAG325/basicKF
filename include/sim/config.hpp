#pragma once

#include <string>

#include "sim/scene.hpp"
#include "sim/sensor.hpp"

namespace sim {

struct SimSetup {
    Scene scene;
    SensorConfig sensor;
    std::string source_path;
};

/// 从 YAML 装配「场景 + 传感器」。失败抛 std::runtime_error（带位置信息）。
SimSetup loadSimSetup(const std::string& yaml_path);

}  // namespace sim
