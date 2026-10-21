#pragma once

#include <string>
#include <vector>

#include "sim/types.hpp"

namespace sim {

/// 预设运动的可调参数。
struct PresetParams {
    double speed = 2.0;     ///< 目标线速度 [m/s]
    double radius = 1.0;    ///< 圆周半径 [m]（负值 = 顺时针）
    double accel = 2.0;     ///< 起步加速度 [m/s²]
    double duration = 5.0;  ///< 总时长 [s]
};

/// 预设运动 → 控制序列。
///
/// 加新预设只需要：写一个生成函数，然后在 makePresetControls 的分支和 presetNames() 里各加一行。
std::vector<Control> makePresetControls(const std::string& name, const PresetParams& p);

/// 当前支持的预设名（给配置报错和帮助信息用）。
std::vector<std::string> presetNames();

}  // namespace sim
