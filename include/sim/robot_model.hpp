#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include "sim/types.hpp"

namespace sim {

/// 由几何 + 车体位置 + 装甲环总转角，派生每一块装甲板的真值位姿。
/// 纯函数：不缓存、不依赖调用顺序。
///
/// 约定（世界系，右手，z 向上）：
///   armor[i] 中心 = center + R_z(ring + i·2π/n) · (r, 0, dz)
///   armor[i] 外法向 = R_z(ring + i·2π/n) · (1, 0, 0)
/// 因此 yaw(法向) == ring + i·2π/n（折到 (-π, π]）。
inline std::array<ArmorTruth, kMaxArmors> makeArmors(const RobotGeometry& g,
                                                     const Vec3& center,
                                                     double ring_yaw) {
    std::array<ArmorTruth, kMaxArmors> out{};
    const int n = std::clamp(g.armor_count, 1, kMaxArmors);
    const double step = kTwoPi / static_cast<double>(n);

    for (int i = 0; i < n; ++i) {
        const double ang = ring_yaw + static_cast<double>(i) * step;
        ArmorTruth a;
        a.index = i;
        a.p = center + Vec3(g.armor_radius * std::cos(ang),
                            g.armor_radius * std::sin(ang),
                            g.armor_dz);
        a.n = Vec3(std::cos(ang), std::sin(ang), 0.0);
        a.yaw = wrapPi(ang);
        a.width = g.armor_width;
        a.height = g.armor_height;
        out[static_cast<std::size_t>(i)] = a;
    }
    return out;
}

}  // namespace sim
