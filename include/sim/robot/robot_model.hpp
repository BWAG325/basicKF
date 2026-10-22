#pragma once

#include <algorithm>
#include <array>
#include <cmath>

#include "sim/core/types.hpp"

namespace sim {

/// 由几何 + 车体位姿，派生每一块装甲板的真值位姿（纯函数）。
///
/// 装甲板定义在**车体系**里：绕车体 z 轴排成一圈，环角 = ring + i·2π/n，
/// 再整体被车体姿态 `body` 旋转到世界系：
///
///   p  = center + body · (r·cos, r·sin, armor_dz)
///   q  = body ⊗ R_z(ring + i·2π/n)
///   n  = q · x̂      （板外法向）
///   up = q · ẑ      （板「向上」轴）
inline std::array<ArmorTruth, kMaxArmors> makeArmors(const RobotGeometry& g,
                                                     const Vec3& center,
                                                     const Quat& body,
                                                     double ring) {
    std::array<ArmorTruth, kMaxArmors> out{};
    const int n = std::clamp(g.armor_count, 1, kMaxArmors);
    const double step = kTwoPi / static_cast<double>(n);

    for (int i = 0; i < n; ++i) {
        const double ang = ring + static_cast<double>(i) * step;
        const Quat qa = (body * Quat(Eigen::AngleAxisd(ang, Vec3::UnitZ()))).normalized();

        ArmorTruth a;
        a.index = i;
        a.p = center + body * Vec3(g.armor_radius * std::cos(ang),
                                   g.armor_radius * std::sin(ang),
                                   g.armor_dz);
        a.q = qa;
        a.n = qa * Vec3(1.0, 0.0, 0.0);
        a.up = qa * Vec3(0.0, 0.0, 1.0);
        a.yaw = std::atan2(a.n.y(), a.n.x());
        a.width = g.armor_width;
        a.height = g.armor_height;
        out[static_cast<std::size_t>(i)] = a;
    }
    return out;
}

}  // namespace sim
