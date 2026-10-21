#include "sim/preset.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace sim {

namespace {

/// 从静止加速到 speed，然后匀速跑到 duration。
std::vector<Control> straightPlan(const PresetParams& p) {
    std::vector<Control> out;
    if (!(p.duration > 0.0)) return out;

    const double v = std::max(0.0, p.speed);
    const double a = std::max(1e-6, std::abs(p.accel));
    const double t_acc = (v > 0.0) ? std::min(p.duration, v / a) : 0.0;

    if (t_acc > 0.0) {
        Control c;
        c.accel = a;
        c.duration = t_acc;
        out.push_back(c);
    }
    if (p.duration - t_acc > 0.0) {
        Control c;
        c.duration = p.duration - t_acc;
        out.push_back(c);
    }
    return out;
}

}  // namespace

std::vector<Control> makePresetControls(const std::string& name, const PresetParams& p) {
    if (name == "idle") {
        Control c;
        c.duration = p.duration;
        return {c};
    }

    if (name == "straight") return straightPlan(p);

    if (name == "circle") {
        auto out = straightPlan(p);
        if (out.empty()) return out;

        const double v = std::max(0.0, p.speed);
        const double turn = (std::abs(p.radius) > 1e-9) ? v / p.radius : 0.0;
        out.back().angular_rate.z() = turn;  // 匀速段改成绕圈；radius 为负则顺时针
        return out;
    }

    return {};
}

std::vector<std::string> presetNames() { return {"idle", "straight", "circle"}; }

}  // namespace sim
