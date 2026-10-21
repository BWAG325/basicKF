#pragma once

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "sim/random.hpp"
#include "sim/types.hpp"

namespace sim {

// ---------------------------------------------------------------------------
// 控制接口
// ---------------------------------------------------------------------------

/// 控制量来源。仿真器每一步向它要一段控制（带 duration）。
/// 以后接键盘就在这一层加实现：读按键 → 返回 Control，仿真器不用改。
class IController {
public:
    virtual ~IController() = default;
    [[nodiscard]] virtual Control next(const RobotState& state) = 0;
};

/// 预排计划：按顺序发放控制段，用完之后回到「零控制」（滑行 + 停止自转）。
class PlanController final : public IController {
public:
    explicit PlanController(std::vector<Control> plan) : plan_(std::move(plan)) {}

    [[nodiscard]] Control next(const RobotState&) override {
        if (i_ < plan_.size()) return plan_[i_++];
        return Control{};
    }

    [[nodiscard]] std::size_t remaining() const { return plan_.size() - i_; }
    [[nodiscard]] bool finished() const { return i_ >= plan_.size(); }

private:
    std::vector<Control> plan_;
    std::size_t i_ = 0;
};

/// 随机运动：每段控制随机抽参数（含 duration），用同一个 seed 可复现。
class RandomController final : public IController {
public:
    struct Config {
        std::uint64_t seed = 1;
        double min_duration = 0.4;
        double max_duration = 1.5;
        double min_accel = -2.0;
        double max_accel = 2.0;
        double min_turn_rate = -1.0;
        double max_turn_rate = 1.0;
        double min_pitch_rate = 0.0;
        double max_pitch_rate = 0.0;
        double min_roll_rate = 0.0;
        double max_roll_rate = 0.0;
        double min_spin_rate = -10.0;
        double max_spin_rate = 10.0;
    };

    RandomController() : RandomController(Config{}) {}
    explicit RandomController(Config cfg) : cfg_(cfg), rng_(cfg.seed) {}

    [[nodiscard]] Control next(const RobotState&) override {
        Control c;
        c.duration = uniform(cfg_.min_duration, cfg_.max_duration);
        c.accel = uniform(cfg_.min_accel, cfg_.max_accel);
        c.angular_rate = Vec3(uniform(cfg_.min_roll_rate, cfg_.max_roll_rate),
                              uniform(cfg_.min_pitch_rate, cfg_.max_pitch_rate),
                              uniform(cfg_.min_turn_rate, cfg_.max_turn_rate));
        c.spin_rate = uniform(cfg_.min_spin_rate, cfg_.max_spin_rate);
        return c;
    }

private:
    [[nodiscard]] double uniform(double lo, double hi) { return lo + (hi - lo) * rng_.uniform(); }

    Config cfg_;
    Rng rng_;
};

}  // namespace sim
