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

    /// 取完 next() 后，若控制器要求把物理状态重置回初值（plan 循环），返回 true。
    virtual bool consumeReset() { return false; }

    /// 仿真器 reset() 时会调用。
    virtual void reset() {}
};

/// 预排计划：按顺序发放控制段，用完之后回到「零控制」（滑行 + 停止自转）。
class PlanController final : public IController {
public:
    explicit PlanController(std::vector<Control> plan, bool loop = false)
        : plan_(std::move(plan)), loop_(loop) {}

    [[nodiscard]] Control next(const RobotState&) override {
        if (plan_.empty()) return Control{};
        if (i_ >= plan_.size()) {
            if (!loop_) return Control{};
            i_ = 0;
            reset_pending_ = true;  // 循环：先把状态重置回初值，再放第一段
        }
        return plan_[i_++];
    }

    [[nodiscard]] bool consumeReset() override {
        const bool r = reset_pending_;
        reset_pending_ = false;
        return r;
    }

    void reset() override {
        i_ = 0;
        reset_pending_ = false;
    }

    [[nodiscard]] bool loops() const { return loop_; }

    [[nodiscard]] std::size_t remaining() const { return plan_.size() - i_; }
    [[nodiscard]] bool finished() const { return i_ >= plan_.size(); }

private:
    std::vector<Control> plan_;
    std::size_t i_ = 0;
    bool loop_ = false;
    bool reset_pending_ = false;
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

    void reset() override { rng_ = Rng(cfg_.seed); }

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
