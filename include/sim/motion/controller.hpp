#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "sim/core/random.hpp"
#include "sim/core/types.hpp"

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

/// 沿椭圆匀速巡航：x = a·cosθ，y = b·sinθ。
/// 曲率 κ(θ) = ab / (a²sin²θ + b²cos²θ)^{3/2}，取 ω = v·κ，θ 按弧长推进。
/// 初始位姿要放在 (a, 0)、朝向 +y（切线方向）。
class EllipseController final : public IController {
public:
    struct Config {
        double a = 2.0;          ///< 半长轴 [m]
        double b = 1.0;          ///< 半短轴 [m]
        double speed = 1.5;      ///< 基准线速度 [m/s]
        double speed_amp = 0.0;  ///< 速度起伏比例（0.4 = ±40%）
        double speed_waves = 2.0;///< 每圈速度起伏的次数
        double spin_rate = 0.0;  ///< 小陀螺 [rad/s]
        double segment = 0.02;   ///< 每段控制时长 [s]
    };

    explicit EllipseController(Config cfg) : cfg_(cfg) { reset(); }

    void reset() override { theta_ = 0.0; }

    [[nodiscard]] Control next(const RobotState& state) override {
        const double s = std::sin(theta_);
        const double c = std::cos(theta_);
        const double denom = cfg_.a * cfg_.a * s * s + cfg_.b * cfg_.b * c * c;
        const double kappa = cfg_.a * cfg_.b / std::pow(denom, 1.5);

        // 期望速度沿椭圆起伏；实际速度由 accel 跟踪，路径一致性用实际速度算 ω。
        const double v_des = desiredSpeed(theta_);
        const double v_act = std::max(0.05, state.speed);

        Control ctrl;
        ctrl.accel = std::clamp(5.0 * (v_des - v_act), -3.0, 3.0);
        ctrl.angular_rate = Vec3(0.0, 0.0, v_act * kappa);
        ctrl.spin_rate = cfg_.spin_rate;
        ctrl.duration = cfg_.segment;

        const double ds = v_act * cfg_.segment;
        theta_ += ds / std::sqrt(denom);
        if (theta_ >= 2.0 * kPi) theta_ -= 2.0 * kPi;
        return ctrl;
    }

private:
    [[nodiscard]] double desiredSpeed(double theta) const {
        const double f = 1.0 + cfg_.speed_amp * std::sin(cfg_.speed_waves * theta);
        return cfg_.speed * std::max(0.05, f);
    }

    Config cfg_;
    double theta_ = 0.0;
};

}  // namespace sim
