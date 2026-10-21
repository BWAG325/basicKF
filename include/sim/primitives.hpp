#pragma once

#include <cmath>
#include <limits>
#include <memory>
#include <string_view>

#include "sim/primitive.hpp"
#include "sim/types.hpp"

namespace sim {

inline constexpr double kHeadingInherit = std::numeric_limits<double>::quiet_NaN();
inline constexpr double kSpeedInherit = std::numeric_limits<double>::quiet_NaN();
inline constexpr double kRateInherit = std::numeric_limits<double>::quiet_NaN();

namespace detail {
inline constexpr double kEps = 1e-9;

inline double headingOf(const TranslationState& s) {
    const double n = std::hypot(s.v.x(), s.v.y());
    return n > kEps ? std::atan2(s.v.y(), s.v.x()) : 0.0;
}
}  // namespace detail

// ---------------------------------------------------------------------------
// 接口
// ---------------------------------------------------------------------------

class ITranslationPrimitive {
public:
    virtual ~ITranslationPrimitive() = default;
    [[nodiscard]] virtual double duration() const = 0;
    [[nodiscard]] virtual TranslationState eval(const TranslationState& start, double tau) const = 0;
    [[nodiscard]] virtual std::string_view name() const = 0;
};

class IYawPrimitive {
public:
    virtual ~IYawPrimitive() = default;
    [[nodiscard]] virtual double duration() const = 0;
    [[nodiscard]] virtual YawState eval(const YawState& start, double tau) const = 0;
    [[nodiscard]] virtual std::string_view name() const = 0;
};

class ISpinPrimitive {
public:
    virtual ~ISpinPrimitive() = default;
    [[nodiscard]] virtual double duration() const = 0;
    [[nodiscard]] virtual PhaseState eval(const PhaseState& start, double tau) const = 0;
    [[nodiscard]] virtual std::string_view name() const = 0;
};

// ---------------------------------------------------------------------------
// 平移
// ---------------------------------------------------------------------------

/// 原地保持：p 不变、速度归零。跟在运动段后面就是一个「瞬时刹车」，
/// 会在 validate() 里被标记为未声明的速度跳变（如果前面有速度）。
class Hold final : public ITranslationPrimitive {
public:
    explicit Hold(double duration) : dur_(duration) {}

    [[nodiscard]] double duration() const override { return dur_; }
    [[nodiscard]] std::string_view name() const override { return "hold"; }

    [[nodiscard]] TranslationState eval(const TranslationState& s, double) const override {
        return TranslationState{s.p, Vec3::Zero(), Vec3::Zero()};
    }

private:
    double dur_;
};

/// 匀速直线（可带恒定竖直速度，常用于模拟跳台阶/下落）。
/// heading = kHeadingInherit 时继承段首速度方向，保证 C1 连续。
class Line final : public ITranslationPrimitive {
public:
    Line(double duration, double speed = kSpeedInherit, double heading = kHeadingInherit, double vz = 0.0)
        : dur_(duration), speed_(speed), heading_(heading), vz_(vz) {}

    [[nodiscard]] double duration() const override { return dur_; }
    [[nodiscard]] std::string_view name() const override { return "line"; }

    [[nodiscard]] TranslationState eval(const TranslationState& s, double tau) const override {
        const double hdg = std::isnan(heading_) ? detail::headingOf(s) : heading_;
        const double sp = std::isnan(speed_) ? s.v.head<2>().norm() : speed_;
        const Vec3 v(sp * std::cos(hdg), sp * std::sin(hdg), vz_);
        TranslationState o;
        o.p = s.p + v * tau;
        o.v = v;
        o.a.setZero();
        return o;
    }

private:
    double dur_;
    double speed_;
    double heading_;
    double vz_;
};

/// 恒定速度 + 恒定航向角速度 ⇒ 圆弧。
/// 入口速度方向继承段首速度；omega > 0 为逆时针（左转）。
class Arc final : public ITranslationPrimitive {
public:
    Arc(double duration, double speed = kSpeedInherit, double omega = 0.0,
        double heading = kHeadingInherit, double vz = 0.0)
        : dur_(duration), speed_(speed), omega_(omega), heading_(heading), vz_(vz) {}

    [[nodiscard]] double duration() const override { return dur_; }
    [[nodiscard]] std::string_view name() const override { return "arc"; }

    [[nodiscard]] TranslationState eval(const TranslationState& s, double tau) const override {
        const double hdg0 = std::isnan(heading_) ? detail::headingOf(s) : heading_;
        const double sp = std::isnan(speed_) ? s.v.head<2>().norm() : speed_;

        if (std::abs(omega_) < 1e-12) {  // 退化成正弦/直线
            const Vec3 v(sp * std::cos(hdg0), sp * std::sin(hdg0), vz_);
            return TranslationState{s.p + v * tau, v, Vec3::Zero()};
        }

        const double th = hdg0 + omega_ * tau;
        const double R = sp / omega_;

        TranslationState o;
        o.p = s.p + Vec3(R * (std::sin(th) - std::sin(hdg0)),
                         -R * (std::cos(th) - std::cos(hdg0)),
                         vz_ * tau);
        o.v = Vec3(sp * std::cos(th), sp * std::sin(th), vz_);
        o.a = Vec3(-sp * omega_ * std::sin(th), sp * omega_ * std::cos(th), 0.0);
        return o;
    }

private:
    double dur_;
    double speed_;
    double omega_;
    double heading_;
    double vz_;
};

/// 恒定切向加速度（accel < 0 即减速，速度到 0 后保持静止）。
/// heading = kHeadingInherit 时沿段首速度方向，保证 C1 连续。
class Accel final : public ITranslationPrimitive {
public:
    Accel(double duration, double accel, double heading = kHeadingInherit, double vz = 0.0)
        : dur_(duration), accel_(accel), heading_(heading), vz_(vz) {}

    [[nodiscard]] double duration() const override { return dur_; }
    [[nodiscard]] std::string_view name() const override { return "accel"; }

    [[nodiscard]] TranslationState eval(const TranslationState& s, double tau) const override {
        const double hdg = std::isnan(heading_) ? detail::headingOf(s) : heading_;
        const Vec3 u(std::cos(hdg), std::sin(hdg), 0.0);
        const double s0 = s.v.head<2>().dot(u.head<2>());  // 沿航向的分量

        double t_zero = std::numeric_limits<double>::infinity();
        if (accel_ != 0.0 && s0 * accel_ < 0.0) t_zero = -s0 / accel_;

        const double te = std::min(tau, t_zero);
        const double s_t = s0 + accel_ * te;
        const double dist = s0 * te + 0.5 * accel_ * te * te;

        TranslationState o;
        o.p = s.p + u * dist;
        o.v = (std::abs(s_t) < 1e-12) ? Vec3::Zero() : Vec3(u * s_t);
        o.a = (tau < t_zero) ? Vec3(u * accel_) : Vec3::Zero();
        return o;
    }

private:
    double dur_;
    double accel_;
    double heading_;
    double vz_;
};

/// 恒定减速度刹停（沿当前速度方向）。
class Brake final : public ITranslationPrimitive {
public:
    Brake(double duration, double decel) : dur_(duration), decel_(decel) {}

    [[nodiscard]] double duration() const override { return dur_; }
    [[nodiscard]] std::string_view name() const override { return "brake"; }

    [[nodiscard]] TranslationState eval(const TranslationState& s, double tau) const override {
        const double s0 = s.v.head<2>().norm();
        if (s0 < detail::kEps || decel_ <= 0.0) return TranslationState{s.p, Vec3::Zero(), Vec3::Zero()};

        const Vec3 u(s.v.x() / s0, s.v.y() / s0, 0.0);  // 水平单位方向
        const double t_stop = s0 / decel_;

        TranslationState o;
        if (tau >= t_stop) {
            o.p = s.p + u * (s0 * t_stop * 0.5);
            o.v.setZero();
            o.a.setZero();
        } else {
            const double d = s0 * tau - 0.5 * decel_ * tau * tau;
            o.p = s.p + u * d;
            o.v = u * (s0 - decel_ * tau);
            o.a = -u * decel_;
        }
        return o;
    }

private:
    double dur_;
    double decel_;
};

// ---------------------------------------------------------------------------
// 车体朝向
// ---------------------------------------------------------------------------

class YawHold final : public IYawPrimitive {
public:
    explicit YawHold(double duration) : dur_(duration) {}
    [[nodiscard]] double duration() const override { return dur_; }
    [[nodiscard]] std::string_view name() const override { return "yaw_hold"; }
    [[nodiscard]] YawState eval(const YawState& s, double) const override {
        return YawState{s.yaw, 0.0, 0.0};
    }

private:
    double dur_;
};

class YawRate final : public IYawPrimitive {
public:
    explicit YawRate(double duration, double rate = kRateInherit) : dur_(duration), rate_(rate) {}
    [[nodiscard]] double duration() const override { return dur_; }
    [[nodiscard]] std::string_view name() const override { return "yaw_rate"; }
    [[nodiscard]] YawState eval(const YawState& s, double tau) const override {
        const double r = std::isnan(rate_) ? s.rate : rate_;
        return YawState{s.yaw + r * tau, r, 0.0};
    }

private:
    double dur_;
    double rate_;
};

class YawRamp final : public IYawPrimitive {
public:
    YawRamp(double duration, double rate0, double rate1)
        : dur_(duration), r0_(rate0), r1_(rate1) {}
    [[nodiscard]] double duration() const override { return dur_; }
    [[nodiscard]] std::string_view name() const override { return "yaw_ramp"; }
    [[nodiscard]] YawState eval(const YawState& s, double tau) const override {
        if (dur_ <= 0.0) return YawState{s.yaw, s.rate, 0.0};
        const double w0 = std::isnan(r0_) ? s.rate : r0_;  // 未给起点角速度则继承
        const double acc = (r1_ - w0) / dur_;
        return YawState{s.yaw + w0 * tau + 0.5 * acc * tau * tau, w0 + acc * tau, acc};
    }

private:
    double dur_;
    double r0_;
    double r1_;
};

// ---------------------------------------------------------------------------
// 装甲板自转
// ---------------------------------------------------------------------------

class SpinOff final : public ISpinPrimitive {
public:
    explicit SpinOff(double duration) : dur_(duration) {}
    [[nodiscard]] double duration() const override { return dur_; }
    [[nodiscard]] std::string_view name() const override { return "spin_off"; }
    [[nodiscard]] PhaseState eval(const PhaseState& s, double) const override {
        return PhaseState{s.phase, 0.0, 0.0};
    }

private:
    double dur_;
};

class SpinRate final : public ISpinPrimitive {
public:
    explicit SpinRate(double duration, double rate = kRateInherit) : dur_(duration), rate_(rate) {}
    [[nodiscard]] double duration() const override { return dur_; }
    [[nodiscard]] std::string_view name() const override { return "spin_rate"; }
    [[nodiscard]] PhaseState eval(const PhaseState& s, double tau) const override {
        const double w = std::isnan(rate_) ? s.omega : rate_;
        return PhaseState{s.phase + w * tau, w, 0.0};
    }

private:
    double dur_;
    double rate_;
};

/// 线性变化的角速度，相位解析积分（保证任意 t 可直接求值）。
class SpinRamp final : public ISpinPrimitive {
public:
    SpinRamp(double duration, double rate0, double rate1)
        : dur_(duration), w0_(rate0), w1_(rate1) {}
    [[nodiscard]] double duration() const override { return dur_; }
    [[nodiscard]] std::string_view name() const override { return "spin_ramp"; }
    [[nodiscard]] PhaseState eval(const PhaseState& s, double tau) const override {
        if (dur_ <= 0.0) return PhaseState{s.phase, s.omega, 0.0};
        const double w0 = std::isnan(w0_) ? s.omega : w0_;  // 未给起点角速度则继承
        const double alpha = (w1_ - w0) / dur_;
        return PhaseState{s.phase + w0 * tau + 0.5 * alpha * tau * tau, w0 + alpha * tau, alpha};
    }

private:
    double dur_;
    double w0_;
    double w1_;
};

}  // namespace sim
