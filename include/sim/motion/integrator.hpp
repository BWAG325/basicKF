#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include "sim/core/types.hpp"

namespace sim {

namespace detail {

/// s(τ) = s0 + a0·τ + ½j·τ² 第一次撞到 0 的时刻；不会撞到就返回 inf。
inline double firstStopTime(double s0, double a0, double j) {
    constexpr double kEps = 1e-12;

    if (std::abs(j) < kEps) {
        return (a0 < 0.0) ? std::max(0.0, -s0 / a0) : std::numeric_limits<double>::infinity();
    }

    const double disc = a0 * a0 - 2.0 * j * s0;
    if (disc < 0.0) return std::numeric_limits<double>::infinity();

    const double sq = std::sqrt(disc);
    double best = std::numeric_limits<double>::infinity();
    for (const double cand : {(-a0 + sq) / j, (-a0 - sq) / j}) {
        if (cand > kEps && cand < best) best = cand;
    }
    return best;
}

/// body 系恒角速度 ω 下的精确姿态更新：q ← q ⊗ exp(ω·dt)。
inline Quat rotateBody(const Quat& q, const Vec3& w, double dt) {
    const double angle = w.norm() * dt;
    if (angle < 1e-15) return q;
    return (q * Quat(Eigen::AngleAxisd(angle, w.normalized()))).normalized();
}

}  // namespace detail

/// 用一段恒定控制量把状态推进 h 秒。
///
/// 姿态：q(t+τ) = q(t) ⊗ exp(ω·τ)          （body 系恒角速度，精确解）
/// 速率：s(τ)   = s0 + accel·τ + ½·jerk·τ² （撞到 0 停住，不倒车）
/// 位置：p += ∫ s(τ)·(q(τ)·x̂) dτ          （Simpson 三点积分）
///
/// 位置积分精度由调用方的步长决定 —— `Simulator` 会按 `min_dt` 切成小步。
inline void integrate(RobotState& s, const Control& c, double h) {
    if (!(h > 0.0)) return;

    const double s0 = s.speed;
    const double a0 = c.accel;
    const double j = c.jerk;
    const Vec3 w = c.angular_rate;

    // ---- 本步里能移动多久 ----
    double t_move = h;
    if (s0 > 1e-12) {
        t_move = std::min(h, detail::firstStopTime(s0, a0, j));
    } else if (!(a0 > 0.0) && !(a0 == 0.0 && j > 0.0)) {
        t_move = 0.0;
    }

    // ---- 姿态（精确） ----
    const Quat q1 = detail::rotateBody(s.q, w, h);

    // ---- 位置（Simpson 三点） ----
    const auto speed_at = [&](double tau) {
        return std::max(0.0, s0 + a0 * tau + 0.5 * j * tau * tau);
    };
    const auto dir_at = [&](double tau) {
        return detail::rotateBody(s.q, w, tau) * Vec3(1.0, 0.0, 0.0);
    };

    if (t_move > 0.0) {
        const double half = 0.5 * t_move;
        const Vec3 u0 = dir_at(0.0);
        const Vec3 um = dir_at(half);
        const Vec3 u1 = dir_at(t_move);
        s.p += (t_move / 6.0) *
               (speed_at(0.0) * u0 + 4.0 * speed_at(half) * um + speed_at(t_move) * u1);
    }

    // ---- 速率 / 速度 / 加速度 ----
    s.speed = (t_move < h) ? 0.0 : speed_at(h);
    s.q = q1;
    s.omega = w;

    const Vec3 u_body(1.0, 0.0, 0.0);
    const Vec3 u_world = s.q * u_body;
    s.v = s.speed * u_world;

    if (s.speed > 1e-12) {
        s.accel = a0 + j * t_move;
        // 车体上的点：a = ṡ·u + s·(ω_world × u)
        s.a = s.accel * u_world + s.speed * (s.q * (w.cross(u_body)));
    } else {
        s.accel = 0.0;
        s.a.setZero();
    }

    // ---- 装甲环相对车体的自转 ----
    s.spin_phase += c.spin_rate * h;
    s.spin_rate = c.spin_rate;

    s.t += h;
}

}  // namespace sim
