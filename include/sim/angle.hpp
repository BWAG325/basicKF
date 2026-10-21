#pragma once

#include <cmath>

namespace sim {

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kTwoPi = 2.0 * kPi;

/// 折到 (-π, π]。
inline double wrapPi(double a) noexcept {
    a = std::remainder(a, kTwoPi);
    return (a <= -kPi) ? a + kTwoPi : a;
}

/// a - b 折到 (-π, π]，用于角度差比较（不受 2π 整数倍影响）。
inline double angleDiff(double a, double b) noexcept { return wrapPi(a - b); }

}  // namespace sim
