// EKF 模型检查：CTRV 追圆弧、SPIN 追“底盘平移 + 装甲环自转”。
#include <cmath>
#include <cstdio>

#include "kf/models/ct.hpp"
#include "kf/models/spin.hpp"
#include "kf/models/spin_gen.hpp"

namespace {
int failures = 0;
void check(bool ok, const char* what) {
    if (!ok) { std::printf("FAIL: %s\n", what); ++failures; }
}
}  // namespace

static void testCtrv() {
    const double dt = 0.04;
    kf::CtParams p;  // has_a/has_alpha = false
    p.sigma_v = 1.0;
    p.sigma_w = 1.0;
    p.R = kf::Mat3::Identity() * 1e-6;

    kf::CtKalman f(p);
    f.init(kf::Vec3(1, 0, 0.18), kf::Vec3(0, 1, 0));

    double t = 0.0;
    for (int i = 0; i < 200; ++i) {
        t += dt;
        const kf::Vec3 z(std::cos(t), std::sin(t), 0.18);
        f.predict(dt);
        f.update(z);
    }
    const auto e = f.estimate();
    check((e.position - kf::Vec3(std::cos(t), std::sin(t), 0.18)).norm() < 0.02,
          "CTRV 应跟上单位圆");
    check(std::abs(e.turn_rate - 1.0) < 0.05, "CTRV 应估计出 ω≈1");
}

static void testSpin() {
    const double dt = 0.04;
    const double r = 0.22, w = 6.0, vx = 2.0;

    kf::SpinParams p;
    p.armor_radius = r;
    p.sigma_v = 2.0;
    p.sigma_w = 2.0;
    p.R = kf::Mat3::Identity() * 1e-6;

    kf::SpinKalman f(p);
    f.init(kf::Vec3(r, 0, 0.18), kf::Vec3(vx, 0, 0));

    double t = 0.0;
    for (int i = 0; i < 100; ++i) {
        t += dt;
        const kf::Vec3 z(vx * t + r * std::cos(w * t), r * std::sin(w * t), 0.18);
        f.predict(dt);
        f.update(z);
    }
    const auto e = f.estimate();
    const kf::Vec3 truth(vx * t + r * std::cos(w * t), r * std::sin(w * t), 0.18);
    check((e.plate - truth).norm() < 0.01, "SPIN 应跟上自转的装甲板");
    check(std::abs(e.omega - w) < 0.05, "SPIN 应估计出 ω≈6");
    check(std::abs(e.chassis.x() - vx * t) < 0.02, "SPIN 底盘位置应≈匀速");
}

static void testSpinGen() {
    const double dt = 0.04;
    const double r_a = 0.30, r_b = 0.15, ox = 0.10, oy = 0.05, w = 6.0, vx = 2.0;

    kf::SpinGenParams p;
    p.init_radius = 0.22;
    p.sigma_v = 2.0;
    p.sigma_w = 2.0;
    p.R = kf::Mat3::Identity() * 1e-6;

    kf::SpinGenKalman f(p);
    f.init(kf::Vec3(ox + r_a, oy, 0.18), kf::Vec3(vx, 0, 0));

    double t = 0.0;
    for (int i = 0; i < 150; ++i) {
        t += dt;
        const kf::Vec3 z(vx * t + ox + r_a * std::cos(w * t), oy + r_b * std::sin(w * t), 0.18);
        f.predict(dt);
        f.update(z);
    }
    const auto e = f.estimate();
    check((e.plate - kf::Vec3(vx * t + ox + r_a * std::cos(w * t),
                              oy + r_b * std::sin(w * t), 0.18)).norm() < 0.02,
          "SPIN-G 应跟上偏心/椭圆装甲板");
    check(std::abs(e.a - r_a) < 0.05 && std::abs(e.b - r_b) < 0.05,
          "SPIN-G 应估计出半轴 a/b");
}

int main() {
    testCtrv();
    testSpin();
    testSpinGen();
    if (failures == 0) std::printf("all CT/SPIN tests passed\n");
    return failures == 0 ? 0 : 1;
}
