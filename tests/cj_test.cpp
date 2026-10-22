// CJ（匀加加速度）模型检查：F/Q 正确时能从位置观测恢复 jerk。
#include <cmath>
#include <cstdio>

#include "kf/models/cj.hpp"

namespace {
int failures = 0;
void check(bool ok, const char* what) {
    if (!ok) { std::printf("FAIL: %s\n", what); ++failures; }
}
}  // namespace

int main() {
    const double dt = 0.04;
    const double jerk = 2.0;

    kf::CjKalman cj;
    cj.init(kf::Vec3::Zero(), kf::Vec3::Zero(), kf::Vec3::Zero(), kf::Vec3::Zero(),
            0.05, 1.5, 2.0, 5.0);
    cj.setSnapNoise(0.1);
    cj.setMeasurementNoise(1e-4);

    check(cj.stateDim() == 12, "CJ 状态维度应为 12");

    double t = 0.0;
    for (int i = 1; i <= 150; ++i) {
        t = i * dt;
        const kf::Vec3 z(jerk * t * t * t / 6.0, 0.0, 0.0);  // p = j t³/6
        cj.predict(dt);
        cj.update(z);
    }

    const auto est = cj.estimate();
    check(std::abs(est.jerk.x() - jerk) < 0.05, "CJ 应恢复 jerk ≈ 2");
    check(std::abs(est.acceleration.x() - jerk * t) < 0.05, "CJ 加速度应 ≈ jerk·t");
    check(std::abs(est.velocity.x() - 0.5 * jerk * t * t) < 0.05, "CJ 速度应 ≈ ½jt²");
    check(std::abs(est.position.x() - jerk * t * t * t / 6.0) < 0.01, "CJ 位置应 ≈ jt³/6");

    if (failures == 0) std::printf("all CJ tests passed\n");
    return failures == 0 ? 0 : 1;
}
