// CA（匀加速）模型检查：F/Q 是否正确，能否从位置观测里恢复加速度。
#include <cmath>
#include <cstdio>

#include "kf/models/ca.hpp"
#include "kf/models/cv9.hpp"

namespace {

int failures = 0;

void check(bool ok, const char* what) {
    if (!ok) {
        std::printf("FAIL: %s\n", what);
        ++failures;
    }
}

}  // namespace

int main() {
    const double dt = 0.04;
    const double acc = 2.0;

    kf::CaKalman ca;
    ca.init(kf::Vec3::Zero(), kf::Vec3::Zero(), kf::Vec3::Zero(), 0.05, 1.5, 2.0);
    ca.setJerkNoise(0.1);
    ca.setMeasurementNoise(1e-4);

    check(ca.stateDim() == 9, "CA 状态维度应为 9");
    check(ca.measDim() == 3, "CA 观测维度应为 3");

    double t = 0.0;
    for (int i = 1; i <= 150; ++i) {
        t = i * dt;
        // p = 1/2 a t²，v = a t，a 恒定
        const kf::Vec3 z(0.5 * acc * t * t, 0.0, 0.0);
        ca.predict(dt);
        ca.update(z);
    }

    const auto est = ca.estimate();
    check(std::abs(est.acceleration.x() - acc) < 0.05, "CA 应恢复加速度 ≈ 2 m/s²");
    check(std::abs(est.velocity.x() - acc * t) < 0.05, "CA 速度应 ≈ a·t");
    check(std::abs(est.position.x() - 0.5 * acc * t * t) < 0.01, "CA 位置应 ≈ ½at²");

    // CV9：虚拟加速度必须恒为 0，即使被 setState 写了非零值也要清零。
    kf::Cv9Kalman cv9;
    cv9.init(kf::Vec3::Zero(), kf::Vec3(2, 0, 0), 0.05, 1.5);
    cv9.setAccelNoise(0.5);
    cv9.setMeasurementNoise(0.02);

    Eigen::VectorXd x = cv9.state();
    x.tail<3>() << 1.0, 2.0, 3.0;
    cv9.setState(x, cv9.covariance());
    check(cv9.state().tail<3>().norm() < 1e-12, "CV9 虚拟加速度应被清零");

    for (int i = 1; i <= 30; ++i) {
        const double dt = 0.04;
        cv9.predict(dt);
        cv9.update(kf::Vec3(2.0 * i * dt, 0.0, 0.0));
    }
    check(cv9.estimate().acceleration.norm() < 1e-12, "CV9 运行中加速度应保持 0");
    check(std::abs(cv9.estimate().position.x() - 2.0 * 30 * 0.04) < 0.02, "CV9 位置应跟上匀速目标");

    if (failures == 0) std::printf("all CA tests passed\n");
    return failures == 0 ? 0 : 1;
}
