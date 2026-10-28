#pragma once

#include <Eigen/Core>

#include <cmath>
#include <memory>
#include <utility>

#include "kf/ekf.hpp"
#include "kf/models/cv.hpp"

namespace kf {

/// CA（常加速度）整车模型，状态在 CV 的 11 维后面追加平移加速度：
///   x = [cx, vx, cy, vy, cz, vz, yaw, yaw_rate, r, dr, dh, ax, ay, az]
///   (0..10 与 CvKalman 完全相同，11..13 是加速度)
///
/// - ca_tau <= 0：纯 CA，a 保持常值；
/// - ca_tau > 0：Singer/阻尼加速度，a 按 exp(-dt/tau) 衰减；
///   位置/速度用精确积分，Q 仍沿用白 jerk 近似（设计评审时可改成 Singer Q）。
struct CaParams {
    double q_jerk = 200.0;  ///< jerk PSD，单位 (m/s²)²/s；越大越信机动
    double q_yaw = 75.0;    ///< 与 CV 相同的 yaw 角加速度方差
    double ca_tau = 0.0;    ///< 0=纯 CA；>0 为 Singer 相关时间 [s]
    double armor_radius = 0.22;
    int armor_num = 4;
    int armor_id = 0;
    double p0_accel_var = 25.0;  ///< 初始加速度方差 (5 m/s²)^2
    Mat3 R = Mat3::Identity() * 0.0004;
};

inline Ekf::Model makeCaModel(const CaParams& p) {
    Ekf::Model m;
    m.nx = 14;
    m.nz = 3;

    // 状态布局：pos = 0/2/4, vel = 1/3/5, acc = 11/12/13
    m.f = [tau = p.ca_tau](const Eigen::VectorXd& x, double dt) {
        Eigen::VectorXd y = x;
        const double rho = tau > 0.0 ? std::exp(-dt / tau) : 1.0;
        for (int axis = 0; axis < 3; ++axis) {
            const int ip = 2 * axis;
            const int iv = ip + 1;
            const int ia = 11 + axis;
            const double a = x[ia];
            if (tau > 0.0) {
                const double s = tau * (1.0 - rho);
                y[ip] += x[iv] * dt + a * (tau * dt - tau * s);
                y[iv] += a * s;
                y[ia] = rho * a;
            } else {
                y[ip] += x[iv] * dt + 0.5 * a * dt * dt;
                y[iv] += a * dt;
                // y[ia] = a;
            }
        }
        y[6] = std::atan2(std::sin(x[6] + x[7] * dt), std::cos(x[6] + x[7] * dt));
        return y;
    };

    m.Q = [q_jerk = p.q_jerk, q_yaw = p.q_yaw](double dt) {
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(14, 14);
        const double a = dt * dt * dt * dt * dt / 20.0;
        const double b = dt * dt * dt * dt / 8.0;
        const double c = dt * dt * dt / 6.0;
        const double d = dt * dt * dt / 3.0;
        const double e = dt * dt / 2.0;
        const double f = dt;

        for (int axis = 0; axis < 3; ++axis) {
            const int ip = 2 * axis;
            const int iv = ip + 1;
            const int ia = 11 + axis;
            Q(ip, ip) = a * q_jerk;
            Q(ip, iv) = b * q_jerk;
            Q(ip, ia) = c * q_jerk;
            Q(iv, ip) = b * q_jerk;
            Q(iv, iv) = d * q_jerk;
            Q(iv, ia) = e * q_jerk;
            Q(ia, ip) = c * q_jerk;
            Q(ia, ia) = f * q_jerk;
            Q(ia, iv) = e * q_jerk;
        }

        // yaw 仍是常角速度，过程噪声与 CV 同一套白角加速度离散化。
        // 不能复用上面的 jerk 系数，否则同样的 q_yaw 会让角速度方差小约两个数量级。
        const double ya = dt * dt * dt * dt / 4.0;
        const double yb = dt * dt * dt / 2.0;
        const double yc = dt * dt;
        Q(6, 6) = ya * q_yaw;
        Q(6, 7) = yb * q_yaw;
        Q(7, 6) = yb * q_yaw;
        Q(7, 7) = yc * q_yaw;
        return Q;
    };

    // 观测仍是“某块装甲板的三维位置”，加速度不直接进观测。
    m.h = [p](const Eigen::VectorXd& x) {
        const double ang = x[6] + p.armor_id * 2.0 * 3.14159265358979323846 / p.armor_num;
        const bool lh = (p.armor_num == 4) && (p.armor_id == 1 || p.armor_id == 3);
        const double r = x[8] + (lh ? x[9] : 0.0);
        Eigen::VectorXd h = Eigen::VectorXd::Zero(3);
        h(0) = x[0] - r * std::cos(ang);
        h(1) = x[2] - r * std::sin(ang);
        h(2) = x[4] + (lh ? x[10] : 0.0);
        return h;
    };

    m.Hx = [p](const Eigen::VectorXd& x) {
        const double ang = x[6] + p.armor_id * 2.0 * 3.14159265358979323846 / p.armor_num;
        const bool lh = (p.armor_num == 4) && (p.armor_id == 1 || p.armor_id == 3);
        const double r = x[8] + (lh ? x[9] : 0.0);
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 14);
        H(0, 0) = 1.0;
        H(1, 2) = 1.0;
        H(2, 4) = 1.0;
        H(0, 6) = r * std::sin(ang);
        H(1, 6) = -r * std::cos(ang);
        H(0, 8) = -std::cos(ang);
        H(1, 8) = -std::sin(ang);
        if (lh) {
            H(0, 9) = -std::cos(ang);
            H(1, 9) = -std::sin(ang);
            H(2, 10) = 1.0;
        }
        return H;
    };
    return m;
}

class CaKalman final : public IFilter {
public:
    CaKalman() = default;
    explicit CaKalman(CaParams p) : params_(std::move(p)) {}

    void init(const Vec3& plate, const Vec3& velocity) {
        const double ang = params_.armor_id * 2.0 * 3.14159265358979323846 / params_.armor_num;
        Eigen::VectorXd x0 = Eigen::VectorXd::Zero(14);
        x0[6] = 0.0;
        x0[8] = params_.armor_radius;
        x0[0] = plate.x() + params_.armor_radius * std::cos(ang);
        x0[2] = plate.y() + params_.armor_radius * std::sin(ang);
        x0[4] = plate.z();
        x0[1] = velocity.x();
        x0[3] = velocity.y();
        x0[5] = velocity.z();

        Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(14, 14);
        P0(0, 0) = P0(2, 2) = 0.5 * 0.5;
        P0(4, 4) = 0.05 * 0.05;
        P0(1, 1) = P0(3, 3) = 1.5 * 1.5;
        P0(5, 5) = 0.5 * 0.5;
        P0(6, 6) = 1.5707963267948966 * 1.5707963267948966;
        P0(7, 7) = 8.0 * 8.0;
        P0(8, 8) = 0.05 * 0.05;
        P0(9, 9) = 0.05 * 0.05;
        P0(10, 10) = 0.02 * 0.02;
        for (int i = 11; i < 14; ++i) P0(i, i) = params_.p0_accel_var;

        filter_ = Ekf(makeCaModel(params_), x0, P0, params_.R);
    }

    void predict(double dt) override { filter_.predict(dt); }
    void update(const Eigen::VectorXd& z) override { filter_.update(z); }
    void setMeasurementNoise(const Mat3& R) { filter_.setMeasurementNoise(R); }

    [[nodiscard]] Vec3 position() const override {
        const Eigen::VectorXd& x = filter_.state();
        const double ang = x[6] + params_.armor_id * 2.0 * 3.14159265358979323846 / params_.armor_num;
        const bool lh = (params_.armor_num == 4) && (params_.armor_id == 1 || params_.armor_id == 3);
        const double r = x[8] + (lh ? x[9] : 0.0);
        return Vec3(x[0] - r * std::cos(ang), x[2] - r * std::sin(ang), x[4] + (lh ? x[10] : 0.0));
    }

    [[nodiscard]] int stateDim() const override { return 14; }
    [[nodiscard]] int measDim() const override { return 3; }
    [[nodiscard]] Eigen::VectorXd state() const override { return filter_.state(); }
    [[nodiscard]] Eigen::MatrixXd covariance() const override { return filter_.covariance(); }
    [[nodiscard]] Eigen::VectorXd innovation() const override { return filter_.innovation(); }
    [[nodiscard]] Eigen::MatrixXd innovationCov() const override { return filter_.innovationCov(); }
    [[nodiscard]] double nis() const override { return filter_.nis(); }
    [[nodiscard]] double logLikelihood() const override { return filter_.logLikelihood(); }
    void setState(const Eigen::VectorXd& x, const Eigen::MatrixXd& P) override { filter_.setState(x, P); }

private:
    CaParams params_{};
    Ekf filter_{};
};

}  // namespace kf
