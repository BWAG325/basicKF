#pragma once

#include <Eigen/Core>

#include <cmath>
#include <memory>

#include "kf/ekf.hpp"
#include "kf/ukf.hpp"

namespace kf {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

/// auto_aim Target 的 11 维整车 EKF：
///   x = [cx, vx, cy, vy, cz, vz, yaw, yaw_rate, r, l, h]
///   z = [yaw, pitch, distance, armor_angle]
/// F/Q 是分段白噪声，H = ∂(ypd,angle)/∂xyz · ∂xyz/∂x，R 用哨兵配置那组自适应对角。
struct CvYpdParams {
    double q_xyz = 100.0;
    double q_yaw = 75.0;
    double r_yaw = 4.0e-2;
    double r_pitch = 4.0e-2;
    double r_dist_base = 1.5e-2;
    // true：R 的形状仍随斜视放大，基准改成仿真传感器（位置、法向）。
    // false：auto_aim 哨兵配置的原始数值。
    bool match_sensor = false;
    double position_sigma = 0.02;
    double angle_sigma = 0.01;
    double armor_radius = 0.2;
    int armor_num = 4;
    int armor_id = 0;
};

inline double limitRad(double a) { return std::atan2(std::sin(a), std::cos(a)); }

inline Eigen::VectorXd autoAimP0() {
    Eigen::VectorXd d(11);
    d << 1, 64, 1, 64, 1, 64, 0.4, 400, 1.0e-5, 1, 1;
    return d;
}

inline Eigen::Vector3d xyz2ypd(const Eigen::Vector3d& xyz) {
    const double x = xyz.x(), y = xyz.y(), z = xyz.z();
    return Eigen::Vector3d(std::atan2(y, x), std::atan2(z, std::sqrt(x * x + y * y)), xyz.norm());
}

inline Eigen::Matrix3d xyz2ypdJacobian(const Eigen::Vector3d& xyz) {
    const double x = xyz.x(), y = xyz.y(), z = xyz.z();
    const double xy2 = x * x + y * y;
    const double pitch_den = (z * z / xy2 + 1.0);
    Eigen::Matrix3d J;
    J(0, 0) = -y / xy2;
    J(0, 1) = x / xy2;
    J(0, 2) = 0.0;
    J(1, 0) = -(x * z) / (pitch_den * std::pow(xy2, 1.5));
    J(1, 1) = -(y * z) / (pitch_den * std::pow(xy2, 1.5));
    J(1, 2) = 1.0 / (pitch_den * std::sqrt(xy2));
    const double dist = std::sqrt(xy2 + z * z);
    J(2, 0) = x / dist;
    J(2, 1) = y / dist;
    J(2, 2) = z / dist;
    return J;
}

inline Eigen::Vector3d armorXyz(const Eigen::VectorXd& x, int id, int armor_num) {
    const double angle = limitRad(x[6] + id * 2.0 * M_PI / armor_num);
    const bool use_lh = armor_num == 4 && (id == 1 || id == 3);
    const double r = use_lh ? x[8] + x[9] : x[8];
    const double az = use_lh ? x[4] + x[10] : x[4];
    return {x[0] - r * std::cos(angle), x[2] - r * std::sin(angle), az};
}

inline Eigen::MatrixXd armorYpdaJacobian(const Eigen::VectorXd& x, int id, int armor_num) {
    const double angle = limitRad(x[6] + id * 2.0 * M_PI / armor_num);
    const bool use_lh = armor_num == 4 && (id == 1 || id == 3);
    const double r = use_lh ? x[8] + x[9] : x[8];
    const double dx_da = r * std::sin(angle);
    const double dy_da = -r * std::cos(angle);
    const double dx_dr = -std::cos(angle);
    const double dy_dr = -std::sin(angle);
    const double dx_dl = use_lh ? dx_dr : 0.0;
    const double dy_dl = use_lh ? dy_dr : 0.0;
    const double dz_dh = use_lh ? 1.0 : 0.0;

    Eigen::MatrixXd H_xyza(4, 11);
    H_xyza.setZero();
    H_xyza(0, 0) = 1;
    H_xyza(0, 6) = dx_da;
    H_xyza(0, 8) = dx_dr;
    H_xyza(0, 9) = dx_dl;
    H_xyza(1, 2) = 1;
    H_xyza(1, 6) = dy_da;
    H_xyza(1, 8) = dy_dr;
    H_xyza(1, 9) = dy_dl;
    H_xyza(2, 4) = 1;
    H_xyza(2, 10) = dz_dh;
    H_xyza(3, 6) = 1;

    const Eigen::Matrix3d J = xyz2ypdJacobian(armorXyz(x, id, armor_num));
    Eigen::MatrixXd H_ypda(4, 4);
    H_ypda.setZero();
    H_ypda.block<3, 3>(0, 0) = J;
    H_ypda(3, 3) = 1;
    return H_ypda * H_xyza;
}

inline Ekf::Model makeCvYpdModel(const CvYpdParams& p) {
    Ekf::Model m;
    m.nx = 11;
    m.nz = 4;

    m.f = [](const Eigen::VectorXd& x, double dt) {
        Eigen::VectorXd y = x;
        y[0] += x[1] * dt;
        y[2] += x[3] * dt;
        y[4] += x[5] * dt;
        y[6] = limitRad(x[6] + x[7] * dt);
        return y;
    };

    m.Fx = [](const Eigen::VectorXd&, double dt) {
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(11, 11);
        F(0, 1) = dt;
        F(2, 3) = dt;
        F(4, 5) = dt;
        F(6, 7) = dt;
        return F;
    };

    m.xadd = [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
        Eigen::VectorXd c = a + b;
        c[6] = limitRad(c[6]);
        return c;
    };

    m.Q = [p](double dt) {
        const double a = dt * dt * dt * dt / 4.0, b = dt * dt * dt / 2.0, c = dt * dt;
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(11, 11);
        for (int k : {0, 2, 4}) {
            Q(k, k) = a * p.q_xyz; Q(k, k + 1) = b * p.q_xyz;
            Q(k + 1, k) = b * p.q_xyz; Q(k + 1, k + 1) = c * p.q_xyz;
        }
        Q(6, 6) = a * p.q_yaw; Q(6, 7) = b * p.q_yaw;
        Q(7, 6) = b * p.q_yaw; Q(7, 7) = c * p.q_yaw;
        return Q;
    };

    m.h = [p](const Eigen::VectorXd& x) {
        const Eigen::Vector3d xyz = armorXyz(x, p.armor_id, p.armor_num);
        const Eigen::Vector3d ypd = xyz2ypd(xyz);
        Eigen::VectorXd z(4);
        z[0] = ypd[0];
        z[1] = ypd[1];
        z[2] = ypd[2];
        z[3] = limitRad(x[6] + p.armor_id * 2.0 * M_PI / p.armor_num);
        return z;
    };

    m.Hx = [p](const Eigen::VectorXd& x) {
        return armorYpdaJacobian(x, p.armor_id, p.armor_num);
    };

    m.Rz = [p](const Eigen::VectorXd& z, const Eigen::VectorXd&) {
        const double delta = limitRad(z[3] - z[0]);
        const double oblique = std::log(std::abs(delta) + 1.0) + 1.0;
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(4, 4);
        if (p.match_sensor) {
            const double horiz = std::max(0.3, std::abs(z[2] * std::cos(z[1])));
            const double bearing = (p.position_sigma / horiz) * (p.position_sigma / horiz);
            R(0, 0) = bearing;
            R(1, 1) = bearing;
            R(2, 2) = p.position_sigma * p.position_sigma * oblique;
            R(3, 3) = p.angle_sigma * p.angle_sigma * oblique;
            return R;
        }
        R(0, 0) = p.r_yaw;
        R(1, 1) = p.r_pitch;
        R(2, 2) = oblique;
        R(3, 3) = std::log(std::abs(z[2]) + 1.0) / 200.0 + p.r_dist_base;
        return R;
    };

    m.zdiff = [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
        Eigen::VectorXd d = a - b;
        d[0] = limitRad(d[0]);
        d[1] = limitRad(d[1]);
        d[3] = limitRad(d[3]);
        return d;
    };
    return m;
}

class CvYpdUkf final : public IFilter {
public:
    CvYpdUkf() = default;
    explicit CvYpdUkf(CvYpdParams p) : params_(std::move(p)) {}

    void init(const Vec3& armor_xyz, double yaw) {
        const double r = params_.armor_radius;
        Eigen::VectorXd x0 = Eigen::VectorXd::Zero(11);
        x0[6] = limitRad(yaw);
        x0[8] = r;
        x0[0] = armor_xyz.x() + r * std::cos(x0[6]);
        x0[2] = armor_xyz.y() + r * std::sin(x0[6]);
        x0[4] = armor_xyz.z();
        const Eigen::MatrixXd P0 = autoAimP0().asDiagonal();
        filter_ = UkfKalman(makeCvYpdModel(params_), x0, P0);
        filter_.setArmor(params_.armor_id, params_.armor_num);
    }

    [[nodiscard]] Vec3 position() const override { return filter_.position(); }
    [[nodiscard]] int stateDim() const override { return 11; }
    [[nodiscard]] int measDim() const override { return 4; }
    void predict(double dt) override { filter_.predict(dt); }
    void update(const Eigen::VectorXd& z) override { filter_.update(z); }
    [[nodiscard]] Eigen::VectorXd state() const override { return filter_.state(); }
    [[nodiscard]] Eigen::MatrixXd covariance() const override { return filter_.covariance(); }
    [[nodiscard]] Eigen::VectorXd innovation() const override { return filter_.innovation(); }
    [[nodiscard]] Eigen::MatrixXd innovationCov() const override { return filter_.innovationCov(); }
    [[nodiscard]] double nis() const override { return filter_.nis(); }
    [[nodiscard]] double logLikelihood() const override { return filter_.logLikelihood(); }
    void setState(const Eigen::VectorXd& x, const Eigen::MatrixXd& P) override { filter_.setState(x, P); }

private:
    CvYpdParams params_{};
    UkfKalman filter_{};
};

class CvYpdKalman final : public IFilter {
public:
    CvYpdKalman() = default;
    explicit CvYpdKalman(CvYpdParams p) : params_(std::move(p)) {}

    void init(const Vec3& armor_xyz, double yaw) {
        const double r = params_.armor_radius;
        Eigen::VectorXd x0 = Eigen::VectorXd::Zero(11);
        x0[6] = limitRad(yaw);
        x0[8] = r;
        x0[0] = armor_xyz.x() + r * std::cos(x0[6]);
        x0[2] = armor_xyz.y() + r * std::sin(x0[6]);
        x0[4] = armor_xyz.z();
        const Eigen::MatrixXd P0 = autoAimP0().asDiagonal();
        filter_ = Ekf(makeCvYpdModel(params_), x0, P0, Eigen::MatrixXd::Identity(4, 4));
    }

    void setMeasurementNoise(const Eigen::MatrixXd& R) override { filter_.setMeasurementNoise(R); }

    [[nodiscard]] Vec3 position() const override {
        const Eigen::VectorXd& x = filter_.state();
        const double ang = x[6] + params_.armor_id * 2.0 * M_PI / params_.armor_num;
        const bool lh = (params_.armor_num == 4) && (params_.armor_id == 1 || params_.armor_id == 3);
        const double r = x[8] + (lh ? x[9] : 0.0);
        return Vec3(x[0] - r * std::cos(ang), x[2] - r * std::sin(ang), x[4] + (lh ? x[10] : 0.0));
    }

    [[nodiscard]] int stateDim() const override { return 11; }
    [[nodiscard]] int measDim() const override { return 4; }
    void predict(double dt) override { filter_.predict(dt); }
    void update(const Eigen::VectorXd& z) override { filter_.update(z); }
    [[nodiscard]] Eigen::VectorXd state() const override { return filter_.state(); }
    [[nodiscard]] Eigen::MatrixXd covariance() const override { return filter_.covariance(); }
    [[nodiscard]] Eigen::VectorXd innovation() const override { return filter_.innovation(); }
    [[nodiscard]] Eigen::MatrixXd innovationCov() const override { return filter_.innovationCov(); }
    [[nodiscard]] double nis() const override { return filter_.nis(); }
    [[nodiscard]] double logLikelihood() const override { return filter_.logLikelihood(); }
    void setState(const Eigen::VectorXd& x, const Eigen::MatrixXd& P) override {
        Eigen::VectorXd y = x;
        y[6] = limitRad(y[6]);
        filter_.setState(y, P);
    }

private:
    CvYpdParams params_{};
    Ekf filter_{};
};

}  // namespace kf
