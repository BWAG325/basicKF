#pragma once

#include <Eigen/Core>

#include <cmath>
#include <utility>

#include "kf/models/cv_ypd.hpp"

namespace kf {

/// 常加速度版的 auto_aim 整车模型。观测与 CvYpdKalman 相同，
/// 状态在 11 维后面加平移加速度 [ax, ay, az]。yaw 仍是常角速度。
struct CaYpdParams {
    double q_jerk = 800.0;
    double q_yaw = 75.0;
    double r_yaw = 4.0e-2;
    double r_pitch = 4.0e-2;
    double r_dist_base = 1.5e-2;
    bool match_sensor = false;
    double position_sigma = 0.02;
    double angle_sigma = 0.01;
    double armor_radius = 0.2;
    int armor_num = 4;
    int armor_id = 0;
    double p0_accel_var = 25.0;
};

inline Ekf::Model makeCaYpdModel(const CaYpdParams& p) {
    Ekf::Model m;
    m.nx = 14;
    m.nz = 4;

    m.f = [](const Eigen::VectorXd& x, double dt) {
        Eigen::VectorXd y = x;
        for (int axis = 0; axis < 3; ++axis) {
            const int ip = 2 * axis;
            const int iv = ip + 1;
            const int ia = 11 + axis;
            const double a = x[ia];
            y[ip] += x[iv] * dt + 0.5 * a * dt * dt;
            y[iv] += a * dt;
        }
        y[6] = limitRad(x[6] + x[7] * dt);
        return y;
    };

    m.Fx = [](const Eigen::VectorXd&, double dt) {
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(14, 14);
        const double half = 0.5 * dt * dt;
        for (int axis = 0; axis < 3; ++axis) {
            const int ip = 2 * axis;
            F(ip, ip + 1) = dt;
            F(ip, 11 + axis) = half;
            F(ip + 1, 11 + axis) = dt;
        }
        F(6, 7) = dt;
        return F;
    };

    m.xadd = [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
        Eigen::VectorXd c = a + b;
        c[6] = limitRad(c[6]);
        return c;
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
            Q(ia, iv) = e * q_jerk;
            Q(ia, ia) = f * q_jerk;
        }
        const double ya = dt * dt * dt * dt / 4.0;
        const double yb = dt * dt * dt / 2.0;
        const double yc = dt * dt;
        Q(6, 6) = ya * q_yaw;
        Q(6, 7) = yb * q_yaw;
        Q(7, 6) = yb * q_yaw;
        Q(7, 7) = yc * q_yaw;
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
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(4, 14);
        H.leftCols(11) = armorYpdaJacobian(x, p.armor_id, p.armor_num);
        return H;
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

class CaYpdKalman final : public IFilter {
public:
    CaYpdKalman() = default;
    explicit CaYpdKalman(CaYpdParams p) : params_(std::move(p)) {}

    void init(const Vec3& armor_xyz, double yaw) {
        const double r = params_.armor_radius;
        Eigen::VectorXd x0 = Eigen::VectorXd::Zero(14);
        x0[6] = limitRad(yaw);
        x0[8] = r;
        x0[0] = armor_xyz.x() + r * std::cos(x0[6]);
        x0[2] = armor_xyz.y() + r * std::sin(x0[6]);
        x0[4] = armor_xyz.z();
        Eigen::VectorXd diag = Eigen::VectorXd::Zero(14);
        diag.head(11) = autoAimP0();
        diag[11] = diag[12] = diag[13] = params_.p0_accel_var;
        filter_ = Ekf(makeCaYpdModel(params_), x0, diag.asDiagonal(), Eigen::MatrixXd::Identity(4, 4));
    }

    void setMeasurementNoise(const Eigen::MatrixXd& R) override { filter_.setMeasurementNoise(R); }

    [[nodiscard]] Vec3 position() const override {
        const Eigen::VectorXd& x = filter_.state();
        const double ang = x[6] + params_.armor_id * 2.0 * M_PI / params_.armor_num;
        const bool lh = (params_.armor_num == 4) && (params_.armor_id == 1 || params_.armor_id == 3);
        const double r = x[8] + (lh ? x[9] : 0.0);
        return Vec3(x[0] - r * std::cos(ang), x[2] - r * std::sin(ang), x[4] + (lh ? x[10] : 0.0));
    }

    [[nodiscard]] int stateDim() const override { return 14; }
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
        if (y.size() > 6) y[6] = limitRad(y[6]);
        filter_.setState(y, P);
    }

private:
    CaYpdParams params_{};
    Ekf filter_{};
};

}  // namespace kf
