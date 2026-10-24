#pragma once

#include <Eigen/Core>

#include <cmath>
#include <memory>

#include "kf/ekf.hpp"
#include "kf/ukf.hpp"

namespace kf {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

/// 与 auto_aim 同一套观测：z = [yaw, pitch, distance, armor_angle]。
/// 几何/过程模型与 cv.hpp 相同（11 维），R 用自适应形式。
struct CvYpdParams {
    double q_xyz = 100.0;
    double q_yaw = 75.0;
    double armor_radius = 0.22;
    int armor_num = 4;
    int armor_id = 0;
};

inline double limitRad(double a) { return std::atan2(std::sin(a), std::cos(a)); }

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
        const double ang = x[6] + p.armor_id * 2.0 * M_PI / p.armor_num;
        const bool lh = (p.armor_num == 4) && (p.armor_id == 1 || p.armor_id == 3);
        const double r = x[8] + (lh ? x[9] : 0.0);
        const double ax = x[0] - r * std::cos(ang);
        const double ay = x[2] - r * std::sin(ang);
        const double az = x[4] + (lh ? x[10] : 0.0);
        Eigen::VectorXd z(4);
        const double q = std::sqrt(ax * ax + ay * ay);
        z[0] = std::atan2(ay, ax);
        z[1] = std::atan2(az, q);
        z[2] = std::sqrt(ax * ax + ay * ay + az * az);
        z[3] = limitRad(ang);
        return z;
    };

    m.Hx = [&p, h = m.h](const Eigen::VectorXd& x) {
        Eigen::MatrixXd H(4, 11);
        for (int j = 0; j < 11; ++j) {
            const double eps = 1e-6 * std::max(1.0, std::abs(x[j]));
            Eigen::VectorXd xp = x, xm = x;
            xp[j] += eps; xm[j] -= eps;
            Eigen::VectorXd dp = h(xp) - h(xm);
            for (int k = 0; k < 4; ++k) dp[k] = limitRad(dp[k]);
            H.col(j) = dp / (2 * eps);
        }
        return H;
    };

    m.Rz = [](const Eigen::VectorXd& z, const Eigen::VectorXd&) {
        const double delta = limitRad(z[3] - z[0]);
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(4, 4);
        (void)delta;
        const double d = std::max(z[2], 0.5);
        const double var_ang = (0.02 / d) * (0.02 / d);
        R(0, 0) = var_ang;
        R(1, 1) = var_ang;
        R(2, 2) = 0.02 * 0.02;
        R(3, 3) = 0.01 * 0.01;
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
        Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(11, 11);
        const double d[11] = {1, 64, 1, 64, 1, 64, 0.4, 100, 1e-5, 1, 1};
        for (int i = 0; i < 11; ++i) P0(i, i) = d[i];
        filter_ = UkfKalman(makeCvYpdModel(params_), x0, P0);
        filter_.setArmor(params_.armor_id, params_.armor_num);
    }

    void setMeasurementNoise(const Mat3&) {}
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
        Eigen::MatrixXd P0 = Eigen::VectorXd::Zero(11).asDiagonal();
        P0 = Eigen::VectorXd::Zero(11).asDiagonal();
        const double d[11] = {1, 64, 1, 64, 1, 64, 0.4, 100, 1e-5, 1, 1};
        for (int i = 0; i < 11; ++i) P0(i, i) = d[i];
        filter_ = Ekf(makeCvYpdModel(params_), x0, P0, Eigen::MatrixXd::Identity(4, 4));
    }

    void setMeasurementNoise(const Mat3&) {}

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
