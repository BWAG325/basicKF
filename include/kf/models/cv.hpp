#pragma once

#include <Eigen/Core>

#include <cmath>
#include <memory>
#include <utility>

#include "kf/ekf.hpp"

namespace kf {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

/// 复刻 auto_aim_ 的 11 维整车 EKF（观测改成装甲板三维位置）：
///   x = [cx, vx, cy, vy, cz, vz, yaw, yaw_rate, r, dr, dh]
///   armor(id) = [cx - r_eff·cos(a), cy - r_eff·sin(a), cz + (id∈{1,3} ? dh : 0)]
///   a = yaw + id·2π/N,  r_eff = r + (id∈{1,3} ? dr : 0)
struct CvParams {
    double q_xyz = 100.0;   ///< 位置/速度加速度方差
    double q_yaw = 75.0;    ///< 角加速度方差
    double armor_radius = 0.22;
    int armor_num = 4;
    int armor_id = 0;
    Mat3 R = Mat3::Identity() * 0.0004;
};

inline Ekf::Model makeCvModel(const CvParams& p) {
    Ekf::Model m;
    m.nx = 11;
    m.nz = 3;

    m.f = [](const Eigen::VectorXd& x, double dt) {
        Eigen::VectorXd y = x;
        y[0] += x[1] * dt;
        y[2] += x[3] * dt;
        y[4] += x[5] * dt;
        y[6] = std::atan2(std::sin(x[6] + x[7] * dt), std::cos(x[6] + x[7] * dt));
        return y;
    };

    m.Q = [p](double dt) {
        const double a = dt * dt * dt * dt / 4.0;
        const double b = dt * dt * dt / 2.0;
        const double c = dt * dt;
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(11, 11);
        for (int k : {0, 2, 4}) {
            Q(k, k) = a * p.q_xyz;
            Q(k, k + 1) = b * p.q_xyz;
            Q(k + 1, k) = b * p.q_xyz;
            Q(k + 1, k + 1) = c * p.q_xyz;
        }
        Q(6, 6) = a * p.q_yaw;
        Q(6, 7) = b * p.q_yaw;
        Q(7, 6) = b * p.q_yaw;
        Q(7, 7) = c * p.q_yaw;
        return Q;
    };

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
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 11);
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

class CvKalman final : public IFilter {
public:
    CvKalman() = default;
    explicit CvKalman(CvParams p) : params_(std::move(p)) {}

    void init(const Vec3& plate, const Vec3& velocity) {
        Eigen::VectorXd x0 = Eigen::VectorXd::Zero(11);
        x0[6] = 0.0;
        x0[8] = params_.armor_radius;
        x0[9] = 0.0;
        x0[10] = 0.0;
        x0[0] = plate.x() + x0[8];
        x0[2] = plate.y();
        x0[4] = plate.z();
        x0[1] = velocity.x();
        x0[3] = velocity.y();
        x0[5] = 0.0;
        x0[7] = 0.0;

        Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(11, 11);
        P0(0, 0) = P0(2, 2) = 0.5 * 0.5;
        P0(4, 4) = 0.05 * 0.05;
        P0(1, 1) = P0(3, 3) = 1.5 * 1.5;
        P0(5, 5) = 0.5 * 0.5;
        P0(6, 6) = 1.5707963267948966 * 1.5707963267948966;
        P0(7, 7) = 8.0 * 8.0;
        P0(8, 8) = 0.05 * 0.05;
        P0(9, 9) = 0.05 * 0.05;
        P0(10, 10) = 0.02 * 0.02;

        filter_ = Ekf(makeCvModel(params_), x0, P0, params_.R);
    }

    void update(const Vec3& z) { filter_.update(z); }
    void setMeasurementNoise(const Mat3& R) { filter_.setMeasurementNoise(R); }

    [[nodiscard]] Vec3 position() const override {
        const Eigen::VectorXd& x = filter_.state();
        const double ang = x[6] + params_.armor_id * 2.0 * 3.14159265358979323846 / params_.armor_num;
        const bool lh = (params_.armor_num == 4) && (params_.armor_id == 1 || params_.armor_id == 3);
        const double r = x[8] + (lh ? x[9] : 0.0);
        return Vec3(x[0] - r * std::cos(ang), x[2] - r * std::sin(ang), x[4] + (lh ? x[10] : 0.0));
    }

    [[nodiscard]] int stateDim() const override { return 11; }
    [[nodiscard]] int measDim() const override { return 3; }
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
    CvParams params_{};
    Ekf filter_{};
};

}  // namespace kf
