#pragma once

#include <Eigen/Core>

#include <cmath>
#include <memory>
#include <utility>

#include "kf/ekf.hpp"

namespace kf {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

/// 自转感知模型：
///   底盘做匀速平移，装甲板绕底盘以角度 θ 旋转，观测 = 底盘 + r·(cosθ, sinθ)。
/// 状态：[x, y, z, vx, vy, θ, ω, (α)]
/// 这正是小陀螺工况的物理结构，能显式利用角速度/角加速度。
struct SpinParams {
    double armor_radius = 0.22;  ///< 装甲板到车体中心的距离 [m]
    bool has_alpha = false;      ///< 角加速度状态

    double sigma_v = 2.0;        ///< 底盘加速度噪声 [m/s²]
    double sigma_w = 2.0;        ///< 角加速度噪声 [rad/s²]（作用于 ω）
    double sigma_alpha = 5.0;    ///< 角加加速度噪声 [rad/s³]（作用于 α）
    double sigma_z = 0.01;       ///< 高度过程噪声 [m/s]

    Mat3 R = Mat3::Identity() * 0.0001;
};

struct SpinLayout {
    int nx = 0;
    int x = 0, y = 1, z = 2, vx = 3, vy = 4, theta = 5, omega = 6, alpha = -1;
};

inline SpinLayout spinLayout(const SpinParams& p) {
    SpinLayout L;
    L.x = 0; L.y = 1; L.z = 2; L.vx = 3; L.vy = 4; L.theta = 5; L.omega = 6;
    L.alpha = p.has_alpha ? 7 : -1;
    L.nx = p.has_alpha ? 8 : 7;
    return L;
}

inline Ekf::Model makeSpinModel(const SpinParams& p) {
    const SpinLayout L = spinLayout(p);

    Ekf::Model m;
    m.nx = L.nx;
    m.nz = 3;

    m.f = [L, p](const Eigen::VectorXd& s, double dt) {
        Eigen::VectorXd out = s;
        const double alpha = p.has_alpha ? s[L.alpha] : 0.0;
        out[L.x] = s[L.x] + s[L.vx] * dt;
        out[L.y] = s[L.y] + s[L.vy] * dt;
        out[L.theta] = s[L.theta] + s[L.omega] * dt + 0.5 * alpha * dt * dt;
        out[L.omega] = s[L.omega] + alpha * dt;
        return out;
    };

    m.Q = [L, p](double dt) {
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(L.nx, L.nx);
        Q(L.vx, L.vx) = p.sigma_v * p.sigma_v * dt;
        Q(L.vy, L.vy) = p.sigma_v * p.sigma_v * dt;
        Q(L.omega, L.omega) = p.sigma_w * p.sigma_w * dt;
        Q(L.z, L.z) = p.sigma_z * p.sigma_z * dt;
        if (p.has_alpha) Q(L.alpha, L.alpha) = p.sigma_alpha * p.sigma_alpha * dt;
        return Q;
    };

    // h(x)：观测方程（非线性）；Hx 是它的雅可比
    m.h = [L, p](const Eigen::VectorXd& s) {
        Eigen::VectorXd h = Eigen::VectorXd::Zero(3);
        h(0) = s[L.x] + p.armor_radius * std::cos(s[L.theta]);
        h(1) = s[L.y] + p.armor_radius * std::sin(s[L.theta]);
        h(2) = s[L.z];
        return h;
    };

    m.Hx = [L, p](const Eigen::VectorXd& s) {
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, L.nx);
        H(0, L.x) = 1.0;
        H(1, L.y) = 1.0;
        H(2, L.z) = 1.0;
        H(0, L.theta) = -p.armor_radius * std::sin(s[L.theta]);
        H(1, L.theta) = p.armor_radius * std::cos(s[L.theta]);
        return H;
    };

    return m;
}

class SpinKalman final : public IFilter {
public:
    struct Estimate {
        Vec3 plate = Vec3::Zero();
        Vec3 chassis = Vec3::Zero();
        Vec3 velocity = Vec3::Zero();
        double theta = 0.0;
        double omega = 0.0;
        double alpha = 0.0;
    };

    SpinKalman() = default;
    explicit SpinKalman(SpinParams p) : params_(std::move(p)) {}

    void init(const Vec3& plate, const Vec3& velocity, double init_pos_sd = 0.5,
              double init_vel_sd = 1.5, double init_theta_sd = 1.5707963267948966,
              double init_omega_sd = 8.0) {
        const SpinLayout L = spinLayout(params_);
        Eigen::VectorXd x0 = Eigen::VectorXd::Zero(L.nx);
        x0[L.theta] = 0.0;
        x0[L.x] = plate.x() - params_.armor_radius * std::cos(x0[L.theta]);
        x0[L.y] = plate.y() - params_.armor_radius * std::sin(x0[L.theta]);
        x0[L.z] = plate.z();
        x0[L.vx] = velocity.x();
        x0[L.vy] = velocity.y();
        x0[L.omega] = 0.0;
        if (params_.has_alpha) x0[L.alpha] = 0.0;

        Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(L.nx, L.nx);
        P0(L.x, L.x) = init_pos_sd * init_pos_sd;
        P0(L.y, L.y) = init_pos_sd * init_pos_sd;
        P0(L.z, L.z) = 0.05 * 0.05;
        P0(L.vx, L.vx) = init_vel_sd * init_vel_sd;
        P0(L.vy, L.vy) = init_vel_sd * init_vel_sd;
        P0(L.theta, L.theta) = init_theta_sd * init_theta_sd;
        P0(L.omega, L.omega) = init_omega_sd * init_omega_sd;
        if (params_.has_alpha) P0(L.alpha, L.alpha) = 5.0 * 5.0;

        filter_ = Ekf(makeSpinModel(params_), x0, P0, params_.R);
    }

    void setProcessNoise(double sigma_v, double sigma_w) {
        params_.sigma_v = sigma_v;
        params_.sigma_w = sigma_w;
        filter_.setModel(makeSpinModel(params_));
    }

    void setMeasurementNoise(const Mat3& R) {
        params_.R = R;
        filter_.setMeasurementNoise(R);
    }

    void setMeasurementNoise(double sigma) {
        params_.R = Mat3::Identity() * sigma * sigma;
        filter_.setMeasurementNoise(params_.R);
    }

    [[nodiscard]] Estimate estimate() const {
        const SpinLayout L = spinLayout(params_);
        const Eigen::VectorXd& s = filter_.state();
        Estimate e;
        e.chassis = Vec3(s[L.x], s[L.y], s[L.z]);
        e.velocity = Vec3(s[L.vx], s[L.vy], 0.0);
        e.theta = s[L.theta];
        e.omega = s[L.omega];
        e.alpha = params_.has_alpha ? s[L.alpha] : 0.0;
        e.plate = e.chassis + Vec3(params_.armor_radius * std::cos(e.theta),
                                   params_.armor_radius * std::sin(e.theta), 0.0);
        return e;
    }

    void update(const Vec3& z) { filter_.update(z); }

    [[nodiscard]] int stateDim() const override { return spinLayout(params_).nx; }
    [[nodiscard]] int measDim() const override { return 3; }
    [[nodiscard]] Eigen::Vector3d position() const override { return estimate().plate; }

    void predict(double dt) override { filter_.predict(dt); }
    void update(const Eigen::VectorXd& z) override { filter_.update(z); }

    [[nodiscard]] Eigen::VectorXd state() const override { return filter_.state(); }
    [[nodiscard]] Eigen::MatrixXd covariance() const override { return filter_.covariance(); }
    [[nodiscard]] Eigen::VectorXd innovation() const override { return filter_.innovation(); }
    [[nodiscard]] Eigen::MatrixXd innovationCov() const override { return filter_.innovationCov(); }
    [[nodiscard]] double nis() const override { return filter_.nis(); }
    [[nodiscard]] double logLikelihood() const override { return filter_.logLikelihood(); }

    void setState(const Eigen::VectorXd& x, const Eigen::MatrixXd& P) override {
        filter_.setState(x, P);
    }

private:
    SpinParams params_{};
    Ekf filter_{};
};

}  // namespace kf
