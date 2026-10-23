#pragma once

#include <Eigen/Core>

#include <cmath>
#include <memory>
#include <utility>

#include "kf/ekf.hpp"

namespace kf {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

/// 广义自转模型：装甲环可以偏心、也可以是椭圆。
///   p_plate = p_chassis + [ox + a·cosθ, oy + b·sinθ, 0]
/// 状态：[x, y, z, vx, vy, ox, oy, a, b, θ, ω]
/// 固定半径的 SPIN 是它的特例（ox=oy=0, a=b=r）。
struct SpinGenParams {
    double init_radius = 0.22;  ///< 初始 a/b
    double sigma_v = 2.0;
    double sigma_w = 2.0;
    double sigma_geom = 0.01;   ///< ox/oy/a/b 的随机游走强度
    double sigma_z = 0.01;
    Mat3 R = Mat3::Identity() * 0.0001;
};

struct SpinGenLayout {
    int nx = 11;
    int x = 0, y = 1, z = 2, vx = 3, vy = 4;
    int ox = 5, oy = 6, a = 7, b = 8, theta = 9, omega = 10;
};

inline Ekf::Model makeSpinGenModel(const SpinGenParams& p) {
    const SpinGenLayout L;
    Ekf::Model m;
    m.nx = L.nx;
    m.nz = 3;

    m.f = [L, p](const Eigen::VectorXd& s, double dt) {
        Eigen::VectorXd out = s;
        out[L.x] = s[L.x] + s[L.vx] * dt;
        out[L.y] = s[L.y] + s[L.vy] * dt;
        out[L.theta] = s[L.theta] + s[L.omega] * dt;
        return out;
    };

    m.Q = [L, p](double dt) {
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(L.nx, L.nx);
        Q(L.vx, L.vx) = p.sigma_v * p.sigma_v * dt;
        Q(L.vy, L.vy) = p.sigma_v * p.sigma_v * dt;
        Q(L.omega, L.omega) = p.sigma_w * p.sigma_w * dt;
        Q(L.z, L.z) = p.sigma_z * p.sigma_z * dt;
        const double g = p.sigma_geom * p.sigma_geom * dt;
        Q(L.ox, L.ox) = g;
        Q(L.oy, L.oy) = g;
        Q(L.a, L.a) = g;
        Q(L.b, L.b) = g;
        return Q;
    };

    m.h = [L](const Eigen::VectorXd& s) {
        Eigen::VectorXd h = Eigen::VectorXd::Zero(3);
        h(0) = s[L.x] + s[L.ox] + s[L.a] * std::cos(s[L.theta]);
        h(1) = s[L.y] + s[L.oy] + s[L.b] * std::sin(s[L.theta]);
        h(2) = s[L.z];
        return h;
    };

    m.Hx = [L](const Eigen::VectorXd& s) {
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, L.nx);
        H(0, L.x) = 1.0;
        H(1, L.y) = 1.0;
        H(2, L.z) = 1.0;
        H(0, L.ox) = 1.0;
        H(1, L.oy) = 1.0;
        H(0, L.a) = std::cos(s[L.theta]);
        H(1, L.b) = std::sin(s[L.theta]);
        H(0, L.theta) = -s[L.a] * std::sin(s[L.theta]);
        H(1, L.theta) = s[L.b] * std::cos(s[L.theta]);
        return H;
    };

    return m;
}

class SpinGenKalman final : public IFilter {
public:
    struct Estimate {
        Vec3 plate = Vec3::Zero();
        Vec3 chassis = Vec3::Zero();
        double ox = 0.0, oy = 0.0, a = 0.0, b = 0.0;
        double theta = 0.0, omega = 0.0;
    };

    SpinGenKalman() = default;
    explicit SpinGenKalman(SpinGenParams p) : params_(std::move(p)) {}

    void init(const Vec3& plate, const Vec3& velocity, double init_pos_sd = 0.5,
              double init_vel_sd = 1.5, double init_theta_sd = 1.5707963267948966,
              double init_omega_sd = 8.0) {
        const SpinGenLayout L;
        Eigen::VectorXd x0 = Eigen::VectorXd::Zero(L.nx);
        x0[L.ox] = 0.0;
        x0[L.oy] = 0.0;
        x0[L.a] = params_.init_radius;
        x0[L.b] = params_.init_radius;
        x0[L.theta] = 0.0;
        x0[L.x] = plate.x() - x0[L.ox] - x0[L.a];
        x0[L.y] = plate.y() - x0[L.oy];
        x0[L.z] = plate.z();
        x0[L.vx] = velocity.x();
        x0[L.vy] = velocity.y();
        x0[L.omega] = 0.0;

        Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(L.nx, L.nx);
        P0(L.x, L.x) = init_pos_sd * init_pos_sd;
        P0(L.y, L.y) = init_pos_sd * init_pos_sd;
        P0(L.z, L.z) = 0.05 * 0.05;
        P0(L.vx, L.vx) = init_vel_sd * init_vel_sd;
        P0(L.vy, L.vy) = init_vel_sd * init_vel_sd;
        P0(L.theta, L.theta) = init_theta_sd * init_theta_sd;
        P0(L.omega, L.omega) = init_omega_sd * init_omega_sd;
        P0(L.ox, L.ox) = 0.1 * 0.1;
        P0(L.oy, L.oy) = 0.1 * 0.1;
        P0(L.a, L.a) = 0.05 * 0.05;
        P0(L.b, L.b) = 0.05 * 0.05;

        filter_ = Ekf(makeSpinGenModel(params_), x0, P0, params_.R);
    }

    void setMeasurementNoise(const Mat3& R) {
        params_.R = R;
        filter_.setMeasurementNoise(R);
    }

    [[nodiscard]] Estimate estimate() const {
        const SpinGenLayout L;
        const Eigen::VectorXd& s = filter_.state();
        Estimate e;
        e.ox = s[L.ox];
        e.oy = s[L.oy];
        e.a = s[L.a];
        e.b = s[L.b];
        e.theta = s[L.theta];
        e.omega = s[L.omega];
        e.chassis = Vec3(s[L.x], s[L.y], s[L.z]);
        e.plate = e.chassis + Vec3(e.ox + e.a * std::cos(e.theta),
                                   e.oy + e.b * std::sin(e.theta), 0.0);
        return e;
    }

    void update(const Vec3& z) { filter_.update(z); }

    [[nodiscard]] int stateDim() const override { return SpinGenLayout{}.nx; }
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
    SpinGenParams params_{};
    Ekf filter_{};
};

}  // namespace kf
