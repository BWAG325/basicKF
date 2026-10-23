#pragma once

#include <Eigen/Core>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>

#include "kf/ekf.hpp"

namespace kf {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

/// 协调转弯（coordinated turn）模型族，在平面内显式建模：
///   ψ   航向角 [rad]
///   v   线速度 [m/s]
///   ω   角速度（转弯率）[rad/s]
///   a   线加速度 [m/s²]（可选）
///   j   加加速度 [m/s³]（可选）
///   α   角加速度 [rad/s²]（可选）
///   z   高度（常数，弱过程噪声）
///
/// 状态布局（按 flag 动态拼）：
///   [x, y, ψ, v, (a), (j), ω, (α), z]
///
/// 过程方程：
///   a ← a + j·h,  v ← v + a·h,  ω ← ω + α·h,  ψ ← ψ + ω·h
///   x ← x + v·cosψ·h,  y ← y + v·sinψ·h
/// 用 h≈2ms 的子步积分。
struct CtParams {
    bool has_a = false;       ///< 线加速度状态
    bool has_j = false;       ///< 加加速度状态
    bool has_alpha = false;   ///< 角加速度状态

    double sigma_v = 2.0;      ///< 线加速度噪声 [m/s²]（作用于 v）
    double sigma_a = 5.0;      ///< jerk 噪声 [m/s³]（作用于 a）
    double sigma_j = 50.0;     ///< snap 噪声 [m/s⁴]（作用于 j）
    double sigma_w = 2.0;      ///< 角加速度噪声 [rad/s²]（作用于 ω）
    double sigma_alpha = 5.0;  ///< 角加加速度噪声 [rad/s³]（作用于 α）
    double sigma_z = 0.01;     ///< 高度过程噪声 [m/s]

    Mat3 R = Mat3::Identity() * 0.0001;
};

/// 状态布局。
struct CtLayout {
    int nx = 0;
    int x = 0, y = 0, psi = 0, v = 0, w = 0, z = 0;
    int a = -1, j = -1, alpha = -1;
};

inline CtLayout ctLayout(const CtParams& p) {
    CtLayout L;
    int k = 0;
    L.x = k++;
    L.y = k++;
    L.psi = k++;
    L.v = k++;
    if (p.has_a) L.a = k++;
    if (p.has_j) L.j = k++;
    L.w = k++;
    if (p.has_alpha) L.alpha = k++;
    L.z = k++;
    L.nx = k;
    return L;
}

/// 一步推进（含子步）。
inline Eigen::VectorXd ctStep(const Eigen::VectorXd& s, double dt, const CtLayout& L,
                              const CtParams& p, int steps) {
    double px = s[L.x], py = s[L.y], psi = s[L.psi], v = s[L.v], w = s[L.w];
    double a = p.has_a ? s[L.a] : 0.0;
    double j = p.has_j ? s[L.j] : 0.0;
    double alpha = p.has_alpha ? s[L.alpha] : 0.0;

    const int n = std::max(1, steps);
    const double h = dt / static_cast<double>(n);
    for (int i = 0; i < n; ++i) {
        a += j * h;
        v += a * h;
        w += alpha * h;
        psi += w * h;
        px += v * std::cos(psi) * h;
        py += v * std::sin(psi) * h;
    }

    Eigen::VectorXd out = s;
    out[L.x] = px;
    out[L.y] = py;
    out[L.psi] = psi;
    out[L.v] = v;
    out[L.w] = w;
    if (p.has_a) out[L.a] = a;
    if (p.has_j) out[L.j] = j;
    if (p.has_alpha) out[L.alpha] = alpha;
    return out;
}

inline Ekf::Model makeCtModel(const CtParams& p) {
    const CtLayout L = ctLayout(p);

    Ekf::Model m;
    m.nx = L.nx;
    m.nz = 3;

    const int steps = 4;  // 子步数（2ms 量级）
    m.f = [L, p, steps](const Eigen::VectorXd& s, double dt) {
        return ctStep(s, dt, L, p, steps);
    };

    m.Q = [L, p](double dt) {
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(L.nx, L.nx);
        Q(L.v, L.v) = p.sigma_v * p.sigma_v * dt;
        Q(L.w, L.w) = p.sigma_w * p.sigma_w * dt;
        Q(L.z, L.z) = p.sigma_z * p.sigma_z * dt;
        if (p.has_a) Q(L.a, L.a) = p.sigma_a * p.sigma_a * dt;
        if (p.has_j) Q(L.j, L.j) = p.sigma_j * p.sigma_j * dt;
        if (p.has_alpha) Q(L.alpha, L.alpha) = p.sigma_alpha * p.sigma_alpha * dt;
        return Q;
    };

    m.H = Eigen::MatrixXd::Zero(3, L.nx);
    m.H(0, L.x) = 1.0;
    m.H(1, L.y) = 1.0;
    m.H(2, L.z) = 1.0;
    return m;
}

class CtKalman final : public IFilter {
public:
    struct Estimate {
        Vec3 position = Vec3::Zero();
        Vec3 velocity = Vec3::Zero();
        double heading = 0.0;
        double speed = 0.0;
        double turn_rate = 0.0;
        double acceleration = 0.0;
        double angular_accel = 0.0;
    };

    CtKalman() = default;
    explicit CtKalman(CtParams params) : params_(std::move(params)) {}

    void init(const Vec3& position, const Vec3& velocity, double init_pos_sd = 0.05,
              double init_vel_sd = 1.5, double init_heading_sd = 0.5,
              double init_turn_sd = 2.0) {
        const CtLayout L = ctLayout(params_);
        Eigen::VectorXd x0 = Eigen::VectorXd::Zero(L.nx);
        x0[L.x] = position.x();
        x0[L.y] = position.y();
        x0[L.z] = position.z();
        x0[L.psi] = std::atan2(velocity.y(), velocity.x());
        x0[L.v] = std::hypot(velocity.x(), velocity.y());
        x0[L.w] = 0.0;
        if (params_.has_a) x0[L.a] = 0.0;
        if (params_.has_j) x0[L.j] = 0.0;
        if (params_.has_alpha) x0[L.alpha] = 0.0;

        Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(L.nx, L.nx);
        const double p2 = init_pos_sd * init_pos_sd;
        P0(L.x, L.x) = p2;
        P0(L.y, L.y) = p2;
        P0(L.z, L.z) = p2;
        P0(L.psi, L.psi) = init_heading_sd * init_heading_sd;
        P0(L.v, L.v) = init_vel_sd * init_vel_sd;
        P0(L.w, L.w) = init_turn_sd * init_turn_sd;
        if (params_.has_a) P0(L.a, L.a) = 2.0 * 2.0;
        if (params_.has_j) P0(L.j, L.j) = 5.0 * 5.0;
        if (params_.has_alpha) P0(L.alpha, L.alpha) = 2.0 * 2.0;

        filter_ = Ekf(makeCtModel(params_), x0, P0, params_.R);
    }

    void setProcessNoise(double sigma_v, double sigma_w) {
        params_.sigma_v = sigma_v;
        params_.sigma_w = sigma_w;
        filter_.setModel(makeCtModel(params_));
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
        const CtLayout L = ctLayout(params_);
        const Eigen::VectorXd& s = filter_.state();
        Estimate e;
        e.position = Vec3(s[L.x], s[L.y], s[L.z]);
        e.heading = s[L.psi];
        e.speed = s[L.v];
        e.turn_rate = s[L.w];
        e.velocity = Vec3(e.speed * std::cos(e.heading), e.speed * std::sin(e.heading), 0.0);
        e.acceleration = params_.has_a ? s[L.a] : 0.0;
        e.angular_accel = params_.has_alpha ? s[L.alpha] : 0.0;
        return e;
    }

    void update(const Vec3& z) { filter_.update(z); }

    [[nodiscard]] int stateDim() const override { return ctLayout(params_).nx; }
    [[nodiscard]] int measDim() const override { return 3; }

    void predict(double dt) override { filter_.predict(dt); }
    void update(const Eigen::VectorXd& z) override { filter_.update(z); }

    [[nodiscard]] Eigen::Vector3d position() const override { return estimate().position; }

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
    CtParams params_{};
    Ekf filter_{};
};

}  // namespace kf
