#pragma once

#include <Eigen/Core>

#include <cmath>
#include <memory>

#include "kf/kalman_filter.hpp"

namespace kf {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

/// 路线 A：板运动学 CV 模型，状态 [p, v, ω]（7 维）。
/// p/v 是装甲板的位置/速度，ω 是环的角速度（标量，不是角度）。
/// 观测直接是板位置，线性 H，所以可以安全地做严格 IMM。
struct PlateCvParams {
    double sigma_v = 2.0;   ///< 板加速度噪声 [m/s²]
    double sigma_w = 2.0;   ///< 角速度随机游走 [rad/s²]
    Mat3 R = Mat3::Identity() * 0.0004;
};

using PlateCvFilter = KalmanFilter<7, 3>;

inline PlateCvFilter::StateMat plateCvTransition(double dt) {
    PlateCvFilter::StateMat F = PlateCvFilter::StateMat::Identity();
    F.block<3, 3>(0, 3) = dt * Mat3::Identity();
    return F;
}

inline PlateCvFilter::StateMat plateCvProcessNoise(double dt, const PlateCvParams& p) {
    PlateCvFilter::StateMat Q = PlateCvFilter::StateMat::Zero();
    Q.block<3, 3>(3, 3) = (p.sigma_v * p.sigma_v * dt) * Mat3::Identity();
    Q(6, 6) = p.sigma_w * p.sigma_w * dt;
    return Q;
}

inline PlateCvFilter::MeasStateMat plateCvObservation() {
    PlateCvFilter::MeasStateMat H = PlateCvFilter::MeasStateMat::Zero();
    H.leftCols<3>().setIdentity();
    return H;
}

inline PlateCvFilter::Model makePlateCvModel(const PlateCvParams& p) {
    PlateCvFilter::Model m;
    m.F = [](double dt) { return plateCvTransition(dt); };
    m.Q = [p](double dt) { return plateCvProcessNoise(dt, p); };
    m.H = [] { return plateCvObservation(); };
    m.R = [p](const PlateCvFilter::MeasVec&) { return p.R; };
    return m;
}

class PlateCvKalman final : public IFilter {
public:
    PlateCvKalman() = default;
    explicit PlateCvKalman(PlateCvParams p) : params_(std::move(p)) {}

    void init(const Vec3& p, const Vec3& v, double pos_sd = 0.05, double vel_sd = 1.5,
              double w_sd = 8.0) {
        PlateCvFilter::StateVec x0 = PlateCvFilter::StateVec::Zero();
        x0.head<3>() = p;
        x0.segment<3>(3) = v;
        x0[6] = 0.0;

        PlateCvFilter::StateMat P0 = PlateCvFilter::StateMat::Zero();
        P0.block<3, 3>(0, 0) = Mat3::Identity() * pos_sd * pos_sd;
        P0.block<3, 3>(3, 3) = Mat3::Identity() * vel_sd * vel_sd;
        P0(6, 6) = w_sd * w_sd;

        filter_ = PlateCvFilter(makePlateCvModel(params_), x0, P0);
    }

    void setMeasurementNoise(const Mat3& R) {
        params_.R = R;
        filter_.setModel(makePlateCvModel(params_));
    }

    void setProcessNoise(double sigma_v, double sigma_w) {
        params_.sigma_v = sigma_v;
        params_.sigma_w = sigma_w;
        filter_.setModel(makePlateCvModel(params_));
    }

    [[nodiscard]] Vec3 position() const override { return filter_.stateVec().head<3>(); }

    [[nodiscard]] int stateDim() const override { return 7; }
    [[nodiscard]] int measDim() const override { return 3; }
    void predict(double dt) override { filter_.predict(dt); }
    void update(const Eigen::VectorXd& z) override { filter_.update(z); }
    void update(const Vec3& z) { filter_.update(z); }
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
    PlateCvParams params_{};
    PlateCvFilter filter_{};
};

}  // namespace kf
