#pragma once

#include <utility>

#include "kf/kalman_filter.hpp"

namespace kf {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

/// 三维 CA（匀加速）模型参数。
struct CaParams {
    double sigma_jerk = 10.0;             ///< 加加速度噪声 std [m/s³] → Q
    Mat3 R = Mat3::Identity() * 0.0001;   ///< 观测噪声协方差 [m²]
};

/// CA 滤波器的固定尺寸类型：状态 [p, v, a]（9 维），观测 [p]（3 维）。
using CaKalmanFilter = KalmanFilter<9, 3>;

// ---------------------------------------------------------------------------
// 模型矩阵
// ---------------------------------------------------------------------------

/// F(dt) = [I  dt·I  dt²/2·I]
///         [0    I     dt·I ]
///         [0    0       I  ]
inline CaKalmanFilter::StateMat caTransition(double dt) {
    CaKalmanFilter::StateMat F = CaKalmanFilter::StateMat::Identity();
    F.block<3, 3>(0, 3) = dt * Mat3::Identity();
    F.block<3, 3>(0, 6) = 0.5 * dt * dt * Mat3::Identity();
    F.block<3, 3>(3, 6) = dt * Mat3::Identity();
    return F;
}

/// Q(dt)：离散白噪声加加速度（jerk）模型。
/// 单轴连续时间积分：
///   [dt⁵/20  dt⁴/8  dt³/6]
///   [dt⁴/8   dt³/3  dt²/2] · σ_j²
///   [dt³/6   dt²/2  dt   ]
inline CaKalmanFilter::StateMat caProcessNoise(double dt, double sigma_jerk) {
    const double q = sigma_jerk * sigma_jerk;
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;
    const double dt5 = dt4 * dt;

    const Mat3 I = Mat3::Identity();
    CaKalmanFilter::StateMat Q = CaKalmanFilter::StateMat::Zero();
    Q.block<3, 3>(0, 0) = (dt5 / 20.0 * q) * I;
    Q.block<3, 3>(0, 3) = (dt4 / 8.0 * q) * I;
    Q.block<3, 3>(0, 6) = (dt3 / 6.0 * q) * I;
    Q.block<3, 3>(3, 0) = (dt4 / 8.0 * q) * I;
    Q.block<3, 3>(3, 3) = (dt3 / 3.0 * q) * I;
    Q.block<3, 3>(3, 6) = (dt2 / 2.0 * q) * I;
    Q.block<3, 3>(6, 0) = (dt3 / 6.0 * q) * I;
    Q.block<3, 3>(6, 3) = (dt2 / 2.0 * q) * I;
    Q.block<3, 3>(6, 6) = (dt * q) * I;
    return Q;
}

/// H = [I  0  0]：只观测位置。
inline CaKalmanFilter::MeasStateMat caObservation() {
    CaKalmanFilter::MeasStateMat H = CaKalmanFilter::MeasStateMat::Zero();
    H.leftCols<3>().setIdentity();
    return H;
}

inline CaKalmanFilter::Model makeCaModel(const CaParams& params) {
    CaKalmanFilter::Model model;
    model.F = [](double dt) { return caTransition(dt); };
    model.Q = [params](double dt) { return caProcessNoise(dt, params.sigma_jerk); };
    model.H = [] { return caObservation(); };
    model.R = [params](const CaKalmanFilter::MeasVec&) { return params.R; };
    return model;
}

// ---------------------------------------------------------------------------
// 友好封装（实现 IFilter，可放进 IMM）
// ---------------------------------------------------------------------------

class CaKalman final : public IFilter {
public:
    struct Estimate {
        Vec3 position = Vec3::Zero();
        Vec3 velocity = Vec3::Zero();
        Vec3 acceleration = Vec3::Zero();
    };

    CaKalman() = default;

    explicit CaKalman(const Vec3& position, const Vec3& velocity, const Vec3& acceleration,
                      double init_pos_sd = 0.1, double init_vel_sd = 1.0, double init_acc_sd = 2.0) {
        init(position, velocity, acceleration, init_pos_sd, init_vel_sd, init_acc_sd);
    }

    void init(const Vec3& position, const Vec3& velocity, const Vec3& acceleration,
              double init_pos_sd = 0.1, double init_vel_sd = 1.0, double init_acc_sd = 2.0) {
        CaKalmanFilter::StateVec x0;
        x0.head<3>() = position;
        x0.segment<3>(3) = velocity;
        x0.tail<3>() = acceleration;

        CaKalmanFilter::StateMat P0 = CaKalmanFilter::StateMat::Zero();
        P0.block<3, 3>(0, 0) = Mat3::Identity() * init_pos_sd * init_pos_sd;
        P0.block<3, 3>(3, 3) = Mat3::Identity() * init_vel_sd * init_vel_sd;
        P0.block<3, 3>(6, 6) = Mat3::Identity() * init_acc_sd * init_acc_sd;

        filter_ = CaKalmanFilter(makeCaModel(params_), x0, P0);
    }

    // ---- 噪声参数 ----

    void setJerkNoise(double sigma_jerk) {
        params_.sigma_jerk = sigma_jerk;
        filter_.setModel(makeCaModel(params_));
    }

    void setMeasurementNoise(double sigma) {
        params_.R = Mat3::Identity() * sigma * sigma;
        filter_.setModel(makeCaModel(params_));
    }

    void setMeasurementNoise(const Mat3& R) {
        params_.R = R;
        filter_.setModel(makeCaModel(params_));
    }

    [[nodiscard]] double jerkNoise() const { return params_.sigma_jerk; }
    [[nodiscard]] const Mat3& measurementNoise() const { return params_.R; }

    // ---- 固定尺寸便捷接口 ----

    [[nodiscard]] Estimate estimate() const {
        return Estimate{filter_.stateVec().head<3>(),
                        filter_.stateVec().segment<3>(3),
                        filter_.stateVec().tail<3>()};
    }

    void update(const Vec3& z) { filter_.update(z); }
    void update(const Vec3& z, const Mat3& R) { filter_.update(z, R); }

    // ---- IFilter ----

    [[nodiscard]] int stateDim() const override { return filter_.stateDim(); }
    [[nodiscard]] int measDim() const override { return filter_.measDim(); }

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
    CaParams params_{};
    CaKalmanFilter filter_{};
};

}  // namespace kf
