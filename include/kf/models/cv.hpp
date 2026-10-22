#pragma once

#include <utility>

#include "kf/kalman_filter.hpp"

namespace kf {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

/// 三维 CV（匀速）模型参数。
struct CvParams {
    double sigma_accel = 1.0;             ///< 加速度噪声 std [m/s²] → Q
    Mat3 R = Mat3::Identity() * 0.0001;   ///< 观测噪声协方差 [m²]
};

/// CV 滤波器的固定尺寸类型：状态 [p, v]（6 维），观测 [p]（3 维）。
using CvKalmanFilter = KalmanFilter<6, 3>;

// ---------------------------------------------------------------------------
// 模型矩阵
// ---------------------------------------------------------------------------

/// F(dt) = [I  dt·I]
///         [0    I ]
inline CvKalmanFilter::StateMat cvTransition(double dt) {
    CvKalmanFilter::StateMat F = CvKalmanFilter::StateMat::Identity();
    F.block<3, 3>(0, 3) = dt * Mat3::Identity();
    return F;
}

/// Q(dt)：离散白噪声加速度模型，位置/速度块由同一个 sigma_accel 生成。
inline CvKalmanFilter::StateMat cvProcessNoise(double dt, double sigma_accel) {
    const double q = sigma_accel * sigma_accel;
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;

    CvKalmanFilter::StateMat Q = CvKalmanFilter::StateMat::Zero();
    Q.block<3, 3>(0, 0) = (0.25 * dt4 * q) * Mat3::Identity();
    Q.block<3, 3>(0, 3) = (0.50 * dt3 * q) * Mat3::Identity();
    Q.block<3, 3>(3, 0) = (0.50 * dt3 * q) * Mat3::Identity();
    Q.block<3, 3>(3, 3) = (dt2 * q) * Mat3::Identity();
    return Q;
}

/// H = [I  0]：只观测位置。
inline CvKalmanFilter::MeasStateMat cvObservation() {
    CvKalmanFilter::MeasStateMat H = CvKalmanFilter::MeasStateMat::Zero();
    H.leftCols<3>().setIdentity();
    return H;
}

inline CvKalmanFilter::Model makeCvModel(const CvParams& params) {
    CvKalmanFilter::Model model;
    model.F = [](double dt) { return cvTransition(dt); };
    model.Q = [params](double dt) { return cvProcessNoise(dt, params.sigma_accel); };
    model.H = [] { return cvObservation(); };
    model.R = [params](const CvKalmanFilter::MeasVec&) { return params.R; };
    return model;
}

// ---------------------------------------------------------------------------
// 友好封装：给普通调用方（也实现 IFilter，可直接放进 IMM）
// ---------------------------------------------------------------------------

class CvKalman final : public IFilter {
public:
    struct Estimate {
        Vec3 position = Vec3::Zero();
        Vec3 velocity = Vec3::Zero();
    };

    CvKalman() = default;

    /// position / velocity：初始状态
    /// init_pos_sd：初始位置不确定度 [m]（进 P₀）
    /// init_vel_sd：初始速度不确定度 [m/s]（进 P₀）
    explicit CvKalman(const Vec3& position, const Vec3& velocity,
                      double init_pos_sd = 0.1, double init_vel_sd = 1.0) {
        init(position, velocity, init_pos_sd, init_vel_sd);
    }

    void init(const Vec3& position, const Vec3& velocity,
              double init_pos_sd = 0.1, double init_vel_sd = 1.0) {
        CvKalmanFilter::StateVec x0;
        x0.head<3>() = position;
        x0.tail<3>() = velocity;

        CvKalmanFilter::StateMat P0 = CvKalmanFilter::StateMat::Zero();
        P0.block<3, 3>(0, 0) = Mat3::Identity() * init_pos_sd * init_pos_sd;
        P0.block<3, 3>(3, 3) = Mat3::Identity() * init_vel_sd * init_vel_sd;

        filter_ = CvKalmanFilter(makeCvModel(params_), x0, P0);
    }

    // ---- 噪声参数（改完会重建模型矩阵）----

    void setAccelNoise(double sigma_accel) {
        params_.sigma_accel = sigma_accel;
        filter_.setModel(makeCvModel(params_));
    }

    void setMeasurementNoise(double sigma) {
        params_.R = Mat3::Identity() * sigma * sigma;
        filter_.setModel(makeCvModel(params_));
    }

    void setMeasurementNoise(const Mat3& R) {
        params_.R = R;
        filter_.setModel(makeCvModel(params_));
    }

    [[nodiscard]] double accelNoise() const { return params_.sigma_accel; }
    [[nodiscard]] const Mat3& measurementNoise() const { return params_.R; }

    // ---- 固定尺寸便捷接口 ----

    [[nodiscard]] Estimate estimate() const {
        return Estimate{filter_.stateVec().head<3>(), filter_.stateVec().tail<3>()};
    }

    [[nodiscard]] const CvKalmanFilter& filter() const { return filter_; }
    [[nodiscard]] CvKalmanFilter& filter() { return filter_; }

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
    CvParams params_{};
    CvKalmanFilter filter_{};
};

}  // namespace kf
