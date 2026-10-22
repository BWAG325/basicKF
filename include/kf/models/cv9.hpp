#pragma once

#include "kf/kalman_filter.hpp"

namespace kf {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

/// CV 的 9 维版本：状态 [p, v, a]，其中 a 是**恒为 0 的虚拟加速度**。
///
/// 它的作用只是和 CA（9 维）对齐维度，方便放进同一个 IMM 做均匀混合；
/// 动态上仍然是 CV：
///   a 没有过程噪声、没有观测、初始协方差为 0，所以永远精确保持 0；
///   F 里也不把 a 耦合进 p（与 CA 的 dt²/2 项不同）。
///
/// 注意：均匀混合时 CV 的 a=0 仍会参与加权，会把 CA 的加速度往 0 拉——
/// 这是“补虚拟分量”方案和直接嵌入公共空间在数学上的等价之处。
struct Cv9Params {
    double sigma_accel = 1.0;
    Mat3 R = Mat3::Identity() * 0.0001;
};

using Cv9KalmanFilter = KalmanFilter<9, 3>;

inline Cv9KalmanFilter::StateMat cv9Transition(double dt) {
    Cv9KalmanFilter::StateMat F = Cv9KalmanFilter::StateMat::Identity();
    F.block<3, 3>(0, 3) = dt * Mat3::Identity();
    return F;  // a 块保持 I，且不耦合进 p
}

inline Cv9KalmanFilter::StateMat cv9ProcessNoise(double dt, double sigma_accel) {
    const double q = sigma_accel * sigma_accel;
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;

    Cv9KalmanFilter::StateMat Q = Cv9KalmanFilter::StateMat::Zero();
    Q.block<3, 3>(0, 0) = (0.25 * dt4 * q) * Mat3::Identity();
    Q.block<3, 3>(0, 3) = (0.50 * dt3 * q) * Mat3::Identity();
    Q.block<3, 3>(3, 0) = (0.50 * dt3 * q) * Mat3::Identity();
    Q.block<3, 3>(3, 3) = (dt2 * q) * Mat3::Identity();
    return Q;  // a 块为 0
}

inline Cv9KalmanFilter::MeasStateMat cv9Observation() {
    Cv9KalmanFilter::MeasStateMat H = Cv9KalmanFilter::MeasStateMat::Zero();
    H.leftCols<3>().setIdentity();
    return H;
}

inline Cv9KalmanFilter::Model makeCv9Model(const Cv9Params& params) {
    Cv9KalmanFilter::Model model;
    model.F = [](double dt) { return cv9Transition(dt); };
    model.Q = [params](double dt) { return cv9ProcessNoise(dt, params.sigma_accel); };
    model.H = [] { return cv9Observation(); };
    model.R = [params](const Cv9KalmanFilter::MeasVec&) { return params.R; };
    return model;
}

class Cv9Kalman final : public IFilter {
public:
    struct Estimate {
        Vec3 position = Vec3::Zero();
        Vec3 velocity = Vec3::Zero();
        Vec3 acceleration = Vec3::Zero();  // 恒为 0
    };

    Cv9Kalman() = default;

    explicit Cv9Kalman(const Vec3& position, const Vec3& velocity,
                       double init_pos_sd = 0.1, double init_vel_sd = 1.0) {
        init(position, velocity, init_pos_sd, init_vel_sd);
    }

    void init(const Vec3& position, const Vec3& velocity,
              double init_pos_sd = 0.1, double init_vel_sd = 1.0) {
        Cv9KalmanFilter::StateVec x0 = Cv9KalmanFilter::StateVec::Zero();
        x0.head<3>() = position;
        x0.segment<3>(3) = velocity;

        Cv9KalmanFilter::StateMat P0 = Cv9KalmanFilter::StateMat::Zero();
        P0.block<3, 3>(0, 0) = Mat3::Identity() * init_pos_sd * init_pos_sd;
        P0.block<3, 3>(3, 3) = Mat3::Identity() * init_vel_sd * init_vel_sd;

        filter_ = Cv9KalmanFilter(makeCv9Model(params_), x0, P0);
    }

    void setAccelNoise(double sigma_accel) {
        params_.sigma_accel = sigma_accel;
        filter_.setModel(makeCv9Model(params_));
    }

    void setMeasurementNoise(double sigma) {
        params_.R = Mat3::Identity() * sigma * sigma;
        filter_.setModel(makeCv9Model(params_));
    }

    void setMeasurementNoise(const Mat3& R) {
        params_.R = R;
        filter_.setModel(makeCv9Model(params_));
    }

    [[nodiscard]] Estimate estimate() const {
        return Estimate{filter_.stateVec().head<3>(),
                        filter_.stateVec().segment<3>(3),
                        filter_.stateVec().tail<3>()};
    }

    void update(const Vec3& z) { filter_.update(z); }

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

    /// 虚拟加速度必须恒为 0：IMM 混合会把公共空间的 a 投影进来，这里强制清零，
    /// 并清掉 a 与 p/v 的交叉协方差，保证它永远不参与动态。
    void setState(const Eigen::VectorXd& x, const Eigen::MatrixXd& P) override {
        Eigen::VectorXd x9 = x;
        x9.tail<3>().setZero();

        Eigen::MatrixXd P9 = P;
        P9.block<3, 3>(6, 6).setZero();
        P9.block<3, 3>(6, 0).setZero();
        P9.block<3, 3>(0, 6).setZero();

        filter_.setState(x9, P9);
    }

private:
    Cv9Params params_{};
    Cv9KalmanFilter filter_{};
};

}  // namespace kf
