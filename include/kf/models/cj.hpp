#pragma once

#include "kf/kalman_filter.hpp"

namespace kf {

using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

/// 三维 CJ（匀加加速度 / constant jerk）模型参数。
/// 状态 [p, v, a, j]，j 的导数是白噪声 snap。
struct CjParams {
    double sigma_snap = 200.0;            ///< snap 噪声 std [m/s⁴] → Q
    Mat3 R = Mat3::Identity() * 0.0001;
};

using CjKalmanFilter = KalmanFilter<12, 3>;

// ---------------------------------------------------------------------------
// 模型矩阵
// ---------------------------------------------------------------------------

/// F(dt) = [I  dt·I  dt²/2·I  dt³/6·I]
///         [0    I     dt·I    dt²/2·I]
///         [0    0       I       dt·I  ]
///         [0    0       0         I   ]
inline CjKalmanFilter::StateMat cjTransition(double dt) {
    CjKalmanFilter::StateMat F = CjKalmanFilter::StateMat::Identity();
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    F.block<3, 3>(0, 3) = dt * Mat3::Identity();
    F.block<3, 3>(0, 6) = 0.5 * dt2 * Mat3::Identity();
    F.block<3, 3>(0, 9) = dt3 / 6.0 * Mat3::Identity();
    F.block<3, 3>(3, 6) = dt * Mat3::Identity();
    F.block<3, 3>(3, 9) = 0.5 * dt2 * Mat3::Identity();
    F.block<3, 3>(6, 9) = dt * Mat3::Identity();
    return F;
}

/// Q(dt)：白噪声 snap 积分到 [p, v, a, j]，单轴块：
///   [dt⁷/252  dt⁶/72  dt⁵/30  dt⁴/24]
///   [dt⁶/72   dt⁵/20  dt⁴/8   dt³/6 ]
///   [dt⁵/30   dt⁴/8   dt³/3   dt²/2 ]
///   [dt⁴/24   dt³/6   dt²/2   dt    ] · σs²
inline CjKalmanFilter::StateMat cjProcessNoise(double dt, double sigma_snap) {
    const double q = sigma_snap * sigma_snap;
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double dt4 = dt2 * dt2;
    const double dt5 = dt4 * dt;
    const double dt6 = dt3 * dt3;
    const double dt7 = dt6 * dt;

    const Mat3 I = Mat3::Identity();
    CjKalmanFilter::StateMat Q = CjKalmanFilter::StateMat::Zero();
    const double k[4][4] = {
        {dt7 / 252.0, dt6 / 72.0, dt5 / 30.0, dt4 / 24.0},
        {dt6 / 72.0,  dt5 / 20.0, dt4 / 8.0,  dt3 / 6.0},
        {dt5 / 30.0,  dt4 / 8.0,  dt3 / 3.0,  dt2 / 2.0},
        {dt4 / 24.0,  dt3 / 6.0,  dt2 / 2.0,  dt},
    };
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            Q.block<3, 3>(3 * i, 3 * j) = (q * k[i][j]) * I;
        }
    }
    return Q;
}

inline CjKalmanFilter::MeasStateMat cjObservation() {
    CjKalmanFilter::MeasStateMat H = CjKalmanFilter::MeasStateMat::Zero();
    H.leftCols<3>().setIdentity();
    return H;
}

inline CjKalmanFilter::Model makeCjModel(const CjParams& params) {
    CjKalmanFilter::Model model;
    model.F = [](double dt) { return cjTransition(dt); };
    model.Q = [params](double dt) { return cjProcessNoise(dt, params.sigma_snap); };
    model.H = [] { return cjObservation(); };
    model.R = [params](const CjKalmanFilter::MeasVec&) { return params.R; };
    return model;
}

class CjKalman final : public IFilter {
public:
    struct Estimate {
        Vec3 position = Vec3::Zero();
        Vec3 velocity = Vec3::Zero();
        Vec3 acceleration = Vec3::Zero();
        Vec3 jerk = Vec3::Zero();
    };

    CjKalman() = default;

    explicit CjKalman(const Vec3& position, const Vec3& velocity, const Vec3& acceleration,
                      const Vec3& jerk, double init_pos_sd = 0.1, double init_vel_sd = 1.0,
                      double init_acc_sd = 2.0, double init_jerk_sd = 5.0) {
        init(position, velocity, acceleration, jerk, init_pos_sd, init_vel_sd, init_acc_sd,
             init_jerk_sd);
    }

    void init(const Vec3& position, const Vec3& velocity, const Vec3& acceleration,
              const Vec3& jerk, double init_pos_sd = 0.1, double init_vel_sd = 1.0,
              double init_acc_sd = 2.0, double init_jerk_sd = 5.0) {
        CjKalmanFilter::StateVec x0;
        x0.head<3>() = position;
        x0.segment<3>(3) = velocity;
        x0.segment<3>(6) = acceleration;
        x0.tail<3>() = jerk;

        CjKalmanFilter::StateMat P0 = CjKalmanFilter::StateMat::Zero();
        P0.block<3, 3>(0, 0) = Mat3::Identity() * init_pos_sd * init_pos_sd;
        P0.block<3, 3>(3, 3) = Mat3::Identity() * init_vel_sd * init_vel_sd;
        P0.block<3, 3>(6, 6) = Mat3::Identity() * init_acc_sd * init_acc_sd;
        P0.block<3, 3>(9, 9) = Mat3::Identity() * init_jerk_sd * init_jerk_sd;

        filter_ = CjKalmanFilter(makeCjModel(params_), x0, P0);
    }

    void setSnapNoise(double sigma_snap) {
        params_.sigma_snap = sigma_snap;
        filter_.setModel(makeCjModel(params_));
    }

    void setMeasurementNoise(double sigma) {
        params_.R = Mat3::Identity() * sigma * sigma;
        filter_.setModel(makeCjModel(params_));
    }

    void setMeasurementNoise(const Mat3& R) {
        params_.R = R;
        filter_.setModel(makeCjModel(params_));
    }

    [[nodiscard]] Estimate estimate() const {
        return Estimate{filter_.stateVec().head<3>(),
                        filter_.stateVec().segment<3>(3),
                        filter_.stateVec().segment<3>(6),
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

    void setState(const Eigen::VectorXd& x, const Eigen::MatrixXd& P) override {
        filter_.setState(x, P);
    }

private:
    CjParams params_{};
    CjKalmanFilter filter_{};
};

}  // namespace kf
