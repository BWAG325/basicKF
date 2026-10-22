#pragma once

#include <Eigen/Cholesky>
#include <Eigen/Core>

#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>

#include "kf/filter_interface.hpp"

namespace kf {

/// 通用线性卡尔曼滤波：只做矩阵运算，不认识任何具体运动模型。
///
/// 思路和旧版 `include/kalman_filter.hpp` 一致：模型通过 `Model` 注入
/// 四个矩阵，核心只负责 predict / update。
///
///   F(dt)   状态转移矩阵
///   Q(dt)   过程噪声协方差
///   H       观测矩阵
///   R(z)    观测噪声协方差（可以依赖观测，比如随距离变大）
///
/// 状态 / 观测都是编译期固定维度的 Eigen 类型，核心内部没有动态分配；
/// 对 IMM 则通过 IFilter 的 VectorXd / MatrixXd 接口转成运行时维度。
template <int NX, int NZ>
class KalmanFilter final : public IFilter {
public:
    using StateVec = Eigen::Matrix<double, NX, 1>;
    using StateMat = Eigen::Matrix<double, NX, NX>;
    using MeasVec = Eigen::Matrix<double, NZ, 1>;
    using MeasMat = Eigen::Matrix<double, NZ, NZ>;
    using MeasStateMat = Eigen::Matrix<double, NZ, NX>;

    struct Model {
        std::function<StateMat(double dt)> F;
        std::function<StateMat(double dt)> Q;
        std::function<MeasStateMat()> H;
        std::function<MeasMat(const MeasVec& z)> R;
    };

    KalmanFilter() = default;

    KalmanFilter(Model model, const StateVec& x0, const StateMat& P0)
        : model_(std::move(model)), x_(x0), P_(P0) {}

    // ---- 固定尺寸便捷接口（普通调用方用这个）----

    [[nodiscard]] const StateVec& stateVec() const noexcept { return x_; }
    [[nodiscard]] const StateMat& stateCov() const noexcept { return P_; }
    [[nodiscard]] const MeasVec& innovationVec() const noexcept { return nu_; }
    [[nodiscard]] const MeasMat& innovationCovMat() const noexcept { return S_; }

    /// 换模型（例如调了噪声参数后重建 F/Q/H/R）。
    void setModel(Model model) { model_ = std::move(model); }

    void reset(const StateVec& x0, const StateMat& P0) {
        x_ = x0;
        P_ = P0;
        nu_.setZero();
        S_.setIdentity();
        has_update_ = false;
    }

    // ---- IFilter：运行时维度 ----

    [[nodiscard]] int stateDim() const override { return NX; }
    [[nodiscard]] int measDim() const override { return NZ; }

    void predict(double dt) override {
        if (!(dt > 0.0) || !model_.F) return;

        const StateMat F = model_.F(dt);
        x_ = F * x_;
        P_ = F * P_ * F.transpose() + model_.Q(dt);
    }

    void update(const Eigen::VectorXd& z) override {
        if (z.size() != NZ) throw std::invalid_argument("KalmanFilter::update: 观测维度不匹配");
        updateFixed(z);
    }

    void update(const MeasVec& z) { updateFixed(z); }

    /// 用外部给的 R 更新（覆盖模型里的 R）。
    void update(const MeasVec& z, const MeasMat& R) { updateFixed(z, R); }

    [[nodiscard]] Eigen::VectorXd state() const override { return x_; }
    [[nodiscard]] Eigen::MatrixXd covariance() const override { return P_; }
    [[nodiscard]] Eigen::VectorXd innovation() const override { return nu_; }
    [[nodiscard]] Eigen::MatrixXd innovationCov() const override { return S_; }

    [[nodiscard]] double nis() const override {
        if (!has_update_) return 0.0;
        return nu_.dot(S_.ldlt().solve(nu_));
    }

    [[nodiscard]] double logLikelihood() const override {
        if (!has_update_) return -std::numeric_limits<double>::infinity();

        const Eigen::LDLT<MeasMat> ldlt(S_);
        if (ldlt.info() != Eigen::Success) return -std::numeric_limits<double>::infinity();

        const double nis_value = nu_.dot(ldlt.solve(nu_));
        const double log_det = ldlt.vectorD().array().log().sum();
        return -0.5 * (nis_value + log_det + NZ * std::log(2.0 * kPi));
    }

    void setState(const Eigen::VectorXd& x, const Eigen::MatrixXd& P) override {
        if (x.size() != NX || P.rows() != NX || P.cols() != NX) {
            throw std::invalid_argument("KalmanFilter::setState: 维度不匹配");
        }
        x_ = x;
        P_ = P;
    }

private:
    static constexpr double kPi = 3.14159265358979323846;

    template <class Z>
    void updateFixed(const Z& z) {
        const MeasStateMat H = model_.H();
        const MeasVec residual = z - H * x_;
        const MeasMat S = H * P_ * H.transpose() + model_.R(z);

        const auto K = P_ * H.transpose() * S.ldlt().solve(MeasMat::Identity());

        x_ += K * residual;

        // Joseph 形式：保持对称正定
        const StateMat IKH = StateMat::Identity() - K * H;
        P_ = IKH * P_ * IKH.transpose() + K * model_.R(z) * K.transpose();
        P_ = 0.5 * (P_ + P_.transpose());

        nu_ = residual;
        S_ = S;
        has_update_ = true;
    }

    void updateFixed(const MeasVec& z, const MeasMat& R) {
        const MeasStateMat H = model_.H();
        const MeasVec residual = z - H * x_;
        const MeasMat S = H * P_ * H.transpose() + R;

        const auto K = P_ * H.transpose() * S.ldlt().solve(MeasMat::Identity());

        x_ += K * residual;

        const StateMat IKH = StateMat::Identity() - K * H;
        P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();
        P_ = 0.5 * (P_ + P_.transpose());

        nu_ = residual;
        S_ = S;
        has_update_ = true;
    }

    Model model_{};
    StateVec x_ = StateVec::Zero();
    StateMat P_ = StateMat::Identity();

    MeasVec nu_ = MeasVec::Zero();
    MeasMat S_ = MeasMat::Identity();
    bool has_update_ = false;
};

}  // namespace kf
