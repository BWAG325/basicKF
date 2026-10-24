#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>

#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>

#include "kf/filter_interface.hpp"

namespace kf {

/// 通用 EKF（数值雅可比）。
/// 模型只提供 f(x, dt)、Q(dt) 和线性观测矩阵 H；F 用中心/前向差分算。
class Ekf final : public IFilter {
public:
    Ekf() = default;

    struct Model {
        int nx = 0;
        int nz = 0;
        std::function<Eigen::VectorXd(const Eigen::VectorXd&, double)> f;
        std::function<Eigen::MatrixXd(double)> Q;
        Eigen::MatrixXd H;  ///< nz × nx，线性观测（Hx 为空时使用）
        std::function<Eigen::MatrixXd(const Eigen::VectorXd&)> Hx;  ///< 可选：状态相关观测矩阵
        std::function<Eigen::VectorXd(const Eigen::VectorXd&)> h;   ///< 可选：非线性观测方程
        std::function<Eigen::MatrixXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> Rz; ///< 可选：R(z,x)
        std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> zdiff; ///< 可选：角度残差
    };

    Ekf(Model model, const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0,
        const Eigen::MatrixXd& R)
        : model_(std::move(model)), x_(x0), P_(P0), R_(R) {
        if (model_.nx <= 0 || model_.nz <= 0) throw std::invalid_argument("Ekf: 维度非法");
        if (x0.size() != model_.nx) throw std::invalid_argument("Ekf: x0 维度不匹配");
        if (P0.rows() != model_.nx || P0.cols() != model_.nx) {
            throw std::invalid_argument("Ekf: P0 维度不匹配");
        }
        if (!model_.Hx && (model_.H.rows() != model_.nz || model_.H.cols() != model_.nx)) {
            throw std::invalid_argument("Ekf: H 维度不匹配");
        }
        nu_ = Eigen::VectorXd::Zero(model_.nz);
        S_ = Eigen::MatrixXd::Identity(model_.nz, model_.nz);
    }

    void predict(double dt) override {
        if (!(dt > 0.0) || model_.nx == 0) return;
        const Eigen::VectorXd f0 = model_.f(x_, dt);

        const int n = model_.nx;
        Eigen::MatrixXd F(n, n);
        for (int j = 0; j < n; ++j) {
            const double eps = 1e-6 * std::max(1.0, std::abs(x_[j]));
            Eigen::VectorXd xp = x_;
            xp[j] += eps;
            F.col(j) = (model_.f(xp, dt) - f0) / eps;
        }

        P_ = F * P_ * F.transpose() + model_.Q(dt);
        x_ = f0;
        has_update_ = false;
    }

    void update(const Eigen::VectorXd& z) override {
        if (model_.nx == 0) return;
        const Eigen::MatrixXd H = model_.Hx ? model_.Hx(x_) : model_.H;
        const Eigen::VectorXd hx = model_.h ? model_.h(x_) : (H * x_);
        const Eigen::MatrixXd R = model_.Rz ? model_.Rz(z, x_) : R_;
        const Eigen::VectorXd nu = model_.zdiff ? model_.zdiff(z, hx) : (z - hx);
        S_ = H * P_ * H.transpose() + R;

        const Eigen::MatrixXd PHt = P_ * H.transpose();
        const Eigen::MatrixXd K = PHt * S_.inverse();

        x_ += K * nu;

        const Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(model_.nx, model_.nx) - K * H;
        P_ = IKH * P_ * IKH.transpose() + K * R * K.transpose();

        nu_ = nu;
        has_update_ = true;
    }

    [[nodiscard]] int stateDim() const override { return model_.nx; }
    [[nodiscard]] int measDim() const override { return model_.nz; }

    [[nodiscard]] Eigen::VectorXd state() const override { return x_; }
    [[nodiscard]] Eigen::MatrixXd covariance() const override { return P_; }
    [[nodiscard]] Eigen::VectorXd innovation() const override { return nu_; }
    [[nodiscard]] Eigen::MatrixXd innovationCov() const override { return S_; }

    [[nodiscard]] double nis() const override {
        if (!has_update_) return 0.0;
        return nu_.dot(S_.inverse() * nu_);
    }

    [[nodiscard]] double logLikelihood() const override {
        if (!has_update_) return -std::numeric_limits<double>::infinity();
        const double logdet = std::log(std::max(1e-300, S_.determinant()));
        const double nz = static_cast<double>(model_.nz);
        return -0.5 * (nz * std::log(2.0 * 3.14159265358979323846) + logdet + nis());
    }

    void setState(const Eigen::VectorXd& x, const Eigen::MatrixXd& P) override {
        x_ = x;
        P_ = P;
    }

    void setMeasurementNoise(const Eigen::Matrix3d& R) { R_ = R; }
    void setMeasurementNoise(const Eigen::MatrixXd& R) { R_ = R; }

    void setModel(Model model) {
        if (model.nx != x_.size()) throw std::invalid_argument("Ekf::setModel: nx 不匹配");
        if (!model.Hx && (model.H.rows() != model.nz || model.H.cols() != model.nx)) {
            throw std::invalid_argument("Ekf::setModel: H 维度不匹配");
        }
        model_ = std::move(model);
        nu_ = Eigen::VectorXd::Zero(model_.nz);
        S_ = Eigen::MatrixXd::Identity(model_.nz, model_.nz);
        has_update_ = false;
    }

private:
    Model model_;
    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd R_;
    Eigen::VectorXd nu_;
    Eigen::MatrixXd S_;
    bool has_update_ = false;
};

}  // namespace kf
