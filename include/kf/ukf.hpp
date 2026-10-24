#pragma once

#include <Eigen/Core>
#include <Eigen/Cholesky>

#include <cmath>
#include <limits>
#include <utility>

#include "kf/ekf.hpp"

namespace kf {

/// 通用 UKF，状态/观测角度分量用圆均值 + wrap 残差处理。
class UkfKalman final : public IFilter {
public:
    UkfKalman() = default;
    UkfKalman(Ekf::Model model, const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0)
        : model_(std::move(model)), x_(x0), P_(P0) {
        n_ = model_.nx;
        lambda_ = alpha_ * alpha_ * (n_ + kappa_) - n_;
        wm_.resize(2 * n_ + 1);
        wc_.resize(2 * n_ + 1);
        wm_[0] = lambda_ / (n_ + lambda_);
        wc_[0] = wm_[0] + (1 - alpha_ * alpha_ + beta_);
        for (int i = 1; i <= 2 * n_; ++i) {
            wm_[i] = 1.0 / (2.0 * (n_ + lambda_));
            wc_[i] = wm_[i];
        }
    }

    static double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

    void predict(double dt) override {
        if (!(dt > 0)) return;
        auto X = sigmaPoints();
        for (auto& s : X) s = model_.f(s, dt);
        const Eigen::VectorXd xm = weightedMean(X);
        Eigen::MatrixXd P = model_.Q(dt);
        for (int i = 0; i <= 2 * n_; ++i) {
            Eigen::VectorXd dx = X[i] - xm;
            dx[6] = wrap(dx[6]);
            P += wc_[i] * dx * dx.transpose();
        }
        x_ = xm;
        P_ = 0.5 * (P + P.transpose());
    }

    void update(const Eigen::VectorXd& z) override {
        auto X = sigmaPoints();
        std::vector<Eigen::VectorXd> Z;
        Z.reserve(X.size());
        for (auto& s : X) Z.push_back(model_.h(s));

        Eigen::VectorXd zm = Eigen::VectorXd::Zero(model_.nz);
        for (int i = 0; i <= 2 * n_; ++i) zm += wm_[i] * Z[i];
        zm[0] = wrap(zm[0]);
        zm[3] = wrap(zm[3]);

        Eigen::MatrixXd S = model_.Rz ? model_.Rz(z, x_) : Eigen::MatrixXd::Identity(model_.nz, model_.nz);
        Eigen::MatrixXd Pxz = Eigen::MatrixXd::Zero(n_, model_.nz);
        for (int i = 0; i <= 2 * n_; ++i) {
            Eigen::VectorXd dz = Z[i] - zm;
            dz[0] = wrap(dz[0]);
            dz[3] = wrap(dz[3]);
            Eigen::VectorXd dx = X[i] - x_;
            dx[6] = wrap(dx[6]);
            S += wc_[i] * dz * dz.transpose();
            Pxz += wc_[i] * dx * dz.transpose();
        }

        const Eigen::MatrixXd K = Pxz * S.inverse();
        Eigen::VectorXd nu = z - zm;
        nu[0] = wrap(nu[0]);
        nu[3] = wrap(nu[3]);
        x_ += K * nu;
        x_[6] = wrap(x_[6]);
        P_ -= K * S * K.transpose();
        P_ = 0.5 * (P_ + P_.transpose());
        nu_ = nu;
        S_ = S;
        has_update_ = true;
    }

    [[nodiscard]] Eigen::Vector3d position() const override {
        const Eigen::VectorXd& x = x_;
        const double ang = x[6] + armor_id_ * 2.0 * M_PI / armor_num_;
        const bool lh = (armor_num_ == 4) && (armor_id_ == 1 || armor_id_ == 3);
        const double r = x[8] + (lh ? x[9] : 0.0);
        return Eigen::Vector3d(x[0] - r * std::cos(ang), x[2] - r * std::sin(ang),
                               x[4] + (lh ? x[10] : 0.0));
    }

    void setArmor(int id, int num) { armor_id_ = id; armor_num_ = num; }

    [[nodiscard]] int stateDim() const override { return n_; }
    [[nodiscard]] int measDim() const override { return model_.nz; }
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
        const double det = S_.determinant();
        return -0.5 * (model_.nz * std::log(2 * M_PI) + std::log(std::max(det, 1e-300)) + nis());
    }
    void setState(const Eigen::VectorXd& x, const Eigen::MatrixXd& P) override {
        x_ = x;
        x_[6] = wrap(x_[6]);
        P_ = P;
    }

private:
    std::vector<Eigen::VectorXd> sigmaPoints() const {
        Eigen::MatrixXd A = (n_ + lambda_) * P_;
        A.diagonal().array() += 1e-9;
        Eigen::LLT<Eigen::MatrixXd> llt(A);
        const Eigen::MatrixXd L = llt.matrixL();
        std::vector<Eigen::VectorXd> X(2 * n_ + 1);
        X[0] = x_;
        for (int i = 0; i < n_; ++i) {
            X[1 + i] = x_ + L.col(i);
            X[1 + n_ + i] = x_ - L.col(i);
        }
        return X;
    }

    Eigen::VectorXd weightedMean(const std::vector<Eigen::VectorXd>& X) const {
        Eigen::VectorXd m = Eigen::VectorXd::Zero(n_);
        for (int i = 0; i <= 2 * n_; ++i) m += wm_[i] * X[i];
        double s = 0, c = 0;
        for (int i = 0; i <= 2 * n_; ++i) {
            s += wm_[i] * std::sin(X[i][6]);
            c += wm_[i] * std::cos(X[i][6]);
        }
        m[6] = std::atan2(s, c);
        return m;
    }

    Ekf::Model model_;
    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;
    Eigen::VectorXd nu_ = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd S_ = Eigen::MatrixXd::Identity(4, 4);
    Eigen::VectorXd wm_, wc_;
    int n_ = 11;
    double alpha_ = 1.0, beta_ = 2.0, kappa_ = 0.0, lambda_ = 0.0;
    int armor_id_ = 0, armor_num_ = 4;
    bool has_update_ = false;
};

}  // namespace kf
