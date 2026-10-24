#pragma once

#include <Eigen/Core>

#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>

#include "kf/models/cv.hpp"

namespace kf {

/// 11 维整车 EKF 的“真 IMM”：
/// - 两个 CV 模型（不同 Q）
/// - 预测前做状态混合（角度对齐 + 圆均值）
/// - 更新后按似然更新模型概率
class CvImm final : public IFilter {
public:
    CvImm(const CvParams& a, const CvParams& b, const Eigen::Matrix2d& Pi,
          const Eigen::Vector2d& mu0)
        : Pi_(Pi), mu_(mu0) {
        models_[0] = CvKalman(a);
        models_[1] = CvKalman(b);
        params_ = a;
        c_ = mu_;
    }

    void init(const Vec3& plate, const Vec3& velocity) {
        models_[0].init(plate, velocity);
        models_[1].init(plate, velocity);
        mu_ = Eigen::Vector2d(0.5, 0.5);
        c_ = mu_;
    }

    void setMeasurementNoise(const Mat3& R) {
        models_[0].setMeasurementNoise(R);
        models_[1].setMeasurementNoise(R);
    }

    void predict(double dt) override {
        mix();
        models_[0].predict(dt);
        models_[1].predict(dt);
    }

    void update(const Eigen::VectorXd& z) override {
        models_[0].update(z);
        models_[1].update(z);

        const double l0 = models_[0].logLikelihood();
        const double l1 = models_[1].logLikelihood();
        const double a = std::log(std::max(c_[0], 1e-300)) + l0;
        const double b = std::log(std::max(c_[1], 1e-300)) + l1;
        const double m = std::max(a, b);
        if (!std::isfinite(m)) {
            mu_ = Eigen::Vector2d(0.5, 0.5);
        } else {
            const double w0 = std::exp(a - m), w1 = std::exp(b - m);
            mu_ = Eigen::Vector2d(w0 / (w0 + w1), w1 / (w0 + w1));
        }
    }

    [[nodiscard]] Vec3 position() const override {
        const Eigen::VectorXd x = combinedState();
        return plateFromState(x);
    }

    [[nodiscard]] Eigen::VectorXd state() const override { return combinedState(); }

    [[nodiscard]] Eigen::MatrixXd covariance() const override {
        return mu_[0] * models_[0].covariance() + mu_[1] * models_[1].covariance();
    }

    [[nodiscard]] Eigen::VectorXd innovation() const override { return models_[0].innovation(); }
    [[nodiscard]] Eigen::MatrixXd innovationCov() const override { return models_[0].innovationCov(); }
    [[nodiscard]] double nis() const override { return models_[0].nis(); }
    [[nodiscard]] double logLikelihood() const override { return models_[0].logLikelihood(); }
    [[nodiscard]] int stateDim() const override { return 11; }
    [[nodiscard]] int measDim() const override { return 3; }

    void setState(const Eigen::VectorXd& x, const Eigen::MatrixXd& P) override {
        models_[0].setState(x, P);
        models_[1].setState(x, P);
    }

    [[nodiscard]] const Eigen::Vector2d& probabilities() const { return mu_; }

private:
    static double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

    static double alignTo(double a, double ref) { return ref + wrap(a - ref); }

    void mix() {
        c_.setZero();
        for (int i = 0; i < 2; ++i)
            for (int j = 0; j < 2; ++j) c_[j] += Pi_(i, j) * mu_[i];

        std::array<Eigen::VectorXd, 2> xs{models_[0].state(), models_[1].state()};
        std::array<Eigen::MatrixXd, 2> Ps{models_[0].covariance(), models_[1].covariance()};

        // 以模型 0 的 yaw 为参考，把 yaw 对齐（去掉 2π 跳变）
        const double ref = xs[0][6];
        xs[1][6] = alignTo(xs[1][6], ref);

        for (int j = 0; j < 2; ++j) {
            Eigen::VectorXd x0 = Eigen::VectorXd::Zero(11);
            Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(11, 11);
            for (int i = 0; i < 2; ++i) {
                const double w = Pi_(i, j) * mu_[i] / std::max(c_[j], 1e-300);
                x0 += w * xs[i];
                const Eigen::VectorXd dx = xs[i] - x0;
                P0 += w * (Ps[i] + dx * dx.transpose());
            }
            x0[6] = wrap(x0[6]);
            models_[j].setState(x0, P0);
        }
    }

    Eigen::VectorXd combinedState() const {
        const Eigen::VectorXd x0 = models_[0].state();
        const Eigen::VectorXd x1 = models_[1].state();
        Eigen::VectorXd x = mu_[0] * x0 + mu_[1] * x1;
        const double y0 = x0[6];
        const double y1 = alignTo(x1[6], y0);
        x[6] = wrap(mu_[0] * y0 + mu_[1] * y1);
        return x;
    }

    Vec3 plateFromState(const Eigen::VectorXd& x) const {
        const double ang = x[6] + params_.armor_id * 2.0 * 3.14159265358979323846 / params_.armor_num;
        const bool lh = (params_.armor_num == 4) && (params_.armor_id == 1 || params_.armor_id == 3);
        const double r = x[8] + (lh ? x[9] : 0.0);
        return Vec3(x[0] - r * std::cos(ang), x[2] - r * std::sin(ang), x[4] + (lh ? x[10] : 0.0));
    }

    std::array<CvKalman, 2> models_;
    Eigen::Matrix2d Pi_;
    Eigen::Vector2d mu_;
    Eigen::Vector2d c_;
    CvParams params_{};
};

}  // namespace kf
