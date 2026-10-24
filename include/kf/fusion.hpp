#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include "kf/filter_interface.hpp"

namespace kf {

/// 输出层融合（方案 A）：各模型独立预测/更新，只按似然加权它们的输出位置。
/// 不做状态混合，因此绕开了“中心/相位”不可辨的问题。
class OutputFusion final : public IFilter {
public:
    OutputFusion(std::vector<std::unique_ptr<IFilter>> models, const Eigen::MatrixXd& transition,
                 const Eigen::VectorXd& initial_probs)
        : models_(std::move(models)), transition_(transition), mu_(initial_probs) {
        const int r = static_cast<int>(models_.size());
        if (r < 2) throw std::invalid_argument("OutputFusion: 至少两个模型");
        if (transition_.rows() != r || transition_.cols() != r) throw std::invalid_argument("Pi 维度不对");
        if (mu_.size() != r) throw std::invalid_argument("mu 维度不对");
        mu_ /= mu_.sum();
        c_ = Eigen::VectorXd::Ones(r);
    }

    void predict(double dt) override {
        for (auto& m : models_) m->predict(dt);
    }

    void update(const Eigen::VectorXd& z) override {
        const int r = static_cast<int>(models_.size());
        for (auto& m : models_) m->update(z);

        // 先按转移矩阵得到先验模型概率 c_j
        c_.setZero(r);
        for (int i = 0; i < r; ++i)
            for (int j = 0; j < r; ++j) c_[j] += transition_(i, j) * mu_[i];

        Eigen::VectorXd logw(r);
        for (int j = 0; j < r; ++j) {
            const double ll = models_[static_cast<std::size_t>(j)]->logLikelihood();
            logw[j] = std::isfinite(ll) ? std::log(std::max(c_[j], 1e-300)) + ll
                                        : -std::numeric_limits<double>::infinity();
        }
        const double mx = logw.maxCoeff();
        if (!std::isfinite(mx)) {
            mu_.setConstant(1.0 / r);
        } else {
            const Eigen::VectorXd w = (logw.array() - mx).exp();
            mu_ = w / w.sum();
        }
    }

    [[nodiscard]] Eigen::Vector3d position() const override {
        Eigen::Vector3d p = Eigen::Vector3d::Zero();
        for (std::size_t j = 0; j < models_.size(); ++j) p += mu_[j] * models_[j]->position();
        return p;
    }

    [[nodiscard]] Eigen::VectorXd state() const override {
        Eigen::VectorXd x = Eigen::VectorXd::Zero(models_.front()->stateDim());
        for (std::size_t j = 0; j < models_.size(); ++j) x += mu_[j] * models_[j]->state();
        return x;
    }

    [[nodiscard]] Eigen::MatrixXd covariance() const override {
        return models_.front()->covariance();
    }

    [[nodiscard]] Eigen::VectorXd innovation() const override { return models_.front()->innovation(); }
    [[nodiscard]] Eigen::MatrixXd innovationCov() const override { return models_.front()->innovationCov(); }
    [[nodiscard]] double nis() const override { return models_.front()->nis(); }
    [[nodiscard]] double logLikelihood() const override { return models_.front()->logLikelihood(); }

    [[nodiscard]] int stateDim() const override { return models_.front()->stateDim(); }
    [[nodiscard]] int measDim() const override { return models_.front()->measDim(); }

    void setState(const Eigen::VectorXd&, const Eigen::MatrixXd&) override {}
    [[nodiscard]] const Eigen::VectorXd& probabilities() const { return mu_; }

private:
    std::vector<std::unique_ptr<IFilter>> models_;
    Eigen::MatrixXd transition_;
    Eigen::VectorXd mu_;
    Eigen::VectorXd c_;
};

}  // namespace kf
