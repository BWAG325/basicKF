#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "kf/filter_interface.hpp"

namespace kf {

/// 交互式多模型（IMM），支持**不同状态维度**的模型（例如 6 维 CV + 9 维 CA）。
///
/// 维度不同时，把每个模型的状态嵌入到公共空间（取各模型维度的最大值）：
///   E_i : 公共空间 ← 模型 i（对角单位块，其余补 0）
/// 混合和组合都在公共空间做，再投影回各模型自己的维度。
///
/// 一个周期：
///   1) 混合：由上一拍的模型概率 μ 和转移矩阵 Π 算混合初值
///        c_j     = Σ_i p_ij μ_i
///        μ_{i|j} = p_ij μ_i / c_j
///        x⁰_j    = Σ_i μ_{i|j} E_i x_i
///        P⁰_j    = Σ_i μ_{i|j} [E_i P_i E_iᵀ + (E_i x_i−x⁰_j)(…)ᵀ]
///   2) 每个模型从 E_jᵀ x⁰_j 出发 predict
///   3) 每个模型 update，用似然更新权重
///   4) 组合输出（公共空间）
///        x = Σ_j μ_j E_j x_j
///        P = Σ_j μ_j [E_j P_j E_jᵀ + (E_j x_j−x)(…)ᵀ]
class Imm {
public:
    struct Model {
        std::string name;
        std::unique_ptr<IFilter> filter;
    };

    Imm(std::vector<Model> models, const Eigen::MatrixXd& transition,
        const Eigen::VectorXd& initial_probs);

    /// 混合 + 每个模型预测。
    void predict(double dt);

    /// 每个模型更新 + 权重更新 + 组合输出。
    void update(const Eigen::VectorXd& z);

    /// 公共空间下的组合状态/协方差（同维模型时就是模型自己的维度）。
    [[nodiscard]] const Eigen::VectorXd& state() const noexcept { return x_; }
    [[nodiscard]] const Eigen::MatrixXd& covariance() const noexcept { return P_; }
    [[nodiscard]] const Eigen::VectorXd& probabilities() const noexcept { return mu_; }

    [[nodiscard]] int modelCount() const noexcept { return static_cast<int>(models_.size()); }
    [[nodiscard]] int commonDim() const noexcept { return nx_common_; }
    [[nodiscard]] const Model& model(int i) const { return models_.at(static_cast<std::size_t>(i)); }

    /// 模型自己的状态（未嵌入公共空间）。
    [[nodiscard]] Eigen::VectorXd modelState(int i) const {
        return models_.at(static_cast<std::size_t>(i)).filter->state();
    }

    /// 模型状态投影到公共空间。
    [[nodiscard]] Eigen::VectorXd modelStateCommon(int i) const {
        const auto j = static_cast<std::size_t>(i);
        return emb_[j] * models_[j].filter->state();
    }

private:
    std::vector<Model> models_;
    Eigen::MatrixXd transition_;
    Eigen::VectorXd mu_;
    Eigen::VectorXd c_;

    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;
    int nx_common_ = 0;
    std::vector<Eigen::MatrixXd> emb_;  ///< E_i：公共空间 × 模型 i 维度
    bool mixed_ = false;
};

// ---------------------------------------------------------------------------
// 实现
// ---------------------------------------------------------------------------

inline Imm::Imm(std::vector<Model> models, const Eigen::MatrixXd& transition,
                const Eigen::VectorXd& initial_probs)
    : models_(std::move(models)), transition_(transition), mu_(initial_probs) {
    if (models_.empty()) throw std::invalid_argument("Imm: 至少需要一个模型");

    const int r = modelCount();
    if (transition_.rows() != r || transition_.cols() != r) {
        throw std::invalid_argument("Imm: 转移矩阵维度与模型数量不一致");
    }
    if (mu_.size() != r) throw std::invalid_argument("Imm: 初始概率维度与模型数量不一致");
    if (!(mu_.sum() > 0.0)) throw std::invalid_argument("Imm: 初始概率之和必须 > 0");
    mu_ /= mu_.sum();

    const int meas_dim = models_.front().filter->measDim();
    nx_common_ = 0;
    for (const auto& m : models_) {
        if (m.filter->measDim() != meas_dim) throw std::invalid_argument("Imm: 各模型观测维度必须一致");
        nx_common_ = std::max(nx_common_, m.filter->stateDim());
    }

    emb_.resize(static_cast<std::size_t>(r));
    for (int i = 0; i < r; ++i) {
        const int d = models_[static_cast<std::size_t>(i)].filter->stateDim();
        Eigen::MatrixXd E = Eigen::MatrixXd::Zero(nx_common_, d);
        E.topLeftCorner(d, d).setIdentity();
        emb_[static_cast<std::size_t>(i)] = std::move(E);
    }

    c_ = Eigen::VectorXd::Ones(r);
    const auto& f0 = models_.front().filter;
    x_ = emb_.front() * f0->state();
    P_ = emb_.front() * f0->covariance() * emb_.front().transpose();
}

inline void Imm::predict(double dt) {
    const int r = modelCount();

    // 1) 混合概率 c_j
    c_.setZero(r);
    for (int j = 0; j < r; ++j) {
        for (int i = 0; i < r; ++i) c_[j] += transition_(i, j) * mu_[i];
    }

    // 2) 收齐原始状态并嵌入公共空间（先全部读完，避免边算边写造成互相污染）
    std::vector<Eigen::VectorXd> xs(static_cast<std::size_t>(r));
    std::vector<Eigen::MatrixXd> Ps(static_cast<std::size_t>(r));
    std::vector<Eigen::VectorXd> xc(static_cast<std::size_t>(r));
    std::vector<Eigen::MatrixXd> Pc(static_cast<std::size_t>(r));
    for (int i = 0; i < r; ++i) {
        const auto k = static_cast<std::size_t>(i);
        xs[k] = models_[k].filter->state();
        Ps[k] = models_[k].filter->covariance();
        xc[k] = emb_[k] * xs[k];
        Pc[k] = emb_[k] * Ps[k] * emb_[k].transpose();
    }

    // 3) 在公共空间算混合初值
    std::vector<Eigen::VectorXd> mixed_x(static_cast<std::size_t>(r));
    std::vector<Eigen::MatrixXd> mixed_P(static_cast<std::size_t>(r));
    for (int j = 0; j < r; ++j) {
        const double cj = std::max(c_[j], 1e-300);

        Eigen::VectorXd x0 = Eigen::VectorXd::Zero(nx_common_);
        for (int i = 0; i < r; ++i) {
            x0 += (transition_(i, j) * mu_[i] / cj) * xc[static_cast<std::size_t>(i)];
        }

        Eigen::MatrixXd P0 = Eigen::MatrixXd::Zero(nx_common_, nx_common_);
        for (int i = 0; i < r; ++i) {
            const double w = transition_(i, j) * mu_[i] / cj;
            const Eigen::VectorXd dx = xc[static_cast<std::size_t>(i)] - x0;
            P0 += w * (Pc[static_cast<std::size_t>(i)] + dx * dx.transpose());
        }

        mixed_x[static_cast<std::size_t>(j)] = std::move(x0);
        mixed_P[static_cast<std::size_t>(j)] = std::move(P0);
    }

    // 4) 投影回各模型维度，再预测
    for (int j = 0; j < r; ++j) {
        const auto k = static_cast<std::size_t>(j);
        const Eigen::VectorXd x0 = emb_[k].transpose() * mixed_x[k];
        const Eigen::MatrixXd P0 = emb_[k].transpose() * mixed_P[k] * emb_[k];
        models_[k].filter->setState(x0, P0);
        models_[k].filter->predict(dt);
    }

    mixed_ = true;
}

inline void Imm::update(const Eigen::VectorXd& z) {
    const int r = modelCount();
    if (!mixed_) c_.setOnes(r);  // 没 predict 也允许更新

    // 1) 各模型更新
    for (auto& m : models_) m.filter->update(z);

    // 2) 权重：log μ_j ∝ log c_j + logLikelihood_j
    Eigen::VectorXd logw(r);
    for (int j = 0; j < r; ++j) {
        const double ll = models_[static_cast<std::size_t>(j)].filter->logLikelihood();
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

    // 3) 公共空间组合
    x_.setZero(nx_common_);
    for (int j = 0; j < r; ++j) {
        const auto k = static_cast<std::size_t>(j);
        x_ += mu_[j] * (emb_[k] * models_[k].filter->state());
    }

    P_.setZero(nx_common_, nx_common_);
    for (int j = 0; j < r; ++j) {
        const auto k = static_cast<std::size_t>(j);
        const Eigen::VectorXd xj = emb_[k] * models_[k].filter->state();
        const Eigen::MatrixXd Pj = emb_[k] * models_[k].filter->covariance() * emb_[k].transpose();
        const Eigen::VectorXd dx = xj - x_;
        P_ += mu_[j] * (Pj + dx * dx.transpose());
    }

    mixed_ = false;
}

}  // namespace kf
