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

/// 交互式多模型（IMM）。
///
/// 维度不同时按「谁拥有这一维」混合，不把小模型没有的状态补成均值 0、方差 0。
/// 例如 CV(11) 与 CA(14)：共享的前 11 维一起混合；加速度只来自 CA。
/// angle_indices 里的分量按圆周均值混合（整车模型的 yaw 在下标 6）。
///
/// 一个周期：
///   1) 混合：c_j = Σ_i π_{ij} μ_i，μ_{i|j} = π_{ij} μ_i / c_j
///      某一维只在拥有它的模型之间重新归一后做均值和协方差
///   2) 每个模型从自己的混合初值出发 predict
///   3) 每个模型 update，用似然更新权重
///   4) 组合输出，同样只在拥有该维的模型之间归一
class Imm {
public:
    struct Model {
        std::string name;
        std::unique_ptr<IFilter> filter;
    };

    /// angle_indices：按角度混合的状态下标。板模型的角速度不是角度，保持空。
    Imm(std::vector<Model> models, const Eigen::MatrixXd& transition,
        const Eigen::VectorXd& initial_probs, std::vector<int> angle_indices = {});

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
    std::vector<int> dims_;
    std::vector<int> angles_;
    std::vector<Eigen::MatrixXd> emb_;  ///< E_i：公共空间 × 模型 i 维度
    bool mixed_ = false;

    [[nodiscard]] bool angleIndex(int k) const;
    static double wrapTo(double a, double ref);

    /// weight 与模型一一对应，不必预先归一。dim 是输出维度。
    void mixGaussian(const std::vector<double>& weight, int dim,
                     const std::vector<Eigen::VectorXd>& xs, const std::vector<Eigen::MatrixXd>& Ps,
                     Eigen::VectorXd& x0, Eigen::MatrixXd& P0) const;
};

// ---------------------------------------------------------------------------
// 实现
// ---------------------------------------------------------------------------

inline bool Imm::angleIndex(int k) const {
    return std::find(angles_.begin(), angles_.end(), k) != angles_.end();
}

inline double Imm::wrapTo(double a, double ref) {
    return ref + std::atan2(std::sin(a - ref), std::cos(a - ref));
}

inline void Imm::mixGaussian(const std::vector<double>& weight, int dim,
                             const std::vector<Eigen::VectorXd>& xs,
                             const std::vector<Eigen::MatrixXd>& Ps, Eigen::VectorXd& x0,
                             Eigen::MatrixXd& P0) const {
    const int r = modelCount();
    x0 = Eigen::VectorXd::Zero(dim);
    P0 = Eigen::MatrixXd::Zero(dim, dim);

    for (int k = 0; k < dim; ++k) {
        double ws = 0.0;
        double acc = 0.0;
        if (!angleIndex(k)) {
            for (int i = 0; i < r; ++i) {
                if (dims_[static_cast<std::size_t>(i)] <= k) continue;
                ws += weight[static_cast<std::size_t>(i)];
                acc += weight[static_cast<std::size_t>(i)] * xs[static_cast<std::size_t>(i)][k];
            }
            x0[k] = acc / std::max(ws, 1e-300);
            continue;
        }
        double ref = 0.0;
        double best = -1.0;
        for (int i = 0; i < r; ++i) {
            if (dims_[static_cast<std::size_t>(i)] <= k) continue;
            if (weight[static_cast<std::size_t>(i)] > best) {
                best = weight[static_cast<std::size_t>(i)];
                ref = xs[static_cast<std::size_t>(i)][k];
            }
        }
        for (int i = 0; i < r; ++i) {
            if (dims_[static_cast<std::size_t>(i)] <= k) continue;
            ws += weight[static_cast<std::size_t>(i)];
            acc += weight[static_cast<std::size_t>(i)] * wrapTo(xs[static_cast<std::size_t>(i)][k], ref);
        }
        const double mean = acc / std::max(ws, 1e-300);
        x0[k] = std::atan2(std::sin(mean), std::cos(mean));
    }

    for (int a = 0; a < dim; ++a) {
        for (int b = a; b < dim; ++b) {
            const int need = std::max(a, b);
            double ws = 0.0;
            for (int i = 0; i < r; ++i) {
                if (dims_[static_cast<std::size_t>(i)] > need) ws += weight[static_cast<std::size_t>(i)];
            }
            ws = std::max(ws, 1e-300);
            double pab = 0.0;
            for (int i = 0; i < r; ++i) {
                if (dims_[static_cast<std::size_t>(i)] <= need) continue;
                const double wi = weight[static_cast<std::size_t>(i)] / ws;
                double dxi = xs[static_cast<std::size_t>(i)][a] - x0[a];
                double dxj = xs[static_cast<std::size_t>(i)][b] - x0[b];
                if (angleIndex(a)) dxi = wrapTo(xs[static_cast<std::size_t>(i)][a], x0[a]) - x0[a];
                if (angleIndex(b)) dxj = wrapTo(xs[static_cast<std::size_t>(i)][b], x0[b]) - x0[b];
                pab += wi * (Ps[static_cast<std::size_t>(i)](a, b) + dxi * dxj);
            }
            P0(a, b) = pab;
            P0(b, a) = pab;
        }
    }
}

inline Imm::Imm(std::vector<Model> models, const Eigen::MatrixXd& transition,
                const Eigen::VectorXd& initial_probs, std::vector<int> angle_indices)
    : models_(std::move(models)), transition_(transition), mu_(initial_probs),
      angles_(std::move(angle_indices)) {
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

    dims_.resize(static_cast<std::size_t>(r));
    emb_.resize(static_cast<std::size_t>(r));
    for (int i = 0; i < r; ++i) {
        const int d = models_[static_cast<std::size_t>(i)].filter->stateDim();
        dims_[static_cast<std::size_t>(i)] = d;
        Eigen::MatrixXd E = Eigen::MatrixXd::Zero(nx_common_, d);
        E.topLeftCorner(d, d).setIdentity();
        emb_[static_cast<std::size_t>(i)] = std::move(E);
    }
    for (int k : angles_) {
        if (k < 0 || k >= nx_common_) throw std::invalid_argument("Imm: 角度下标越界");
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

    // 2) 先读完各模型状态，再混合。没有的维度不参与，避免补 0 把加速度拉回原点。
    std::vector<Eigen::VectorXd> xs(static_cast<std::size_t>(r));
    std::vector<Eigen::MatrixXd> Ps(static_cast<std::size_t>(r));
    for (int i = 0; i < r; ++i) {
        const auto k = static_cast<std::size_t>(i);
        xs[k] = models_[k].filter->state();
        Ps[k] = models_[k].filter->covariance();
    }

    std::vector<double> weight(static_cast<std::size_t>(r));
    for (int j = 0; j < r; ++j) {
        const double cj = std::max(c_[j], 1e-300);
        for (int i = 0; i < r; ++i) {
            weight[static_cast<std::size_t>(i)] = transition_(i, j) * mu_[i] / cj;
        }
        Eigen::VectorXd x0;
        Eigen::MatrixXd P0;
        mixGaussian(weight, dims_[static_cast<std::size_t>(j)], xs, Ps, x0, P0);
        models_[static_cast<std::size_t>(j)].filter->setState(x0, P0);
        models_[static_cast<std::size_t>(j)].filter->predict(dt);
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

    // 3) 组合。某一维只在拥有它的模型之间归一，yaw 走圆周均值。
    std::vector<Eigen::VectorXd> xs(static_cast<std::size_t>(r));
    std::vector<Eigen::MatrixXd> Ps(static_cast<std::size_t>(r));
    std::vector<double> weight(static_cast<std::size_t>(r));
    for (int j = 0; j < r; ++j) {
        const auto k = static_cast<std::size_t>(j);
        xs[k] = models_[k].filter->state();
        Ps[k] = models_[k].filter->covariance();
        weight[k] = mu_[j];
    }
    mixGaussian(weight, nx_common_, xs, Ps, x_, P_);

    mixed_ = false;
}

}  // namespace kf
