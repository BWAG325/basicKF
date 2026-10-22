#pragma once

#include <Eigen/Core>

namespace kf {

/// 所有滤波器对上层（IMM、日志、调试）暴露的统一接口。
///
/// 用运行时维度（VectorXd / MatrixXd），这样 IMM 可以把不同实现
/// （CV、CA、EKF……）放进同一个容器里混合。
class IFilter {
public:
    virtual ~IFilter() = default;

    [[nodiscard]] virtual int stateDim() const = 0;
    [[nodiscard]] virtual int measDim() const = 0;

    virtual void predict(double dt) = 0;
    virtual void update(const Eigen::VectorXd& z) = 0;

    [[nodiscard]] virtual Eigen::VectorXd state() const = 0;
    [[nodiscard]] virtual Eigen::MatrixXd covariance() const = 0;

    [[nodiscard]] virtual Eigen::VectorXd innovation() const = 0;
    [[nodiscard]] virtual Eigen::MatrixXd innovationCov() const = 0;
    [[nodiscard]] virtual double nis() const = 0;

    /// IMM 的模型似然：log N(ν; 0, S)。
    [[nodiscard]] virtual double logLikelihood() const = 0;

    /// IMM 混合后回写初值。
    virtual void setState(const Eigen::VectorXd& x, const Eigen::MatrixXd& P) = 0;
};

}  // namespace kf
