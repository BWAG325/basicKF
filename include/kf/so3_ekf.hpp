#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <limits>

#include "kf/filter_interface.hpp"

namespace kf {

/// 误差状态 EKF（SO(3) 右乘扰动），参考 awakening 的整车估计：
/// 名义状态：c, v, R, wyaw, log_r1, log_r2, h
/// 误差状态：dc, dv, dtheta(3), dw, dlr1, dlr2, dh（13 维）
class So3Ekf final : public IFilter {
public:
    struct Params {
        double q_xyz = 100.0;
        double q_yaw = 75.0;
        double q_wpr = 0.01;
        int armor_num = 4;
        int armor_id = 0;
        Eigen::MatrixXd Rm;  // 4×4 观测噪声
    };

    So3Ekf() = default;
    explicit So3Ekf(Params p) : p_(std::move(p)) {
        if (p_.Rm.rows() == 0) p_.Rm = Eigen::MatrixXd::Identity(4, 4);
    }

    static double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

    void init(const Eigen::Vector3d& plate, double yaw) {
        R_ = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        lr1_ = std::log(0.22);
        lr2_ = std::log(0.22);
        h_ = 0.0;
        w_ = 0.0;
        v_.setZero();
        const double r = 0.22;
        c_ = plate - R_ * Eigen::Vector3d(r, 0, 0);  // id=0 的板

        P_ = Eigen::MatrixXd::Zero(13, 13);
        const double d[13] = {1, 1, 1, 64, 64, 64, 0.4, 0.4, 0.4, 100, 1e-5, 1, 1};
        for (int i = 0; i < 13; ++i) P_(i, i) = d[i];
    }

    void setArmor(int id, int num) {
        p_.armor_id = id;
        p_.armor_num = num;
    }

    void predict(double dt) override {
        if (!(dt > 0)) return;
        const Eigen::Matrix3d R0 = R_;
        const Eigen::Vector3d c0 = c_, v0 = v_;
        const double w0 = w_, lr10 = lr1_, lr20 = lr2_, h0 = h_;

        Nominal x = propagate(nominal());
        // 数值 F：对每个误差方向扰动后传播，再用 box_minus 回到误差坐标
        for (int j = 0; j < 13; ++j) {
            const double eps = 1e-6;
            Eigen::VectorXd d = Eigen::VectorXd::Zero(13);
            d[j] = eps;
            Nominal xp = inject(d, nominal());
            Nominal xpp = propagate(xp);
            F_.col(j) = boxMinus(x, xpp) / eps;
        }
        P_ = F_ * P_ * F_.transpose() + processNoise(dt, R_);

        c_ = x.c; v_ = x.v; R_ = x.R; w_ = x.w; lr1_ = x.lr1; lr2_ = x.lr2; h_ = x.h;
        (void)R0; (void)c0; (void)v0; (void)w0; (void)lr10; (void)lr20; (void)h0;
    }

    void update(const Eigen::VectorXd& z) override {
        const Nominal x = nominal();
        const Eigen::VectorXd hx = h(x);
        Eigen::MatrixXd H(4, 13);
        for (int j = 0; j < 13; ++j) {
            const double eps = 1e-6;
            Eigen::VectorXd d = Eigen::VectorXd::Zero(13);
            d[j] = eps;
            Eigen::VectorXd hp = h(inject(d, x));
            for (int k = 0; k < 4; ++k) hp[k] = hx[k] + wrap(hp[k] - hx[k]);
            H.col(j) = (hp - hx) / eps;
        }
        Eigen::VectorXd nu = z - hx;
        nu[0] = wrap(nu[0]);
        nu[1] = wrap(nu[1]);
        nu[3] = wrap(nu[3]);

        const Eigen::MatrixXd S = H * P_ * H.transpose() + p_.Rm;
        const Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
        const Eigen::VectorXd d = K * nu;
        const Nominal xp = inject(d, x);
        c_ = xp.c; v_ = xp.v; R_ = xp.R; w_ = xp.w; lr1_ = xp.lr1; lr2_ = xp.lr2; h_ = xp.h;

        const Eigen::MatrixXd IKH = Eigen::MatrixXd::Identity(13, 13) - K * H;
        P_ = IKH * P_ * IKH.transpose() + K * p_.Rm * K.transpose();
        P_ = 0.5 * (P_ + P_.transpose());
        nu_ = nu;
        S_ = S;
        has_ = true;
    }

    [[nodiscard]] Eigen::Vector3d position() const override {
        return c_ + R_ * plateBody(p_.armor_id);
    }

    [[nodiscard]] int stateDim() const override { return 13; }
    [[nodiscard]] int measDim() const override { return 4; }

    [[nodiscard]] Eigen::VectorXd state() const override {
        Eigen::VectorXd x(13);
        x.segment<3>(0) = c_;
        x.segment<3>(3) = v_;
        x.segment<3>(6) = rotvec(R_);
        x[9] = w_; x[10] = lr1_; x[11] = lr2_; x[12] = h_;
        return x;
    }
    [[nodiscard]] Eigen::MatrixXd covariance() const override { return P_; }
    [[nodiscard]] Eigen::VectorXd innovation() const override { return nu_; }
    [[nodiscard]] Eigen::MatrixXd innovationCov() const override { return S_; }
    [[nodiscard]] double nis() const override {
        if (!has_) return 0.0;
        return nu_.dot(S_.ldlt().solve(nu_));
    }
    [[nodiscard]] double logLikelihood() const override {
        if (!has_) return -std::numeric_limits<double>::infinity();
        return -0.5 * (4 * std::log(2 * M_PI) + std::log(std::max(S_.determinant(), 1e-300)) + nis());
    }
    void setState(const Eigen::VectorXd&, const Eigen::MatrixXd&) override {}

private:
    struct Nominal {
        Eigen::Vector3d c, v;
        Eigen::Matrix3d R;
        double w = 0, lr1 = 0, lr2 = 0, h = 0;
    };

    static Eigen::Matrix3d Exp(const Eigen::Vector3d& w) {
        const double th = w.norm();
        if (th < 1e-12) return Eigen::Matrix3d::Identity();
        return Eigen::AngleAxisd(th, w / th).toRotationMatrix();
    }
    static Eigen::Vector3d rotvec(const Eigen::Matrix3d& R) {
        Eigen::AngleAxisd aa(R);
        return aa.axis() * aa.angle();
    }

    Nominal nominal() const { return {c_, v_, R_, w_, lr1_, lr2_, h_}; }

    Nominal inject(const Eigen::VectorXd& d, const Nominal& x) const {
        Nominal y = x;
        y.c += d.segment<3>(0);
        y.v += d.segment<3>(3);
        y.R = x.R * Exp(d.segment<3>(6));
        y.w += d[9];
        y.lr1 += d[10];
        y.lr2 += d[11];
        y.h += d[12];
        return y;
    }

    Eigen::VectorXd boxMinus(const Nominal& a, const Nominal& b) const {
        Eigen::VectorXd d(13);
        d.segment<3>(0) = b.c - a.c;
        d.segment<3>(3) = b.v - a.v;
        d.segment<3>(6) = rotvec(a.R.transpose() * b.R);
        d[9] = b.w - a.w;
        d[10] = b.lr1 - a.lr1;
        d[11] = b.lr2 - a.lr2;
        d[12] = b.h - a.h;
        return d;
    }

    Nominal propagate(const Nominal& x) const {
        Nominal y = x;
        y.c += x.v * dt_;
        y.R = x.R * Exp(Eigen::Vector3d(0, 0, x.w * dt_));
        return y;
    }

    Eigen::Vector3d plateBody(int id) const {
        const double th = id * 2.0 * M_PI / p_.armor_num;
        const bool lh = (p_.armor_num == 4) && (id == 1 || id == 3);
        const double r = std::exp(lh ? lr2_ : lr1_);
        const double dz = lh ? h_ : 0.0;
        return Eigen::Vector3d(r * std::cos(th), r * std::sin(th), dz);
    }

    Eigen::VectorXd h(const Nominal& x) const {
        const Eigen::Vector3d pb = plateBody(p_.armor_id);
        const Eigen::Vector3d p = x.c + x.R * pb;
        const Eigen::Vector3d n = x.R * Eigen::Vector3d(pb.x(), pb.y(), 0).normalized();
        Eigen::VectorXd z(4);
        const double q = std::sqrt(p.x() * p.x() + p.y() * p.y());
        z[0] = std::atan2(p.y(), p.x());
        z[1] = std::atan2(p.z(), q);
        z[2] = p.norm();
        z[3] = std::atan2(n.y(), n.x());
        return z;
    }

    Eigen::MatrixXd processNoise(double dt, const Eigen::Matrix3d& R) const {
        Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(13, 13);
        const Eigen::Matrix3d A = R * Eigen::Vector3d(p_.q_xyz, p_.q_xyz, p_.q_xyz).asDiagonal() * R.transpose();
        const double a = dt * dt * dt * dt / 4, b = dt * dt * dt / 2, c = dt * dt;
        Q.block<3,3>(0,0) = a * A;
        Q.block<3,3>(0,3) = b * A;
        Q.block<3,3>(3,0) = b * A;
        Q.block<3,3>(3,3) = c * A;
        Q(8,8) = a * p_.q_yaw; Q(8,9) = b * p_.q_yaw;
        Q(9,8) = b * p_.q_yaw; Q(9,9) = c * p_.q_yaw;
        Q(6,6) = c * p_.q_wpr;
        Q(7,7) = c * p_.q_wpr;
        Q(10,10) = dt * 1e-8;
        Q(11,11) = dt * 1e-8;
        Q(12,12) = dt * 1e-6;
        return Q;
    }

    Params p_;
    Eigen::Vector3d c_ = Eigen::Vector3d::Zero(), v_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R_ = Eigen::Matrix3d::Identity();
    double w_ = 0, lr1_ = std::log(0.22), lr2_ = std::log(0.22), h_ = 0;
    Eigen::MatrixXd P_ = Eigen::MatrixXd::Identity(13, 13);
    Eigen::MatrixXd F_ = Eigen::MatrixXd::Identity(13, 13);
    Eigen::VectorXd nu_ = Eigen::VectorXd::Zero(4);
    Eigen::MatrixXd S_ = Eigen::MatrixXd::Identity(4, 4);
    double dt_ = 0.0;
    bool has_ = false;
};

}  // namespace kf
