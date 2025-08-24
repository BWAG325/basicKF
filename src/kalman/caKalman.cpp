//
// Created by mmz on 25-3-16.
//

#include "kalman/caKalman.h"

void caKalman::init() {
     // clang-format off
    std::function kfA = [this]() {
         Eigen::Matrix<double, X_N, X_N> A;
          //   xx     vx      ax      xy      vy      ay      xz      vz      az
         A << 1, deltaT, pow(deltaT, 2) / 2, 0,     0,              0,           0,   0,             0,
              0,   1,            deltaT,         0,     0,              0,           0,   0,             0,
              0,   0,               1,           0,     0,              0,           0,   0,             0,
              0,   0,               0,           1, deltaT, pow(deltaT, 2) / 2,  0,   0,             0,
              0,   0,               0,           0,     1,           deltaT,         0,   0,             0,
              0,   0,               0,           0,     0,              1,           0,   0,             0,
              0,   0,               0,           0,     0,              0,           1, deltaT, pow(deltaT, 2) / 2,
              0,   0,               0,           0,     0,              0,           0,   1,            deltaT,
              0,   0,               0,           0,     0,              0,           0,   0,             1;
         return A;
     };
     std::function kfH = [this]() {
          Eigen::Matrix<double, Z_N, X_N> H;
          H << 1, 0, 0, 0, 0, 0, 0, 0, 0,
               0, 0, 0, 1, 0, 0, 0, 0, 0,
               0, 0, 0, 0, 0, 0, 1, 0, 0;
          return H;
     };

     std::function kfQ = [this]()->Eigen::Matrix<double,X_N,X_N> {
          Eigen::Vector<double, 3> u;
          u << value.q.q_x_, value.q.q_y_, value.q.q_z_;
          Eigen::MatrixXd U = u.asDiagonal();
          Eigen::Matrix<double, X_N, 3> G;
          G << pow(deltaT, 3) / 6.0, 0, 0,
               pow(deltaT, 2) / 2.0, 0, 0,
               deltaT, 0, 0,
               0, pow(deltaT, 3) / 6.0, 0,
               0, pow(deltaT, 2) / 2.0, 0,
               0, deltaT, 0,
               0, 0, pow(deltaT, 3) / 6.0,
               0, 0, pow(deltaT, 2) / 2.0,
               0, 0, deltaT;

          return G*U*G.transpose();
     };
     std::function kfR = [this](const Eigen::Matrix<double, Z_N, 1>& z) {
          (void) z;
          Eigen::Matrix<double,Z_N,Z_N> R;
          R << value.r.r_x_, 0, 0,
               0, value.r.r_y_, 0,
               0, 0, value.r.r_z_;
          // clang-format on
          return R;
     };
     std::function kfP = [this]() {
          return Eigen::Matrix<double, X_N, X_N>::Identity();
     };
    KF = std::make_unique<KalmanFilter<X_N,Z_N>>(kfA,kfH,kfP,kfQ,kfR);
}

void caKalman::setStatus(Eigen::Matrix<double, Z_N, 1> x0,double dt_) {
     Eigen::Matrix<double, X_N, 1> inputX;
     inputX << x0[0],0,0,
               x0[1],0,0,
               x0[2],0,0;
     deltaT = dt_;
     KF->setState(inputX);
}

void caKalman::setValueR(const R& r_) {
     this->value.r = r_;
}

void caKalman::setValueQ(const Q& q_) {
     this->value.q = q_;
}

//不会判断是否设置过 R Q
Eigen::Matrix<double, X_N, 1> caKalman::update(const Eigen::Matrix<double, Z_N, 1>& z,double dt_) {
     deltaT = dt_;
     Eigen::Matrix<double, X_N, 1> x_new = KF->update(z);
     return x_new;
}

Eigen::Matrix<double, X_N, 1> caKalman::predict(double dt_) {
     deltaT = dt_;
     Eigen::Matrix<double,X_N,1> pre = KF->predict();

     return pre;
}


