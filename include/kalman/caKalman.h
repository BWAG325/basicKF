//
// Created by mmz on 25-3-16.
//

#ifndef KALMAN_H
#define KALMAN_H

#include "kalman_filter.hpp"
#include "tools/types.h"

#include <memory>

class caKalman {
    std::unique_ptr<KalmanFilter<X_N, Z_N>> KF;

    double deltaT;
    kfValue value;
public:
    explicit caKalman() = default;

    void init();

    void setStatus(Eigen::Matrix<double,Z_N,1> x0,double dt_);

    void setValueR(const R& r_);
    void setValueQ(const Q& q_);

    Eigen::Matrix<double, X_N, 1> update(const Eigen::Matrix<double, Z_N, 1>& x,double dt_);

    Eigen::Matrix<double, X_N, 1> predict(double dt_);


};


#endif //KALMAN_H
