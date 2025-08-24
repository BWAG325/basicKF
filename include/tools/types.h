//
// Created by mmz on 2025/8/24.
//

#ifndef BASICKF_VALUE_H
#define BASICKF_VALUE_H

#include <Eigen/Dense>

//BCN 卡尔曼矩阵大小
constexpr int X_N = 9, Z_N = 3;

//BCN 异常情况重置卡尔曼使用
constexpr double max_x_x = 10, max_y_x = 10, max_z_x = 10;
constexpr double max_x_v = 10, max_y_v = 10, max_z_v = 10;
constexpr double max_x_a = 1, max_y_a = 1, max_z_a = 1;

//虽然处理的是二维点，还是预留一下三维的接口
//NOTE 数据读取格式
struct kfState {
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d a;

    kfState() = default;

    void saveState(Eigen::Matrix<double, X_N, 1> data) {
        p.x() = data(0);
        p.y() = data(3);
        p.z() = data(6);

        v.x() = data(1);
        v.y() = data(4);
        v.z() = data(7);

        a.x() = data(2);
        a.y() = data(5);
        a.z() = data(8);
    }
};

//NOTE 传入数据类型
struct kfData {
    //框的中心点
    Eigen::Vector3d p;
};

//NOTE 参数传入格式
struct Q {
    double q_x_;
    double q_y_;
    double q_z_;

    Q() = default;
    Q(double q_x, double q_y, double q_z) : q_x_(q_x), q_y_(q_y), q_z_(q_z) {};
};

struct R {
    double r_x_;
    double r_y_;
    double r_z_;

    R() = default;
    R(double r_x, double r_y, double r_z) : r_x_(r_x), r_y_(r_y), r_z_(r_z) {};
};

struct kfValue {
    Q q;
    R r;

    kfValue() = default;
    kfValue(const Q& q_, const R& r_) : q(q_), r(r_) {};
};
#endif //BASICKF_VALUE_H
