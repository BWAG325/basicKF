#ifndef KALMAN_HPP
#define KALMAN_HPP

//#define QUEEN_LENGTH 20 //计算方差的窗口，窗口越大，滞后越大
#include <Eigen/Dense>
#include <functional>


// Extended Kalman Filter with auto differentiation
// N_X: state vector dimension
// N_Z: measurement vector dimension
// PredicFunc: process nonlinear vector function
// MeasureFunc: observation nonlinear vector function
template<int N_X, int N_Z>
class KalmanFilter {
public:
    KalmanFilter() = default;

    using MatrixXX = Eigen::Matrix<double, N_X, N_X>;
    using MatrixZX = Eigen::Matrix<double, N_Z, N_X>;
    using MatrixXZ = Eigen::Matrix<double, N_X, N_Z>;
    using MatrixZZ = Eigen::Matrix<double, N_Z, N_Z>;
    using MatrixX1 = Eigen::Matrix<double, N_X, 1>;
    using MatrixZ1 = Eigen::Matrix<double, N_Z, 1>;
    using Matrix11 = Eigen::Matrix<double, 1, 1>;

    using AFunc = std::function<MatrixXX()>;
    using HFunc = std::function<MatrixZX()>;
    using PFunc = std::function<MatrixXX()>;
    using QFunc = std::function<MatrixXX()>;
    using RFunc = std::function<MatrixZZ(const MatrixZ1 &z)>;

    explicit KalmanFilter(AFunc u_A,
                          HFunc u_H,
                          PFunc u_P,
                          QFunc u_q,
                          RFunc u_r) noexcept
            : update_A(u_A), update_H(u_H), update_P(u_P),update_Q(u_q), update_R(u_r) {

    }

    // Set the initial state
    void setState(const MatrixX1 &x0) noexcept { 
    	MatrixZ1 z ;
        z << x0(0),x0(3),x0(6);
        
        A = update_A();
        H = update_H();
	P = update_P();
        Q = update_Q();
        R = update_R(z);

        x = x0;
    }

    void Qset(MatrixXX Q) noexcept { this->Q = Q; }

    void Rset(MatrixZZ R) noexcept { this->R = R; }

    void Aset(const MatrixXX A) noexcept { this->A = A; }

    void Hset(const MatrixZX H) noexcept { this->H = H; }

    void Pset(const MatrixXX P) noexcept { this->P = P; }

    // Compute a predicted state
    MatrixX1 predict() noexcept {
        A = update_A(); 
        x = A * x;
        Q = update_Q();
        P = A * P * A.transpose() + Q;
        
        return x;
    }

    // Update the estimated state based on measurement
    MatrixX1 update(const MatrixZ1 &z) noexcept {
          H = update_H();
                
        MatrixXZ K = P * H.transpose() * ((H * P * H.transpose() + R).inverse());
        x = x + K * (z - H * x);
        MatrixXX Ind;Ind.setIdentity();
        //P = (Ind - K * H) * P;
        P = (Ind - K*H) * P * (Ind - K*H).transpose() + K * R * K.transpose();
        R = update_R(z);
        return x;
    }

private:
    //温馨提示,匀加速和匀速模型的维数不一样哦

    MatrixXX A;    //模型
    MatrixZX H;    //测量模型
    MatrixXX P;    //预测变量的协方差
    MatrixXX Q;    //过程噪声协方差
    MatrixZZ R;    //观测噪声协方差

    MatrixX1 x;    //系统参数


    //更新系统参数，变频卡尔曼
    AFunc update_A;
    HFunc update_H;
    PFunc update_P;
    QFunc update_Q;
    RFunc update_R;
};


#endif
