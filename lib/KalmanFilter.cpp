#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(int state_dim, int meas_dim, int control_dim)
    : state_dim(state_dim), meas_dim(meas_dim),control_dim(control_dim),
      x(state_dim), P(state_dim, state_dim),
      A(state_dim, state_dim), Q(state_dim, state_dim),
      H(meas_dim, state_dim), R(meas_dim, meas_dim) 
    {
        if(control_dim > 0)
        {
            B = Eigen::MatrixXd(state_dim, control_dim);;
        }
        x.setZero();
        P.setIdentity();
        A.setIdentity();
        Q.setIdentity();
        H.setZero();
        R.setIdentity();
        if (control_dim > 0) 
        {
            B.setZero();
        }
    }

// 设置初始状态和协方差矩阵
void KalmanFilter::setInitialState(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0)
{
    x = x0;
    P = P0;
}

// 设置状态转移矩阵、控制矩阵、过程噪声协方差矩阵
void KalmanFilter::setSystemMatrices(const Eigen::MatrixXd& A, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& B)
{
    this->A = A;
    this->Q = Q;
    if (control_dim > 0 && B.size() > 0)
    {
        this->B = B;
    }
    
}

// 设置测量矩阵和测量噪声协方差矩阵
void KalmanFilter::setMeasurementMatrices(const Eigen::MatrixXd& H, const Eigen::MatrixXd& R) 
{
    this->H = H;
    this->R = R;
}

// 预测步骤
void KalmanFilter::predict(const Eigen::VectorXd& u)
{
    if (control_dim > 0 && u.size() > 0)
    {
       x = A * x + B *u;
    }else{
        x = A * x;
    }
    P = A * P * A.transpose() + Q;
}

// 更新步骤
void KalmanFilter::update(const Eigen::VectorXd& z)
{
    Eigen::VectorXd y = z - H * x; //计算测量残差
    Eigen::MatrixXd S = H * P * H.transpose() + R;  // 计算残差协方差
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();  // 计算卡尔曼增益
    x = x + K * y;  // 更新状态
    P = (Eigen::MatrixXd::Identity(state_dim, state_dim) - K * H) * P;  // 更新协方差矩阵
}

// 获取当前状态和协方差矩阵
Eigen::VectorXd KalmanFilter::getState() const {
    return x;
}

Eigen::MatrixXd KalmanFilter::getCovariance() const {
    return P;
}