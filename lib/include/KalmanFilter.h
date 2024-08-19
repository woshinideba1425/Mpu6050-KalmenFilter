#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <ArduinoEigen.h>

class KalmanFilter {
public:
    // 构造函数
    KalmanFilter(int state_dim, int meas_dim, int control_dim = 0);

    // 设置初始状态和协方差矩阵
    void setInitialState(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0);

    // 设置状态转移矩阵、控制矩阵、过程噪声协方差矩阵
    void setSystemMatrices(const Eigen::MatrixXd& A, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& B = Eigen::MatrixXd());

    // 设置测量矩阵和测量噪声协方差矩阵
    void setMeasurementMatrices(const Eigen::MatrixXd& H, const Eigen::MatrixXd& R);

    // 预测步骤
    void predict(const Eigen::VectorXd& u = Eigen::VectorXd());

    // 更新步骤
    void update(const Eigen::VectorXd& z);

    // 获取当前状态和协方差矩阵
    Eigen::VectorXd getState() const;
    Eigen::MatrixXd getCovariance() const;

private:
    int state_dim;
    int meas_dim;
    int control_dim;

    Eigen::VectorXd x;  // 状态向量
    Eigen::MatrixXd P;  // 状态协方差矩阵
    Eigen::MatrixXd A;  // 状态转移矩阵
    Eigen::MatrixXd B;  // 控制输入矩阵
    Eigen::MatrixXd Q;  // 过程噪声协方差矩阵
    Eigen::MatrixXd H;  // 测量矩阵
    Eigen::MatrixXd R;  // 测量噪声协方差矩阵
};

#endif // KALMAN_FILTER_H
