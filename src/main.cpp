#include <Arduino.h>
#include <KalmanFilter.h>
#include <BiquadFilter.h>
#include <MPU6050_tockn.h>

float mpuX, mpuY, mpuZ, gyroZ;
KalmanFilter kf(2,2);
BiquadFilter xFlilter(0.0133592,0.0267184,0.0133592,1,-1.64745998,0.70089678), 
yFlilter(0.0133592,0.0267184,0.0133592,1,-1.64745998,0.70089678),
zFlilter(0.00362168,0.00724336,0.00362168,1,-1.82269493,0.83718165);
unsigned long preInterval = 0;
MPU6050 mpu6050;

void setup() {
    
    mpu6050.begin();
    mpu6050.calcGyroOffsets(true);
    Serial.println("mpu6050 setup done");

    TickType_t lastFixTime = xTaskGetTickCount();
    // 初始状态 [角度, 角速度]
    Eigen::VectorXd x0(2);
    x0 << 0, 0;
    
    // 初始协方差矩阵
    Eigen::MatrixXd P0(2, 2);
    P0 << 1, 0,
          0, 1;

    kf.setInitialState(x0, P0);

    // 状态转移矩阵 A
    double dt = 0.02;  // 假设采样时间间隔为0.02秒
    Eigen::MatrixXd A(2, 2);
    A << 1, dt,
         0, 1;

    // 过程噪声协方差矩阵 Q
    Eigen::MatrixXd Q(2, 2);
    Q << 0.0025, 0,
         0, 0.0025;

    kf.setSystemMatrices(A, Q);

    // 测量矩阵 H
    Eigen::MatrixXd H(2, 2);
    H << 1, 0,
         0, 1;

    // 测量噪声协方差矩阵 R
    Eigen::MatrixXd R(2, 2);
    R << 0.1, 0,
         0, 0.1;

    kf.setMeasurementMatrices(H, R);

    preInterval = millis();
}

void loop() { 
    mpu6050.update();
    mpuX = xFlilter.process(mpu6050.getAngleX());
    mpuY = yFlilter.process(mpu6050.getAngleY());
    mpuZ = zFlilter.process(mpu6050.getAngleZ());

    gyroZ = mpu6050.getGyroZ();

    float interval = (millis() - preInterval) * 0.001;

    // 预测步骤
    kf.predict();

    // 更新步骤
    Eigen::VectorXd z(2);
    z << mpuZ, gyroZ;

    kf.update(z);

    // 获取滤波后的状态
    Eigen::VectorXd x = kf.getState();
    float kalmanAngleZ = x(0);
    
    printf("%.2f, %.2f, %.2f",mpuX, mpuY, kalmanAngleZ);

    preInterval = millis();
    vTaskDelay(pdMS_TO_TICKS(20));
}


