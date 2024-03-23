//
// Created by admin on 2023/9/19.
//

#ifndef CONTROL_FRAME_MAIN_KALMAN_FILTER_H
#define CONTROL_FRAME_MAIN_KALMAN_FILTER_H

#include <string>
#include <vector>
//#include <Dense>//包含Eigen矩阵运算库，用于矩阵计算
#include <cmath>
//using Eigen::MatrixXd;

// 一维滤波器信息结构体
typedef  struct Kalman_Regulator_single{
    float filterValue;  //k-1时刻的滤波值，即是k-1时刻的值
    float kalmanGain;   //   Kalman增益
    float A;   // x(n)=A*x(n-1)+u(n),u(n)~N(0,Q)
    float H;   // z(n)=H*x(n)+w(n),w(n)~N(0,R)
    float Q;   //预测过程噪声偏差的方差
    float R;   //测量噪声偏差，(系统搭建好以后，通过测量统计实验获得)
    float P;   //估计误差协方差
}  Kalman_Regulator_single;

/*typedef  struct Kalman_Regulator_matrix{
    int degree;

    MatrixXd A;
    MatrixXd B;
    MatrixXd H;
    MatrixXd Q;
    MatrixXd R;
    MatrixXd X_evlt;
    MatrixXd Pk;
    MatrixXd K;

}  Kalman_Regulator_matrix;*/

class Kalman_single{
public:
    Kalman_Regulator_single KalmanInfo{};
    //void Init_KalmanInfo(KalmanInfo* info, double Q, double R);
    float KalmanFilter_single(Kalman_Regulator_single* kalmanInfo, float lastMeasurement);

};

/*class Kalman_matrix{
public:
    Kalman_Regulator_matrix KalmanInfo{};
    MatrixXd KalmanFilter_matrix(Kalman_Regulator_matrix* kalmanInfo, MatrixXd Z_meas);
    
};*/
#endif //CONTROL_FRAME_MAIN_KALMAN_FILTER_H
