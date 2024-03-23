//
// Created by admin on 2023/9/19.
//
#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H
#include <Kalman_Filter.h>

//using Eigen::MatrixXd;

/*void Init_KalmanInfo(KalmanInfo* info, double Q, double R)
{
    info->A = 1;  //标量卡尔曼
    info->H = 1;  //
    info->P = 10;  //后验状态估计值误差的方差的初始值（不要为0问题不大）
    info->Q = Q;    //预测（过程）噪声方差 影响收敛速率，可以根据实际需求给出
    info->R = R;    //测量（观测）噪声方差 可以通过实验手段获得
    info->filterValue = 0;// 测量的初始值
}*/
float Kalman_single::KalmanFilter_single(Kalman_Regulator_single* kalmanInfo, float lastMeasurement)
{
    //预测下一时刻的值
    float predictValue = kalmanInfo->A* kalmanInfo->filterValue;
    //x的先验估计由上一个时间点的后验估计值和输入信息给出，此处需要根据基站高度做一个修改

    //求协方差
    kalmanInfo->P = kalmanInfo->A*kalmanInfo->A*kalmanInfo->P + kalmanInfo->Q;
    //计算先验均方差 p(n|n-1)=A^2*p(n-1|n-1)+q

    float preValue = kalmanInfo->filterValue;  //记录上次实际坐标的值

    //计算kalman增益
    kalmanInfo->kalmanGain = kalmanInfo->P*kalmanInfo->H / (kalmanInfo->P*kalmanInfo->H*kalmanInfo->H + kalmanInfo->R);
    //Kg(k)= P(k|k-1) H’ / (H P(k|k-1) H’ + R)

    //修正结果，即计算滤波值
    kalmanInfo->filterValue = predictValue + (lastMeasurement - predictValue)*kalmanInfo->kalmanGain;  //利用残余的信息改善对x(t)的估计，给出后验估计，这个值也就是输出  X(k|k)= X(k|k-1)+Kg(k) (Z(k)-H X(k|k-1))

    //更新后验估计
    kalmanInfo->P = (1 - kalmanInfo->kalmanGain*kalmanInfo->H)*kalmanInfo->P;
    //计算后验均方差  P[n|n]=(1-K[n]*H)*P[n|n-1]

    return  kalmanInfo->filterValue;
}


/*
    const double delta_t = 0.1;//控制周期，100ms
    const int num = 100;//迭代次数
    const double acc = 10;//加速度，ft/m

    MatrixXd A(2,2);
    A(0,0) = 1;
    A(1,0) = 0;
    A(0,1) = delta_t;
    A(1,1) = 1;

    MatrixXd B(2,1);
    B(0,0) = pow(delta_t,2)/2;
    B(1,0) = delta_t;

    MatrixXd H(1,2);//测量的是小车的位移，速度为0
    H(0,0) = 1;
    H(0,1) = 0;

    MatrixXd Q(2,2);//过程激励噪声协方差，假设系统的噪声向量只存在速度分量上，且速度噪声的方差是一个常量0.01，位移分量上的系统噪声为0
    Q(0,0) = 0;
    Q(1,0) = 0;
    Q(0,1) = 0;
    Q(1,1) = 0.01;

    MatrixXd R(1,1);//观测噪声协方差，测量值只有位移，它的协方差矩阵大小是1*1，就是测量噪声的方差本身。
    R(0,0) = 10;
*/

    //变量定义，包括状态预测值，状态估计值，测量值，预测状态与真实状态的协方差矩阵，估计状态和真实状态的协方差矩阵，初始值均为零
    //Pk = MatrixXd::Constant(2,2,0),  K = MatrixXd::Constant(2,1,0);


    //开始迭代
  /*  MatrixXd Kalman_matrix::KalmanFilter_matrix(Kalman_Regulator_matrix* kalmanInfo, MatrixXd Z_meas){
        //预测值
        MatrixXd X_pdct(kalmanInfo->degree, 1);
        X_pdct = kalmanInfo->A * kalmanInfo->X_evlt;// + kalmanInfo_matrix->B * acc;
        //预测状态与真实状态的协方差矩阵，Pk'
        MatrixXd Pk_p(kalmanInfo->degree, kalmanInfo->degree);
        Pk_p = kalmanInfo->A * kalmanInfo->Pk * kalmanInfo->A.transpose() + kalmanInfo->Q;

        MatrixXd tmp(1,1);
        tmp = kalmanInfo->H * Pk_p * kalmanInfo->H.transpose() + kalmanInfo->R;

        kalmanInfo->K = Pk_p * kalmanInfo->H.transpose() * tmp.inverse();

        kalmanInfo->X_evlt = X_pdct + kalmanInfo->K * (Z_meas - kalmanInfo->H * X_pdct);

        kalmanInfo->Pk = (MatrixXd::Identity(kalmanInfo->degree, kalmanInfo->degree) - kalmanInfo->K * kalmanInfo->H) * Pk_p;//估计状态和真实状态的协方差矩阵，Pk

        return kalmanInfo->X_evlt;
    }
*/

#endif // KALMAN_FILTER_H
