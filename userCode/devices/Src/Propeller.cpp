//
// Created by admin on 2023/10/30.
//
#include "Propeller.h"
#include "Sensor.h"
#include "IMU.h"

// V30
int32_t InID_V30[4] = {1, 2, 6, 5}; // 内部的4个轮，左前-左后-右前-右后
int32_t OutID_V30[4] = {3, 0, 7, 4}; // 外部的4个轮，左前-左后-右前-右后
int32_t InitPWM_V30 = 1500;
int32_t PWM_V30[7][4] = {
    {1580, 1600, 1590, 1595}, // Base
    {1450, 1460, 1600, 1600}, // Front
    {1600, 1600, 1460, 1470}, // Back
    {1590, 1455, 1600, 1455}, // Left
    {1465, 1590, 1460, 1600}, // Right
    {1475, 1475, 1475, 1475}, // ClockWise
    {1585, 1585, 1585, 1585}  // AntiClockWise  
};
 
// 直接用构造函数
PID_Regulator_t DepthPID_V30(20, 0.005, 10, 100, 100, 100, 200);
PID_Regulator_t PitchPID_V30(10,/*5*/ 0.04, 200, 100, 100, 100, 200);
PID_Regulator_t RollPID_V30(5, /*2.5*/ 0.02, 100, 100, 100, 100, 200);
PID_Regulator_t YawPID_V30(30, 0.02, 1000, 100, 100, 100, 200);

Propeller_Parameter_t Parameter_V30(InID_V30, OutID_V30, InitPWM_V30, PWM_V30, DepthPID_V30, PitchPID_V30, RollPID_V30, YawPID_V30);


// V31
int32_t InID_V31[4] = {4, 5, 7, 6}; // 内部的4个轮，左前-左后-右前-右后
int32_t OutID_V31[4] = {1, 0, 2, 3}; // 外部的4个轮，左前-左后-右前-右后
int32_t InitPWM_V31 = 1550;
int32_t PWM_V31[7][4] = {
    {1460, 1400, 1460, 1715}, // Base
    {1450, 1460, 1600, 1600}, // Front
    {1600, 1600, 1460, 1470}, // Back
    {1590, 1455, 1600, 1455}, // Left
    {1465, 1590, 1460, 1600}, // Right
    {1475, 1475, 1475, 1475}, // ClockWise
    {1585, 1585, 1585, 1585}  // AntiClockWise  
};
PID_Regulator_t DepthPID_V31(20, 0.005, 10, 100, 100, 100, 200); 
// PID_Regulator_t PitchPID_V31(0,0,0, 100, 100, 100, 200);
PID_Regulator_t PitchPID_V31(25,/*5*/ 0.02, 300, 100, 100, 100, 200);
// PID_Regulator_t RollPID_V31(0,0, 0, 100, 100, 100, 200);
PID_Regulator_t RollPID_V31(15,/*2.5*/ 0.01, 150, 200, 100, 100, 200);
PID_Regulator_t YawPID_V31(30, 0.02, 1000, 100, 100, 100, 200);

Propeller_Parameter_t Parameter_V31(InID_V31, OutID_V31, InitPWM_V31, PWM_V31, DepthPID_V31, PitchPID_V31, RollPID_V31, YawPID_V31);


void Propeller_I2C::Init(){
    std::memcpy(&Parameter,&Parameter_V31, sizeof(Propeller_Parameter_t));
    DepthPID.PIDInfo = Parameter.DepthPID_P;
    PitchPID.PIDInfo = Parameter.PitchPID_P;
    RollPID.PIDInfo = Parameter.RollPID_P;
    YawPID.PIDInfo = Parameter.YawPID_P;
    Target_depth = 30;
    Target_angle = 0;
    Target_speed[0] = 0;
    Target_speed[1] = 0;
    flag_PID = false;
    TCA_SetChannel(4);
    HAL_Delay(5);
    PCA_Write(PCA9685_MODE1,0x0);
    PCA_Setfreq(50); // Hz
    for(int i = 0;i < PROPELLER_NUM; ++i){
        data[i] = Parameter.InitPWM;
        //------TODO：i为推进器在扩展版上的接口编号，根据接线修改，目前为0-7号
        PCA_Setpwm(i, 0, floor(data[i] * 4096 / 20000 + 0.5f));
    }
    /*data_receive[0] = 1500;
    data_receive[1] = 1500;
    data_receive[2] = 1500;
    data_receive[3] = 1500;
    data_receive[4] = 300;*/

    HAL_UARTEx_ReceiveToIdle_IT(&huart6, RxBuffer, SERIAL_LENGTH_MAX);
};




void Propeller_I2C::Receive(){

    int32_t data_receive[8];
    unordered_map<char, int32_t *> mp;
    mp['W'] = Parameter.FrontPWM;
    mp['S'] = Parameter.BackPWM;
    mp['A'] = Parameter.LeftPWM;
    mp['D'] = Parameter.RightPWM;
    mp['E'] = Parameter.ClockwisePWM;
    mp['Q'] = Parameter.AnticlockwisePWM;


    if(flag_PID){
        if (mp.count(RxBuffer[0])){
            for(int i = 0; i < 4; ++i){
                data[Parameter.OutID[i]] = mp[RxBuffer[0]][i];
            }
        }
        if (strncmp((char*)RxBuffer, "OFF", 3) == 0) {
            flag_PID = false;
            for(int i=0;i<8;++i){
                data[i] = Parameter.InitPWM;
            }
        }

        if (strncmp((char*)RxBuffer, "PRO:", 4) == 0) {
            char *data_str = (char*)RxBuffer + 4;
            char *token = strtok(data_str, ",");
            int i = 0;
            while (token != NULL && i < 5) {
                data_receive[i] = atoi(token);
                token = strtok(NULL, ",");
                i++;
            }
            for(int i=0;i<4;++i){
                data[Parameter.OutID[i]] = data_receive[i];
            }
            Target_depth = data_receive[4]/10.0;
        }

        if (strncmp((char*)RxBuffer, "VEL:", 4) == 0) {
            char *data_str = (char*)RxBuffer + 4;
            char *token = strtok(data_str, ",");
            int i = 0;
            while (token != NULL && i < 3) {
                data_receive[i] = atoi(token);
                token = strtok(NULL, ",");
                i++;
            }
            Target_speed[0] = data_receive[0];
            Target_speed[1] = data_receive[1];
            Target_angle = data_receive[2]*3.14/180;
        }

        if (strncmp((char*)RxBuffer, "H:", 2) == 0) {
            char *data_str = (char*)RxBuffer + 2;
            Target_depth = atoi(data_str)/10.0;
        }

        if (strncmp((char*)RxBuffer, "Z", 1) == 0) {
            for(int i=0;i<4;++i){
                data[Parameter.OutID[i]] = Parameter.InitPWM;
            }
        }   
    }
    
    else{
        if (strncmp((char*)RxBuffer, "ON", 2) == 0) {
            flag_PID = true;
            for(int i=0;i<8;++i){
                data[i] = Parameter.InitPWM;
            }
        }

        if (strncmp((char*)RxBuffer, "TES:", 4) == 0) {
            char *data_str = (char*)RxBuffer + 4;
            char *token = strtok(data_str, ",");
            int i = 0;
            while (token != NULL && i < 8) {
                data[i] = atoi(token);
                token = strtok(NULL, ",");
                i++;
            }
        }
    }  
}


void Propeller_I2C::Handle(){

    TCA_SetChannel(4);
    //HAL_Delay(5);
    if(flag_PID){
        float_ctrl();//PID控制悬浮状态
        // speed_ctrl();
    }
    for (int i = 0; i < PROPELLER_NUM; ++i) {
        //------TODO：i为推进器在扩展版上的接口编号，根据接线修改，目前为0-7号
        PCA_Setpwm(i, 0, floor(data[i] * 4096 / 20000 + 0.5f));
    }
}

void Propeller_I2C::float_ctrl() {

    Component.Depth =  DepthPID.PIDCalc(Target_depth, PressureSensor::pressure_sensor.data_depth);
    Component.Roll =  RollPID.PIDCalc(0.0, PressureSensor::pressure_sensor.data_roll);
    Component.Pitch =  PitchPID.PIDCalc(0.0, PressureSensor::pressure_sensor.data_pitch);

    data[Parameter.InID[0]] = Parameter.BasePWM[0] - (- Component.Depth - Component.Roll - Component.Pitch);
    data[Parameter.InID[1]] = Parameter.BasePWM[1] - (- Component.Depth -Component.Roll + Component.Pitch);
    data[Parameter.InID[2]] = Parameter.BasePWM[2] - (- Component.Depth + Component.Roll - Component.Pitch);
    data[Parameter.InID[3]] = Parameter.BasePWM[3] - Component.Depth + Component.Roll + Component.Pitch;

}

void Propeller_I2C::speed_ctrl(){

    //Component.Vx =  DepthPID.PIDCalc(Target_speed[0], IMU::imu.position._velocity[0]);
    //Component.Vy =  RollPID.PIDCalc(Target_speed[1], IMU::imu.position._velocity[1]);
    Component.Vx = Target_speed[0];
    Component.Vy = Target_speed[1];
    angle_error = Target_angle - IMU::imu.attitude.yaw;
    if(angle_error < -3.14) angle_error += 6.28;
    else if(angle_error > 3.14) angle_error -= 6.28;

    Component.Yaw = YawPID.PIDCalc(0, angle_error);

    data[Parameter.OutID[0]] = 1530 + Component_Calc(- Component.Yaw - Component.Vx + Component.Vy);
    data[Parameter.OutID[1]] = 1530 + Component_Calc(- Component.Yaw - Component.Vx - Component.Vy);
    data[Parameter.OutID[2]] = 1530 + Component_Calc(- Component.Yaw + Component.Vx + Component.Vy);
    data[Parameter.OutID[3]] = 1530 + Component_Calc(- Component.Yaw + Component.Vx - Component.Vy);

}

float Propeller_I2C::Component_Calc(float data){

    if(data > 0) return(data+35);
    else return(data-35);

}

