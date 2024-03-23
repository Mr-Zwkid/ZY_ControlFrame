//
// Created by admin on 2023/10/30.
//
#include "Propeller.h"
#include "Sensor.h"
#include "IMU.h"

int32_t InID_V30[4] = {1, 2, 6, 5};
int32_t OutID_V30[4] = {3, 0, 7, 4};
int32_t InitPWM_V30 = 1500;
int32_t BasePWM_V30[4] = {1580, 1600, 1590, 1595};
int32_t FrontPWM_V30[4] = {1450, 1460, 1600, 1600};
int32_t BackPWM_V30[4] = {1600, 1600, 1460, 1470};
int32_t LeftPWM_V30[4] = {1590, 1455, 1600, 1455};
int32_t RightPWM_V30[4] = {1465, 1590, 1460, 1600};
int32_t ClockwisePWM_V30[4] = {1475, 1475, 1475, 1475};
int32_t AnticlockwisePWM_V30[4] = {1585, 1585, 1585, 1585};
  
PID_Regulator_t DepthPID_V30={
    .kp = 20,
    .ki = 0.005,
    .kd = 10,
    .componentKpMax = 100,
    .componentKiMax = 100,
    .componentKdMax = 100,
    .outputMax = 200
};
    
PID_Regulator_t PitchPID_V30 = {
    .kp = 10,//5,
    .ki = 0.04,
    .kd = 200.0,
    .componentKpMax = 100,
    .componentKiMax = 100,
    .componentKdMax = 100,
    .outputMax = 200
};
    
PID_Regulator_t RollPID_V30 = {
    .kp = 5,//2.5,
    .ki = 0.02,
    .kd = 100.0,
    .componentKpMax = 100,
    .componentKiMax = 100,
    .componentKdMax = 100,
    .outputMax = 200
};
 
PID_Regulator_t YawPID_V30={
    .kp = 30,
    .ki = 0.02,
    .kd = 1000,
    .componentKpMax = 100,
    .componentKiMax = 100,
    .componentKdMax = 100,
    .outputMax = 200
};
    
PID_Regulator_t VxPID_V30 = {
    .kp = 1,
    .ki = 0,
    .kd = 0,
    .componentKpMax = 100,
    .componentKiMax = 100,
    .componentKdMax = 100,
    .outputMax = 200
};
    
PID_Regulator_t VyPID_V30 = {
    .kp = 1,
    .ki = 0,
    .kd = 0,
    .componentKpMax = 100,
    .componentKiMax = 100,
    .componentKdMax = 100,
    .outputMax = 200
};

Propeller_Parameter_t Parameter_V30;

void Propeller::Init(){

    
    for(int i=0;i<PROPELLER_NUM;++i){
        data[i]=1500;
    }
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, data[0]);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, data[1]);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, data[2]);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, data[3]);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, data[4]);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, data[5]);
    HAL_UARTEx_ReceiveToIdle_IT(&huart6, RxBuffer, SERIAL_LENGTH_MAX);
}

void Propeller_I2C::Init(){
    //------TODO：以下为三个自由度的PID参数，需要调试
    DataCopy_V30();
    Target_depth = 30;
    Target_angle = 0;
    Target_speed[0] = 0;
    Target_speed[1] = 0;
    flag_PID = false;
    TCA_SetChannel(4);
    HAL_Delay(5);
    PCA_Write(PCA9685_MODE1,0x0);
    PCA_Setfreq(50);//Hz
    for(int i=0;i<PROPELLER_NUM;++i){
        data[i] = Parameter.InitPWM;
        //------TODO：i为推进器在扩展版上的接口编号，根据接线修改，目前为0-7号
        PCA_Setpwm(i,0,floor(data[i] * 4096 / 20000 + 0.5f));
    }
    /*data_receive[0] = 1500;
    data_receive[1] = 1500;
    data_receive[2] = 1500;
    data_receive[3] = 1500;
    data_receive[4] = 300;*/

    HAL_UARTEx_ReceiveToIdle_IT(&huart6, RxBuffer, SERIAL_LENGTH_MAX);

}

void Propeller::Receive(){
    data_extract(RxBuffer, data, PROPELLER_NUM);
    //HAL_UARTEx_ReceiveToIdle_IT(&huart6, RxBuffer, SERIAL_LENGTH_MAX);
}


void Propeller_I2C::Receive(){

    int32_t data_receive[8];
    if(flag_PID){

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
        
        if (strncmp((char*)RxBuffer, "W", 1) == 0) {
            for(int i=0;i<4;++i){
                data[Parameter.OutID[i]] = Parameter.FrontPWM[i];
            }
        }

        if (strncmp((char*)RxBuffer, "S", 1) == 0) {
           for(int i=0;i<4;++i){
                data[Parameter.OutID[i]] = Parameter.BackPWM[i];
            }
        }

        if (strncmp((char*)RxBuffer, "A", 1) == 0) {
            for(int i=0;i<4;++i){
                data[Parameter.OutID[i]] = Parameter.LeftPWM[i];
            }
        }

        if (strncmp((char*)RxBuffer, "D", 1) == 0) {
            for(int i=0;i<4;++i){
                data[Parameter.OutID[i]] = Parameter.RightPWM[i];
            }
        }

        if (strncmp((char*)RxBuffer, "E", 1) == 0) {
            for(int i=0;i<4;++i){
                data[Parameter.OutID[i]] = Parameter.ClockwisePWM[i];
            }
        }

        if (strncmp((char*)RxBuffer, "Q", 1) == 0) {
            for(int i=0;i<4;++i){
                data[Parameter.OutID[i]] = Parameter.AnticlockwisePWM[i];
            }
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

void Propeller::Handle(){
   
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, data[0]);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, data[1]);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, data[2]);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, data[3]);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, data[4]);
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, data[5]);
    
}

void Propeller_I2C::Handle(){

    TCA_SetChannel(4);
    //HAL_Delay(5);
    if(flag_PID){
        float_ctrl();//PID控制悬浮状态
        speed_ctrl();
    }
    for (int i = 0; i < PROPELLER_NUM; ++i) {
        //------TODO：i为推进器在扩展版上的接口编号，根据接线修改，目前为0-7号
        PCA_Setpwm(i, 0, floor(data[i] * 4096 / 20000 + 0.5f));
    }
}


void Propeller::data_extract(uint8_t *rx, int32_t *data, int32_t num){
    // 示例：假设 PWM 命令格式是 "PWM:1000,2000,1500,1800,1600,1400"
    if (strncmp((char*)rx, "PRO:", 4) == 0) {

        char *data_str = (char*)rx + 4;
        char *token = strtok(data_str, ",");
        int i = 0;
        while (token != NULL && i < num) {
            data[i] = atoi(token);
            token = strtok(NULL, ",");
            i++;
        }
    }
}


void Propeller_I2C::float_ctrl() {

    Component.Depth =  DepthPID.PIDCalc(Target_depth, PressureSensor::pressure_sensor.data_depth);
    Component.Roll =  RollPID.PIDCalc(0.0, PressureSensor::pressure_sensor.data_roll);
    Component.Pitch =  PitchPID.PIDCalc(0.0, PressureSensor::pressure_sensor.data_pitch);

    data[Parameter.InID[0]] = Parameter.BasePWM[0] - Component.Depth - Component.Roll - Component.Pitch;
    data[Parameter.InID[1]] = Parameter.BasePWM[1] - Component.Depth - Component.Roll + Component.Pitch;
    data[Parameter.InID[2]] = Parameter.BasePWM[2] - Component.Depth + Component.Roll - Component.Pitch;
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

void Propeller_I2C::DataCopy_V30(){
    Parameter_V30.InitPWM = InitPWM_V30;
    std::memcpy(Parameter_V30.InID, InID_V30, sizeof(InID_V30));
    std::memcpy(Parameter_V30.OutID, OutID_V30, sizeof(OutID_V30));
    std::memcpy(Parameter_V30.BasePWM, BasePWM_V30, sizeof(BasePWM_V30));
    std::memcpy(Parameter_V30.FrontPWM, FrontPWM_V30, sizeof(FrontPWM_V30));
    std::memcpy(Parameter_V30.BackPWM, BackPWM_V30, sizeof(BackPWM_V30));
    std::memcpy(Parameter_V30.LeftPWM, LeftPWM_V30, sizeof(LeftPWM_V30));
    std::memcpy(Parameter_V30.RightPWM, RightPWM_V30, sizeof(RightPWM_V30));
    std::memcpy(Parameter_V30.ClockwisePWM, ClockwisePWM_V30, sizeof(ClockwisePWM_V30));
    std::memcpy(Parameter_V30.AnticlockwisePWM, AnticlockwisePWM_V30, sizeof(AnticlockwisePWM_V30));
	std::memcpy(&Parameter_V30.DepthPID_P, &DepthPID_V30, sizeof(PID_Regulator_t));
    std::memcpy(&Parameter_V30.PitchPID_P, &PitchPID_V30, sizeof(PID_Regulator_t));
    std::memcpy(&Parameter_V30.RollPID_P, &RollPID_V30, sizeof(PID_Regulator_t));
    std::memcpy(&Parameter_V30.YawPID_P, &YawPID_V30, sizeof(PID_Regulator_t));
    std::memcpy(&Parameter_V30.VxPID_P, &VxPID_V30, sizeof(PID_Regulator_t));
    std::memcpy(&Parameter_V30.VyPID_P, &VyPID_V30, sizeof(PID_Regulator_t));

    std::memcpy(&Parameter, &Parameter_V30, sizeof(Propeller_Parameter_t));

    DepthPID.PIDInfo = Parameter.DepthPID_P;
    PitchPID.PIDInfo = Parameter.PitchPID_P;
    RollPID.PIDInfo = Parameter.RollPID_P;
    YawPID.PIDInfo = Parameter.YawPID_P;
    VxPID.PIDInfo = Parameter.VxPID_P;
    VyPID.PIDInfo = Parameter.VyPID_P;
}