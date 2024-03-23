//
// Created by admin on 2023/10/30.
//

#ifndef CONTROL_FRAME_MAIN_PWM_H
#define CONTROL_FRAME_MAIN_PWM_H

#include "Usermain.h"
#include "Extension.h"
#include "PID.h"

typedef struct Propeller_Parameter{

    int32_t InID[4];
    int32_t OutID[4];
    int32_t InitPWM;
    int32_t BasePWM[4];
    int32_t FrontPWM[4];
    int32_t BackPWM[4];
    int32_t LeftPWM[4];
    int32_t RightPWM[4];
    int32_t ClockwisePWM[4];
    int32_t AnticlockwisePWM[4];
    PID_Regulator_t DepthPID_P;
    PID_Regulator_t PitchPID_P;
    PID_Regulator_t RollPID_P;
    PID_Regulator_t YawPID_P;
    PID_Regulator_t VxPID_P;
    PID_Regulator_t VyPID_P;

}Propeller_Parameter_t;

typedef struct Propeller_Component{
    float Depth;
    float Roll,Pitch,Yaw;
    float Vx,Vy,Vz;
}Propeller_Component_t;

class Propeller: public  Device{

public:

    //uint8_t RxBuffer[SERIAL_LENGTH_MAX];
    int32_t data[PROPELLER_NUM];//PWM值，1500为不转，1500-2000正转，1000-1500反转，离1500越远转速越快


    void data_extract(uint8_t *rx, int32_t *data, int32_t num);

    void Init();
    void Handle();
    void Receive();

};

class Propeller_I2C: public  Device{

    //uint8_t RxBuffer[SERIAL_LENGTH_MAX];
    int32_t data[PROPELLER_NUM];//PWM值，1500为不转，1500-2000正转，1000-1500反转，离1500越远转速越快
    //int32_t data_receive[5];
    float Target_depth;
    float Target_angle;
    float angle_error;
    float Target_speed[3];
    bool flag_PID;
    PID DepthPID, RollPID, PitchPID;
    PID VxPID, VyPID, YawPID;

    void data_extract(uint8_t *rx);
    void float_ctrl();
    void speed_ctrl();

    void DataCopy_V30();
    float Component_Calc(float data);



    Propeller_Component_t Component;
    Propeller_Parameter_t Parameter;




public:
    void Init();
    void Handle();
    void Receive();

};


/*extern void Anglectrl_servo();

extern void Speedctrl_propeller_init();

extern void Speedctrl_propeller();

extern void Anglectrl_servo_init();
*/
#endif //CONTROL_FRAME_MAIN_PWM_H
