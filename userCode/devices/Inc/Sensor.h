//
// Created by admin on 2023/11/27.
//
#include "usermain.h"
#include "Matrix.h"
#include "Extension.h"
#include "MahonyAHRS.h"

#ifndef CONTROL_FRAME_MAIN_SENSOR_H
#define CONTROL_FRAME_MAIN_SENSOR_H



#define B02_IIC_ADDRESS 0xEC

#define MS5837_30BA_ResetCommand     0x1E                //��λ
#define	MS5837_30BA_PROM_RD 	       0xA0                //PROM��ȡ,{0XA0,0XA2,0XA4,0XA8,0XAA,0XAC,0XAE}
#define MS5837_30BA_ADC_RD           0x00                //ADC��ȡ

#define MS5837_30BA_D1_OSR256					 0x40
#define MS5837_30BA_D1_OSR512					 0x42
#define MS5837_30BA_D1_OSR1024					 0x44
#define MS5837_30BA_D1_OSR2048					 0x46
#define MS5837_30BA_D1_OSR4096					 0x48
#define MS5837_30BA_D2_OSR256					 0x50
#define MS5837_30BA_D2_OSR512					 0x52
#define MS5837_30BA_D2_OSR1024					 0x54
#define MS5837_30BA_D2_OSR2048					 0x56
#define MS5837_30BA_D2_OSR4096					 0x58


typedef struct Sensor_Site{
    float x[SENSOR_NUM];
    float y[SENSOR_NUM];
    float z[SENSOR_NUM];
}Sensor_Site_t;

class PressureSensor: public  Device{

    unsigned char MS5837_30BA_Crc4(int id);
    unsigned long MS5837_30BA_GetConversion(uint8_t command);
    uint8_t MS5837_30BA_PROM(int id);
    void MS5837_30BA_ReSet(void);
    float MS5837_30BA_GetData(int id);
    //signed int MS5837_30BA_GetTemp(int id);

    void Init_single(int id);
    void Handle_single(int id);
    void Calibrate_single(int id);
    void Calibrate();

    void Solve_plane_3(float* data,float* h,float* x,float* y,float* z);
    void Solve_plane(float* data,float* h,float* x,float* y,float* z,int num);
    void Update_plane();//计算水面方程

    void Delay_us(uint32_t us);

    //void get_angle(float q[4], float *yaw, float *pitch, float *roll);

    signed int dT,TEMP;
    uint32_t Cal_C[6][7];
    int32_t OFFi,SENSi,Ti;
    int64_t OFF2;
    int64_t SENS2;
    uint32_t TEMP2;
    int64_t OFF_,SENS;
    uint32_t D1_Pres,D2_Temp;
    unsigned char flag_ok[6];
    
    int32_t CurrentID;


    Sensor_Site_t site;

public:


    void Init();
    void Handle();
    void Receive();

    float Temperature;
    signed int data_temp[6];
    float data_pressure_offset[6];//压强零偏值，在水上测量
    float data_pressure_raw[6];//水下压强原始值
    float data_pressure[6];//水下压强去掉零偏，即为深度值
    float data_plane[5];//水面方程Ax+By+Cz+D=0,A*A+B*B+C*C=0,5个数据分别为A、B、C、D、拉格朗日乘数
    float data_depth;//深度数据，单位cm
    float data_roll,data_pitch;
    float data_level[3];
    //float quat[4];
    float pitch,roll;//俯仰角、横滚角
    bool flag_Calibrate;

    //uint8_t flag_Busy;

    static PressureSensor pressure_sensor;


};

class Sonar: public Device{

    //uint8_t RxBuffer[SERIAL_LENGTH_MAX];
    int32_t data;
    int32_t data_offset;

public:

    void Init();
    void Handle();
    void Receive();
    static Sonar sonar;

};




#endif //CONTROL_FRAME_MAIN_SENSOR_H
