#ifndef CLOUD_MOTER
#define CLOUD_MOTER
#include "can.h"
#include "Encoder.h"

typedef struct CloudMotorPID
{
    float Kp,Ki,Kd;    //定义
    float PoutMax,IoutMax,DoutMax;
    float OutMax;

    float Set;    //定义设定值
    float Real; //编码器采样值
    float Out; //输出值

    float err;         //定义偏差值
    float err_last; //上一次偏差值
    float err_llast; //最上次偏差值
    float integral; //误差累计
} CloudMotorPID;

extern CloudMotorPID GMYaw_Speedloop;
extern CloudMotorPID GMYaw_Positionloop;
extern volatile Encoder GMYawEncoder;

void CloudMotor_Configure(void);
void CloudMotor_Ctrl(void);
int16_t* CloudMotor_Out(void);

#endif
