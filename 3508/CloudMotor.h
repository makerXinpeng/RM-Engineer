#ifndef CLOUD_MOTER
#define CLOUD_MOTER
#include "can.h"
#include "Encoder.h"

typedef struct CloudMotorPID
{
    float Kp,Ki,Kd;    //����
    float PoutMax,IoutMax,DoutMax;
    float OutMax;

    float Set;    //�����趨ֵ
    float Real; //����������ֵ
    float Out; //���ֵ

    float err;         //����ƫ��ֵ
    float err_last; //��һ��ƫ��ֵ
    float err_llast; //���ϴ�ƫ��ֵ
    float integral; //����ۼ�
} CloudMotorPID;

extern CloudMotorPID GMYaw_Speedloop;
extern CloudMotorPID GMYaw_Positionloop;
extern volatile Encoder GMYawEncoder;

void CloudMotor_Configure(void);
void CloudMotor_Ctrl(void);
int16_t* CloudMotor_Out(void);

#endif
