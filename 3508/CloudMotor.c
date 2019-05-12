#include "CloudMotor.h"
#include "can.h"
#include "Encoder.h"

#include "usart.h"

CloudMotorPID GMYaw_Speedloop;
CloudMotorPID GMYaw_Positionloop;
int16_t CloudOut[2];

void CloudMotor_Configure(void)
{
    GMYaw_Speedloop.Kp = 85.0f;
    GMYaw_Speedloop.Ki = 0.0f;
    GMYaw_Speedloop.Kd = 10.0f;
    GMYaw_Speedloop.PoutMax = 5000.0f;
    GMYaw_Speedloop.IoutMax = 0.0f;
    GMYaw_Speedloop.DoutMax = 1000.0f;
    GMYaw_Speedloop.OutMax = 7000.0f;
    GMYaw_Speedloop.Set = 0.0f;
    GMYaw_Speedloop.Real = 0.0f;
    GMYaw_Speedloop.Out = 0.0f;
    GMYaw_Speedloop.err = 0.0f;
    GMYaw_Speedloop.err_last = 0.0f;
    GMYaw_Speedloop.err_llast = 0.0f;
    GMYaw_Speedloop.integral = 0.0f;
    
    GMYaw_Positionloop.Kp = 40.0f;//80f 52
    GMYaw_Positionloop.Ki = 0.00000f;//0.00024f 0.00040
    GMYaw_Positionloop.Kd = 40.0f;//70f 21
    GMYaw_Positionloop.PoutMax = 15000.0f;
    GMYaw_Positionloop.IoutMax = 0.0f;
    GMYaw_Positionloop.DoutMax = 15000.0f;
    GMYaw_Positionloop.OutMax = 20000.0f;
    GMYaw_Positionloop.Set = 0.0f;
    GMYaw_Positionloop.Real = 0.0f;
    GMYaw_Positionloop.Out = 0.0f;
    GMYaw_Positionloop.err = 0.0f;
    GMYaw_Positionloop.err_last = 0.0f;
    GMYaw_Positionloop.err_llast = 0.0f;
    GMYaw_Positionloop.integral = 0.0f;
}
void CloudMotor_Ctrl(void)
{
    float Pout = 0.0f;
    float Iout = 0.0f;
    float Dout = 0.0f;
    //位置环
    GMYaw_Positionloop.Real = GMYawEncoder.raw_value;
    GMYaw_Positionloop.err_last = GMYaw_Positionloop.err;
    //防止在0和8191之间发生过冲，导致多转一圈或n圈
    float raw_err=GMYaw_Positionloop.Set - GMYaw_Positionloop.Real;
    if(raw_err>0&&(raw_err)>(8190-raw_err))
        GMYaw_Positionloop.err = raw_err-8190;
    else if(raw_err<0&&(-raw_err)>(8190+raw_err))
        GMYaw_Positionloop.err = 8190+raw_err;
    else
        GMYaw_Positionloop.err = raw_err;
    
    GMYaw_Positionloop.integral += GMYaw_Positionloop.err;

    Pout = GMYaw_Positionloop.Kp * GMYaw_Positionloop.err;
    Pout = Pout < GMYaw_Positionloop.PoutMax ? Pout : GMYaw_Positionloop.PoutMax;
    Pout = Pout > -GMYaw_Positionloop.PoutMax ? Pout : -GMYaw_Positionloop.PoutMax;

    Iout = GMYaw_Positionloop.Ki * GMYaw_Positionloop.integral;
    Iout = Iout < GMYaw_Positionloop.IoutMax ? Iout : GMYaw_Positionloop.IoutMax;
    Iout = Iout > -GMYaw_Positionloop.IoutMax ? Iout : -GMYaw_Positionloop.IoutMax;

    Dout = GMYaw_Positionloop.Kd * (GMYaw_Positionloop.err - GMYaw_Positionloop.err_last);
    Dout = Dout < GMYaw_Positionloop.DoutMax ? Dout : GMYaw_Positionloop.DoutMax;
    Dout = Dout > -GMYaw_Positionloop.DoutMax ? Dout : -GMYaw_Positionloop.DoutMax;

    GMYaw_Positionloop.Out = Pout + Iout + Dout;
    GMYaw_Positionloop.Out = GMYaw_Positionloop.Out < GMYaw_Positionloop.OutMax ? GMYaw_Positionloop.Out : GMYaw_Positionloop.OutMax;
    GMYaw_Positionloop.Out = GMYaw_Positionloop.Out > -GMYaw_Positionloop.OutMax ? GMYaw_Positionloop.Out : -GMYaw_Positionloop.OutMax;

    /*
    //速度环
    GMYaw_Speedloop.Set = GMYaw_Positionloop.Out;
    GMYaw_Speedloop.Real = GMYawEncoder.filter_rate;

    GMYaw_Speedloop.err_last = GMYaw_Speedloop.err;
    GMYaw_Speedloop.err = GMYaw_Speedloop.Set - GMYaw_Speedloop.Real;
    GMYaw_Speedloop.integral += GMYaw_Speedloop.err;

    Pout = GMYaw_Speedloop.Kp * GMYaw_Speedloop.err;
    Pout = Pout < GMYaw_Speedloop.PoutMax ? Pout : GMYaw_Speedloop.PoutMax;
    Pout = Pout > -GMYaw_Speedloop.PoutMax ? Pout : -GMYaw_Speedloop.PoutMax;

    Iout = GMYaw_Speedloop.Ki * GMYaw_Speedloop.integral;
    Iout = Iout < GMYaw_Speedloop.IoutMax ? Iout : GMYaw_Speedloop.IoutMax;
    Iout = Iout > -GMYaw_Speedloop.IoutMax ? Iout : -GMYaw_Speedloop.IoutMax;

    Dout = GMYaw_Speedloop.Kd * (GMYaw_Speedloop.err - GMYaw_Speedloop.err_last);
    Dout = Dout < GMYaw_Speedloop.DoutMax ? Dout : GMYaw_Speedloop.DoutMax;
    Dout = Dout > -GMYaw_Speedloop.DoutMax ? Dout : -GMYaw_Speedloop.DoutMax;

    GMYaw_Speedloop.Out = Pout + Iout + Dout;
    GMYaw_Speedloop.Out = GMYaw_Speedloop.Out < GMYaw_Speedloop.OutMax ? GMYaw_Speedloop.Out : GMYaw_Speedloop.OutMax;
    GMYaw_Speedloop.Out = GMYaw_Speedloop.Out > -GMYaw_Speedloop.OutMax ? GMYaw_Speedloop.Out : -GMYaw_Speedloop.OutMax;
    */

}
int16_t* CloudMotor_Out(void)//将任务延后到发射任务时发出
{
    CloudOut[0] = GMYaw_Positionloop.Out;
    CloudOut[1] = 0;
    return CloudOut;
}
