/*********************************************************************************
  *Copyright(C),SLDX->Ambition
  *文件名:   地盘电机 头文件
  *作者:     SLDX->wangX
  *版本:     1.0.0
  *创建日期: 2018年7月3日
  *说明:
			本文件的函数依赖于CAN头文件和Encoder头文件，主要利用其获取和发送数据
			到电机。本文件主要通过定时调用ChassisMotor_Ctrl函数进行pid解算，通过
			调用ChassisMotor_Out函数进行实际输出。ChassisMotor_Configure进行初始
			化电机各项参数

  *其他:  //其他内容说明
  *主要函数列表:
     1.ChassisMotor_Configur
     2.ChassisMotor_Ctrl
	 3.ChassisMotor_Out
  *修改历史:  //修改历史记录列表，每条修改记录应包含修改日期、修改者及修改内容简介
     1.Date:
       Author:
       Modification:
**********************************************************************************/

#ifndef _CHASSIS_
#define _CHASSIS_


typedef struct ChassisMotorPID
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
} ChassisMotorPID;

extern ChassisMotorPID CM_LF_Speedloop;
extern ChassisMotorPID CM_RF_Speedloop;
extern ChassisMotorPID CM_LB_Speedloop;
extern ChassisMotorPID CM_RB_Speedloop;

void ChassisMotor_Configure(void);
void ChassisMotor_Ctrl(void);
void ChassisMotor_Out(void);

#endif
