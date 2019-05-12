/**
  ******************************************************************************
  * 文件名：  can底层驱动文件头文件
  * 作者：    SLDX->wangX
  * 版本：    1.0.0
  * 日期：    2018/7/2
  * 函数列表：can配置函数
  *	          can接收函数
  *			  can接收中断
  ******************************************************************************
  */

#ifndef _CAN_
#define _CAN_
#include "sys.h"

//地盘四个电机的can id
#define CAN_ID_CM1 0x201
#define CAN_ID_CM2 0x202
#define CAN_ID_CM3 0x203
#define CAN_ID_CM4 0x204

//云台三个电机的can id
#define CAN_ID_YAW 0x205
#define CAN_ID_PITCH 0x206

//拨轮电机can id
#define CAN_ID_TRIGGER_17 0x207
#define CAN_ID_TRIGGER_42 0x208

void CAN_Configure(void);
void CanReceiveMsgProcess(CanRxMsg *message);
void Set_CloudMotor_Current(int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq,int16_t trigger_iq);
void Set_ChassisMotor_Current(int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);

#endif
