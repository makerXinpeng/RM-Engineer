#ifndef SHOOT_H
#define SHOOT_H
#include "can.h"
#include "Encoder.h"
#include "CloudMotor.h"
#include "remote_control.h"
#include "user_lib.h"
#include "pid.h"
#include "laser.h"
#include "fric.h"
#include "arm_math.h"
#include "usart.h"

//射击发射开关通道数据
#define Shoot_RC_Channel    1
//云台模式使用的开关通道
#define GIMBAL_ModeChannel  1

#define SHOOT_CONTROL_TIME  GIMBAL_CONTROL_TIME

#define SHOOT_FRIC_PWM_ADD_VALUE    100.0f

//射击摩擦轮激光打开 关闭
#define SHOOT_ON_KEYBOARD KEY_PRESSED_OFFSET_F
#define SHOOT_OFF_KEYBOARD KEY_PRESSED_OFFSET_R

//射击完成后 子弹弹出去后，判断时间，以防误触发
#define SHOOT_DONE_KEY_OFF_TIME 10
//鼠标长按判断
#define PRESS_LONG_TIME 400
//遥控器射击开关打下档一段时间后 连续发射子弹 用于清单
#define RC_S_LONG_TIME 2000
//摩擦轮高速 加速 时间
#define UP_ADD_TIME 80
//电机反馈码盘值范围
#define Half_ecd_range 4096
#define ecd_range 8191
//电机rmp 变化成 旋转速度的比例
#define Motor_RMP_TO_SPEED 0.00290888208665721596153948461415f
#define Motor_ECD_TO_ANGLE 0.000021305288720633905968306772076277f
#define FULL_COUNT 18
//拨弹速度
#define TRIGGER_SPEED 10.0f
#define Ready_Trigger_Speed 6.0f

#define KEY_OFF_JUGUE_TIME 500
#define SWITCH_TRIGGER_ON 0   //开关按下
#define SWITCH_TRIGGER_OFF 1  //开关开启

//卡单时间 以及反转时间
#define BLOCK_TIME 700
#define REVERSE_TIME 500
#define REVERSE_SPEED_LIMIT 13.0f

#define PI_Four 0.78539816339744830961566084581988f
#define PI_Ten 0.314f

//拨弹轮电机PID
#define TRIGGER_ANGLE_PID_KP 500.0f
#define TRIGGER_ANGLE_PID_KI 0.5f
#define TRIGGER_ANGLE_PID_KD 30.0f

#define TRIGGER_BULLET_PID_MAX_OUT 15000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 5000.0f

#define TRIGGER_READY_PID_MAX_OUT 5000.0f
#define TRIGGER_READY_PID_MAX_IOUT 2500.0f

typedef struct
{
    ramp_function_source_t fric1_ramp;
    ramp_function_source_t fric2_ramp;
    //const motor_measure_t *shoot_motor_measure;
    volatile Encoder *encoder1;//小弹丸拨轮
    volatile Encoder *encoder2;//大弹丸拨轮
    fp32 speed;
    fp32 speed_set;
    fp32 angle;
    fp32 set_angle;
    int16_t given_current;
    int8_t ecd_count;

    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    uint16_t press_l_time;
    uint16_t press_r_time;
    uint16_t rc_s_time;

    bool_t move_flag;
    uint32_t cmd_time;
    uint32_t run_time;
    bool_t key;
    uint16_t key_time;
    bool_t shoot_done;
    uint8_t shoot_done_time;
    int16_t BulletShootCnt;
    int16_t last_butter_count;
} Shoot_Motor_t;

typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY,
    SHOOT_BULLET,
    SHOOT_DONE,
} shoot_mode_e;

extern volatile Encoder TR1Encoder;//小弹丸拨轮
extern volatile Encoder TR2Encoder;//大弹丸拨轮

void TriggerMotor_PID_Config(void);
int16_t shoot_control_loop(void);

#endif
