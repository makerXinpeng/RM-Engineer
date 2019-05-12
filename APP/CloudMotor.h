#ifndef CLOUDMOTOR_H
#define CLOUDMOTOR_H

#include "main.h"

#include "arm_math.h"
#include "user_lib.h"
#include "can.h"
#include "Encoder.h"
#include "pid.h"
#include "remote_control.h"
#include "shoot.h"
//pitch 速度环 PID参数以及 PID最大输出，积分输出
#define PITCH_SPEED_PID_KP 2000.0f//600.0f//英雄2000.0f
#define PITCH_SPEED_PID_KI 20.0f
#define PITCH_SPEED_PID_KD 0.0f
#define PITCH_SPEED_PID_MAX_OUT 30000.0f
#define PITCH_SPEED_PID_MAX_IOUT 5000.0f

//yaw 速度环 PID参数以及 PID最大输出，积分输出
#define YAW_SPEED_PID_KP 2500.0f//2200.0f
#define YAW_SPEED_PID_KI 20.0f
#define YAW_SPEED_PID_KD 0.0f
#define YAW_SPEED_PID_MAX_OUT 30000.0f
#define YAW_SPEED_PID_MAX_IOUT 5000.0f

//pitch 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define PITCH_GYRO_ABSOLUTE_PID_KP 15.0f
#define PITCH_GYRO_ABSOLUTE_PID_KI 0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD 0.0f

#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f

//yaw 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define YAW_GYRO_ABSOLUTE_PID_KP 10.0f
#define YAW_GYRO_ABSOLUTE_PID_KI 0.0f
#define YAW_GYRO_ABSOLUTE_PID_KD 0.3f
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f

//pitch 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define PITCH_ENCODE_RELATIVE_PID_KP 30.0f
#define PITCH_ENCODE_RELATIVE_PID_KI 0.0f
#define PITCH_ENCODE_RELATIVE_PID_KD 10.0f

#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 10.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

//yaw 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define YAW_ENCODE_RELATIVE_PID_KP 30.0//8.0f
#define YAW_ENCODE_RELATIVE_PID_KI 0.0f
#define YAW_ENCODE_RELATIVE_PID_KD 10.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT 18.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

//任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 201
//yaw,pitch控制通道以及状态开关通道
#define YawChannel 2
#define PitchChannel 3
#define ModeChannel 0
//掉头180 按键
#define TurnKeyBoard KEY_PRESSED_OFFSET_F
//掉头云台速度
#define TurnSpeed 0.04f
//测试按键尚未使用
#define TestKeyBoard KEY_PRESSED_OFFSET_R
//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_deadband 10
//yaw，pitch角度与遥控器输入比例
#define Yaw_RC_SEN -0.00005f//-0.000005f
#define Pitch_RC_SEN -0.00006f//-0.000006f //0.005
//yaw,pitch角度和鼠标输入的比例
#define Yaw_Mouse_Sen 0.0005f//0.00005f
#define Pitch_Mouse_Sen 0.0015f//0.00015f
//云台编码器控制时候使用的比例
#define Yaw_Encoder_Sen 0.01f
#define Pitch_Encoder_Sen 0.01f
//云台控制周期
#define GIMBAL_CONTROL_TIME 1

//云台测试模式 宏定义 0 为不使用测试模式
#define GIMBAL_TEST_MODE 0

//电机是否反装
#define PITCH_TURN 0
#define YAW_TURN 0

//电机码盘值最大以及中值
#define Half_ecd_range 4096
#define ecd_range 8191
//云台初始化回中值，允许的误差,并且在误差范围内停止一段时间以及最大时间6s后解除初始化状态，
#define GIMBAL_INIT_ANGLE_ERROR 0.1f
#define GIMBAL_INIT_STOP_TIME 100
#define GIMBAL_INIT_TIME 6000
//云台初始化回中值的速度以及控制到的角度
#define GIMBAL_INIT_PITCH_SPEED 0.004f
#define GIMBAL_INIT_YAW_SPEED   0.005f
#define INIT_YAW_SET 0.0f
#define INIT_PITCH_SET 0.0f

//云台校准中值的时候，发送原始电流值，以及堵转时间，通过陀螺仪判断堵转
#define GIMBAL_CALI_MOTOR_SET 8000
#define GIMBAL_CALI_STEP_TIME 2000
#define GIMBAL_CALI_GYRO_LIMIT 0.1f

#define GIMBAL_CALI_PITCH_MAX_STEP 1
#define GIMBAL_CALI_PITCH_MIN_STEP 2
#define GIMBAL_CALI_YAW_MAX_STEP 3
#define GIMBAL_CALI_YAW_MIN_STEP 4

#define GIMBAL_CALI_START_STEP GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP 5

//判断遥控器无输入的时间以及遥控器无输入判断，设置云台yaw回中值以防陀螺仪漂移
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX 3000

//将电机rpm值转换为角速度值 角速度 = rpm * 2pi / 60
#define Motor_rpm_to_angular_velocity 0.104719755120f

//电机编码值转化成角度值
#ifndef Motor_Ecd_to_Rad
#define Motor_Ecd_to_Rad 0.000766990394f //      2*  PI  /8192
#endif

typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //电机原始值控制
    GIMBAL_MOTOR_GYRO,    //电机陀螺仪角度控制
    GIMBAL_MOTOR_ENCONDE, //电机编码值角度控制
} gimbal_motor_mode_e;

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} Gimbal_PID_t;

typedef struct
{
    //const motor_measure_t *gimbal_motor_measure;
    volatile Encoder *Gimbal_Encoder;
    Gimbal_PID_t gimbal_motor_absolute_angle_pid;   
    Gimbal_PID_t gimbal_motor_relative_angle_pid;   
    PidTypeDef gimbal_motor_position_pid;            //电机位置环PID
    PidTypeDef gimbal_motor_speed_pid;               //电机速度环PID
    PidTypeDef gimbal_motor_gyro_pid;
    gimbal_motor_mode_e gimbal_motor_mode;
    gimbal_motor_mode_e last_gimbal_motor_mode;
    uint16_t offset_ecd;
    fp32 max_relative_angle; //rad
    fp32 min_relative_angle; //rad

    fp32 relative_angle;     //rad
    fp32 relative_angle_set; //rad
    fp32 absolute_angle;     //rad
    fp32 absolute_angle_set; //rad
    fp32 motor_gyro;         //rad/s
    fp32 motor_gyro_set;
    fp32 motor_speed;
    fp32 raw_cmd_current;
    fp32 current_set;
    int16_t given_current;

} Gimbal_Motor_t;

typedef struct
{
    fp32 max_yaw;
    fp32 min_yaw;
    fp32 max_pitch;
    fp32 min_pitch;
    uint16_t max_yaw_ecd;
    uint16_t min_yaw_ecd;
    uint16_t max_pitch_ecd;
    uint16_t min_pitch_ecd;
    uint8_t step;
} Gimbal_Cali_t;

typedef struct
{
    const RC_ctrl_t *gimbal_rc_ctrl;
    const fp32 *gimbal_INT_angle_point;
    const fp32 *gimbal_INT_gyro_point;
    Gimbal_Motor_t gimbal_yaw_motor;
    Gimbal_Motor_t gimbal_pitch_motor;
    Gimbal_Cali_t gimbal_cali;
} Gimbal_Control_t;

extern Gimbal_Control_t gimbal_control;
extern volatile Encoder GMYawEncoder;
extern volatile Encoder GMPitchEncoder;

void CloudMotor_Config(void);
void CloudMotor_Ctrl(void);
static fp32 GIMBAL_PID_Calc(PidTypeDef *pid, fp32 ref, fp32 set);
#endif //CLOUDMOTOR_H
