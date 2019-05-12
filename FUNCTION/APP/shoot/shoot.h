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

//������俪��ͨ������
#define Shoot_RC_Channel    1
//��̨ģʽʹ�õĿ���ͨ��
#define GIMBAL_ModeChannel  1

#define SHOOT_CONTROL_TIME  GIMBAL_CONTROL_TIME

#define SHOOT_FRIC_PWM_ADD_VALUE    100.0f

//���Ħ���ּ���� �ر�
#define SHOOT_ON_KEYBOARD KEY_PRESSED_OFFSET_F
#define SHOOT_OFF_KEYBOARD KEY_PRESSED_OFFSET_R

//�����ɺ� �ӵ�����ȥ���ж�ʱ�䣬�Է��󴥷�
#define SHOOT_DONE_KEY_OFF_TIME 10
//��곤���ж�
#define PRESS_LONG_TIME 400
//ң����������ش��µ�һ��ʱ��� ���������ӵ� �����嵥
#define RC_S_LONG_TIME 2000
//Ħ���ָ��� ���� ʱ��
#define UP_ADD_TIME 80
//�����������ֵ��Χ
#define Half_ecd_range 4096
#define ecd_range 8191
//���rmp �仯�� ��ת�ٶȵı���
#define Motor_RMP_TO_SPEED 0.00290888208665721596153948461415f
#define Motor_ECD_TO_ANGLE 0.000021305288720633905968306772076277f
#define FULL_COUNT 18
//�����ٶ�
#define TRIGGER_SPEED 10.0f
#define Ready_Trigger_Speed 6.0f

#define KEY_OFF_JUGUE_TIME 500
#define SWITCH_TRIGGER_ON 0   //���ذ���
#define SWITCH_TRIGGER_OFF 1  //���ؿ���

//����ʱ�� �Լ���תʱ��
#define BLOCK_TIME 700
#define REVERSE_TIME 500
#define REVERSE_SPEED_LIMIT 13.0f

#define PI_Four 0.78539816339744830961566084581988f
#define PI_Ten 0.314f

//�����ֵ��PID
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
    volatile Encoder *encoder1;//С���貦��
    volatile Encoder *encoder2;//���貦��
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

extern volatile Encoder TR1Encoder;//С���貦��
extern volatile Encoder TR2Encoder;//���貦��

void TriggerMotor_PID_Config(void);
int16_t shoot_control_loop(void);

#endif
