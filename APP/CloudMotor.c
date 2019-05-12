#include "CloudMotor.h"

//电机编码值规整 0—8191
#define ECD_Format(ecd)         \
    {                           \
        if ((ecd) > ecd_range)  \
            (ecd) -= ecd_range; \
        else if ((ecd) < 0)     \
            (ecd) += ecd_range; \
    }

#define int_abs(x) ((x) > 0 ? (x) : (-x))
/**
  * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定是发送1024过来，
  * @author         RM
  * @param[in]      输入的遥控器值
  * @param[in]      输出的死区处理后遥控器值
  * @param[in]      死区值
  * @retval         返回空
  */
#define rc_deadline_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }
    
int16_t CloudOut[2];

//云台控制所有相关数据
static Gimbal_Control_t gimbal_control;

//发送的can 指令
static int16_t Yaw_Can_Set_Current = 0, Pitch_Can_Set_Current = 0, Shoot_Can_Set_Current = 0;
    
//云台数据更新
static void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update);
//对数据进行限幅
static void GIMBAL_relative_angle_limit(Gimbal_Motor_t *gimbal_motor, fp32 add);
//遥控数据转换为云台改变角度
void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, Gimbal_Control_t *gimbal_control_set);
//得到遥控数据并对其进行限幅
static void GIMBAL_Set_Contorl(Gimbal_Control_t *gimbal_set_control);
//计算云台电机相对中值的相对角度
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
//云台编码器控制
static void gimbal_motor_relative_angle_control(Gimbal_Motor_t *gimbal_motor);

void CloudMotor_Config(void)
{
    GIMBAL_Feedback_Update(&gimbal_control);
    
    static const fp32 YAW_SpeedPID[3]= {YAW_SPEED_PID_KP,YAW_SPEED_PID_KI,YAW_SPEED_PID_KD};
    static const fp32 YAW_PositionPID[3]= {YAW_ENCODE_RELATIVE_PID_KP,YAW_ENCODE_RELATIVE_PID_KI,YAW_ENCODE_RELATIVE_PID_KD};
    static const fp32 PITCH_SpeedPID[3]= {PITCH_SPEED_PID_KP,PITCH_SPEED_PID_KI,PITCH_SPEED_PID_KD};
    static const fp32 PITCH_PositionPID[3]= {PITCH_ENCODE_RELATIVE_PID_KP,PITCH_ENCODE_RELATIVE_PID_KI,PITCH_ENCODE_RELATIVE_PID_KD};
    
    //遥控器数据指针获取
    gimbal_control.gimbal_rc_ctrl = get_remote_control_point();
    
    //获取编码器数据
    gimbal_control.gimbal_yaw_motor.Gimbal_Encoder = &GMYawEncoder;
    gimbal_control.gimbal_pitch_motor.Gimbal_Encoder = &GMPitchEncoder;
    
    //yaw电机PID初始化
    PID_Init(&gimbal_control.gimbal_yaw_motor.gimbal_motor_speed_pid,PID_POSITION,YAW_SpeedPID,YAW_SPEED_PID_MAX_OUT,YAW_SPEED_PID_MAX_IOUT);
    PID_Init(&gimbal_control.gimbal_yaw_motor.gimbal_motor_position_pid,PID_POSITION,YAW_PositionPID,YAW_ENCODE_RELATIVE_PID_MAX_OUT,YAW_ENCODE_RELATIVE_PID_MAX_IOUT);    
    //yaw电机限幅
    gimbal_control.gimbal_yaw_motor.max_relative_angle = 0.6;
    gimbal_control.gimbal_yaw_motor.min_relative_angle = -0.6;
    gimbal_control.gimbal_yaw_motor.offset_ecd = 4096;
    //yaw电机数据初始化
    gimbal_control.gimbal_yaw_motor.relative_angle_set = 0;
    gimbal_control.gimbal_yaw_motor.motor_gyro_set = gimbal_control.gimbal_yaw_motor.motor_gyro;

    
    //pitch电机PID初始化
    PID_Init(&gimbal_control.gimbal_pitch_motor.gimbal_motor_speed_pid,PID_POSITION,PITCH_SpeedPID,PITCH_SPEED_PID_MAX_OUT,PITCH_SPEED_PID_MAX_IOUT);
    PID_Init(&gimbal_control.gimbal_pitch_motor.gimbal_motor_position_pid,PID_POSITION,PITCH_PositionPID,PITCH_ENCODE_RELATIVE_PID_MAX_OUT,PITCH_ENCODE_RELATIVE_PID_MAX_IOUT);
    //pitch电机限幅
    gimbal_control.gimbal_pitch_motor.max_relative_angle = 0.6;
    gimbal_control.gimbal_pitch_motor.min_relative_angle = -0.6;
    gimbal_control.gimbal_pitch_motor.offset_ecd = 4096;//步兵1200 英雄 4096
    //pitch电机数据初始化
    gimbal_control.gimbal_pitch_motor.relative_angle_set = 0;
    gimbal_control.gimbal_pitch_motor.motor_gyro_set = gimbal_control.gimbal_pitch_motor.motor_gyro;

}

static void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update)
{
    if (gimbal_feedback_update == NULL)
    {
        return;
    }
    //云台数据更新
    gimbal_feedback_update->gimbal_pitch_motor.relative_angle = motor_ecd_to_angle_change(gimbal_control.gimbal_pitch_motor.offset_ecd,gimbal_feedback_update->gimbal_pitch_motor.Gimbal_Encoder->raw_value);
    gimbal_feedback_update->gimbal_pitch_motor.motor_gyro = gimbal_feedback_update->gimbal_pitch_motor.Gimbal_Encoder->rpm * Motor_rpm_to_angular_velocity;

    gimbal_feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(gimbal_control.gimbal_yaw_motor.offset_ecd,gimbal_feedback_update->gimbal_yaw_motor.Gimbal_Encoder->raw_value);
    gimbal_feedback_update->gimbal_yaw_motor.motor_gyro = gimbal_feedback_update->gimbal_yaw_motor.Gimbal_Encoder->rpm * Motor_rpm_to_angular_velocity;
}
//计算相对角度
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > Half_ecd_range)
    {
        relative_ecd -= ecd_range;
    }
    else if (relative_ecd < -Half_ecd_range)
    {
        relative_ecd += ecd_range;
    }

    return relative_ecd * Motor_Ecd_to_Rad;
}

const Gimbal_Motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

const Gimbal_Motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}


/**
  * @brief          云台行为控制，根据不同行为采用不同控制函数
  * @author         RM
  * @param[in]      设置的yaw角度增加值，单位 rad
  * @param[in]      设置的pitch角度增加值，单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, Gimbal_Control_t *gimbal_control_set)
{

    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    static fp32 rc_add_yaw, rc_add_pit;
    static int16_t yaw_channel = 0, pitch_channel = 0;

    //将遥控器的数据处理死区 int16_t yaw_channel,pitch_channel
    rc_deadline_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[4], yaw_channel, RC_deadband);
    rc_deadline_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[1], pitch_channel, RC_deadband);

    rc_add_yaw = yaw_channel * Yaw_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.x * Yaw_Mouse_Sen;
    rc_add_pit = pitch_channel * Pitch_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.y * Pitch_Mouse_Sen;

    //将控制增加量赋值
    *add_yaw = rc_add_yaw;
    *add_pitch = rc_add_pit;
}
//对数据进行限幅
static void GIMBAL_relative_angle_limit(Gimbal_Motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->relative_angle_set += add;
    //是否超过最大 最小值
    if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
    }
    else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
    }
}
static void GIMBAL_Set_Contorl(Gimbal_Control_t *gimbal_set_control)
{
    if (gimbal_set_control == NULL)
    {
        return;
    }
    
    fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;
    
    gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, gimbal_set_control);
    //enconde模式下，电机编码角度控制
    GIMBAL_relative_angle_limit(&gimbal_set_control->gimbal_yaw_motor, add_yaw_angle);
    GIMBAL_relative_angle_limit(&gimbal_set_control->gimbal_pitch_motor, add_pitch_angle);
}
void CloudMotor_Ctrl(void)
{
    //云台电机数据更新
    GIMBAL_Feedback_Update(&gimbal_control);
    GIMBAL_Set_Contorl(&gimbal_control);
    
    //gimbal_control.gimbal_yaw_motor.relative_angle_set = gimbal_control.gimbal_rc_ctrl->rc.ch[4]*PI/660;
    //gimbal_control.gimbal_pitch_motor.relative_angle_set = gimbal_control.gimbal_rc_ctrl->rc.ch[1]*PI/660;
    
    gimbal_motor_relative_angle_control(&gimbal_control.gimbal_yaw_motor);//yaw
    gimbal_motor_relative_angle_control(&gimbal_control.gimbal_pitch_motor);//pitch
    
    Yaw_Can_Set_Current = gimbal_control.gimbal_yaw_motor.gimbal_motor_speed_pid.out;
    Pitch_Can_Set_Current = gimbal_control.gimbal_pitch_motor.gimbal_motor_speed_pid.out;
    Shoot_Can_Set_Current = shoot_control_loop();
    
    Set_CloudMotor_Current(Yaw_Can_Set_Current,Pitch_Can_Set_Current,Shoot_Can_Set_Current);
}

static void gimbal_motor_relative_angle_control(Gimbal_Motor_t *gimbal_motor)
{
    gimbal_motor->motor_gyro_set = GIMBAL_PID_Calc(&gimbal_motor->gimbal_motor_position_pid,gimbal_motor->relative_angle,gimbal_motor->relative_angle_set);
    gimbal_motor->gimbal_motor_speed_pid.out = PID_Calc(&gimbal_motor->gimbal_motor_speed_pid,gimbal_motor->motor_gyro,gimbal_motor->motor_gyro_set);
}

static fp32 GIMBAL_PID_Calc(PidTypeDef *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    
    //防止云台疯转
    float raw_err=pid->error[0];
    if(raw_err>0&&(raw_err)>(2*PI-raw_err))
        pid->error[0] = raw_err-2*PI;
    else if(raw_err<0&&(-raw_err)>(2*PI+raw_err))
        pid->error[0] = 2*PI+raw_err;
    else
        pid->error[0] = raw_err;
    
    pid->error[0] = - pid->error[0];
    
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->Iout = pid->Iout < pid->max_iout ? pid->Iout : pid->max_iout;
        pid->Iout = pid->Iout > -pid->max_iout ? pid->Iout : -pid->max_iout;
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        pid->out = pid->out < pid->max_out ? pid->out : pid->max_out;
        pid->out = pid->out > -pid->max_out ? pid->out : -pid->max_out;
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        pid->out = pid->out < pid->max_out ? pid->out : pid->max_out;
        pid->out = pid->out > -pid->max_out ? pid->out : -pid->max_out;
    }
    return pid->out;
}
