#include "CloudMotor.h"

//�������ֵ���� 0��8191
#define ECD_Format(ecd)         \
    {                           \
        if ((ecd) > ecd_range)  \
            (ecd) -= ecd_range; \
        else if ((ecd) < 0)     \
            (ecd) += ecd_range; \
    }

#define int_abs(x) ((x) > 0 ? (x) : (-x))
/**
  * @brief          ң�����������жϣ���Ϊң�����Ĳ�������λ��ʱ�򣬲�һ���Ƿ���1024������
  * @author         RM
  * @param[in]      �����ң����ֵ
  * @param[in]      ��������������ң����ֵ
  * @param[in]      ����ֵ
  * @retval         ���ؿ�
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

//��̨���������������
static Gimbal_Control_t gimbal_control;

//���͵�can ָ��
static int16_t Yaw_Can_Set_Current = 0, Pitch_Can_Set_Current = 0, Shoot_Can_Set_Current = 0;
    
//��̨���ݸ���
static void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update);
//�����ݽ����޷�
static void GIMBAL_relative_angle_limit(Gimbal_Motor_t *gimbal_motor, fp32 add);
//ң������ת��Ϊ��̨�ı�Ƕ�
void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, Gimbal_Control_t *gimbal_control_set);
//�õ�ң�����ݲ���������޷�
static void GIMBAL_Set_Contorl(Gimbal_Control_t *gimbal_set_control);
//������̨��������ֵ����ԽǶ�
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
//��̨����������
static void gimbal_motor_relative_angle_control(Gimbal_Motor_t *gimbal_motor);

void CloudMotor_Config(void)
{
    GIMBAL_Feedback_Update(&gimbal_control);
    
    static const fp32 YAW_SpeedPID[3]= {YAW_SPEED_PID_KP,YAW_SPEED_PID_KI,YAW_SPEED_PID_KD};
    static const fp32 YAW_PositionPID[3]= {YAW_ENCODE_RELATIVE_PID_KP,YAW_ENCODE_RELATIVE_PID_KI,YAW_ENCODE_RELATIVE_PID_KD};
    static const fp32 PITCH_SpeedPID[3]= {PITCH_SPEED_PID_KP,PITCH_SPEED_PID_KI,PITCH_SPEED_PID_KD};
    static const fp32 PITCH_PositionPID[3]= {PITCH_ENCODE_RELATIVE_PID_KP,PITCH_ENCODE_RELATIVE_PID_KI,PITCH_ENCODE_RELATIVE_PID_KD};
    
    //ң��������ָ���ȡ
    gimbal_control.gimbal_rc_ctrl = get_remote_control_point();
    
    //��ȡ����������
    gimbal_control.gimbal_yaw_motor.Gimbal_Encoder = &GMYawEncoder;
    gimbal_control.gimbal_pitch_motor.Gimbal_Encoder = &GMPitchEncoder;
    
    //yaw���PID��ʼ��
    PID_Init(&gimbal_control.gimbal_yaw_motor.gimbal_motor_speed_pid,PID_POSITION,YAW_SpeedPID,YAW_SPEED_PID_MAX_OUT,YAW_SPEED_PID_MAX_IOUT);
    PID_Init(&gimbal_control.gimbal_yaw_motor.gimbal_motor_position_pid,PID_POSITION,YAW_PositionPID,YAW_ENCODE_RELATIVE_PID_MAX_OUT,YAW_ENCODE_RELATIVE_PID_MAX_IOUT);    
    //yaw����޷�
    gimbal_control.gimbal_yaw_motor.max_relative_angle = 0.6;
    gimbal_control.gimbal_yaw_motor.min_relative_angle = -0.6;
    gimbal_control.gimbal_yaw_motor.offset_ecd = 4096;
    //yaw������ݳ�ʼ��
    gimbal_control.gimbal_yaw_motor.relative_angle_set = 0;
    gimbal_control.gimbal_yaw_motor.motor_gyro_set = gimbal_control.gimbal_yaw_motor.motor_gyro;

    
    //pitch���PID��ʼ��
    PID_Init(&gimbal_control.gimbal_pitch_motor.gimbal_motor_speed_pid,PID_POSITION,PITCH_SpeedPID,PITCH_SPEED_PID_MAX_OUT,PITCH_SPEED_PID_MAX_IOUT);
    PID_Init(&gimbal_control.gimbal_pitch_motor.gimbal_motor_position_pid,PID_POSITION,PITCH_PositionPID,PITCH_ENCODE_RELATIVE_PID_MAX_OUT,PITCH_ENCODE_RELATIVE_PID_MAX_IOUT);
    //pitch����޷�
    gimbal_control.gimbal_pitch_motor.max_relative_angle = 0.6;
    gimbal_control.gimbal_pitch_motor.min_relative_angle = -0.6;
    gimbal_control.gimbal_pitch_motor.offset_ecd = 4096;//����1200 Ӣ�� 4096
    //pitch������ݳ�ʼ��
    gimbal_control.gimbal_pitch_motor.relative_angle_set = 0;
    gimbal_control.gimbal_pitch_motor.motor_gyro_set = gimbal_control.gimbal_pitch_motor.motor_gyro;

}

static void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update)
{
    if (gimbal_feedback_update == NULL)
    {
        return;
    }
    //��̨���ݸ���
    gimbal_feedback_update->gimbal_pitch_motor.relative_angle = motor_ecd_to_angle_change(gimbal_control.gimbal_pitch_motor.offset_ecd,gimbal_feedback_update->gimbal_pitch_motor.Gimbal_Encoder->raw_value);
    gimbal_feedback_update->gimbal_pitch_motor.motor_gyro = gimbal_feedback_update->gimbal_pitch_motor.Gimbal_Encoder->rpm * Motor_rpm_to_angular_velocity;

    gimbal_feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(gimbal_control.gimbal_yaw_motor.offset_ecd,gimbal_feedback_update->gimbal_yaw_motor.Gimbal_Encoder->raw_value);
    gimbal_feedback_update->gimbal_yaw_motor.motor_gyro = gimbal_feedback_update->gimbal_yaw_motor.Gimbal_Encoder->rpm * Motor_rpm_to_angular_velocity;
}
//������ԽǶ�
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
  * @brief          ��̨��Ϊ���ƣ����ݲ�ͬ��Ϊ���ò�ͬ���ƺ���
  * @author         RM
  * @param[in]      ���õ�yaw�Ƕ�����ֵ����λ rad
  * @param[in]      ���õ�pitch�Ƕ�����ֵ����λ rad
  * @param[in]      ��̨����ָ��
  * @retval         ���ؿ�
  */
void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, Gimbal_Control_t *gimbal_control_set)
{

    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    static fp32 rc_add_yaw, rc_add_pit;
    static int16_t yaw_channel = 0, pitch_channel = 0;

    //��ң���������ݴ������� int16_t yaw_channel,pitch_channel
    rc_deadline_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[4], yaw_channel, RC_deadband);
    rc_deadline_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[1], pitch_channel, RC_deadband);

    rc_add_yaw = yaw_channel * Yaw_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.x * Yaw_Mouse_Sen;
    rc_add_pit = pitch_channel * Pitch_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.y * Pitch_Mouse_Sen;

    //��������������ֵ
    *add_yaw = rc_add_yaw;
    *add_pitch = rc_add_pit;
}
//�����ݽ����޷�
static void GIMBAL_relative_angle_limit(Gimbal_Motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->relative_angle_set += add;
    //�Ƿ񳬹���� ��Сֵ
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
    //encondeģʽ�£��������Ƕȿ���
    GIMBAL_relative_angle_limit(&gimbal_set_control->gimbal_yaw_motor, add_yaw_angle);
    GIMBAL_relative_angle_limit(&gimbal_set_control->gimbal_pitch_motor, add_pitch_angle);
}
void CloudMotor_Ctrl(void)
{
    //��̨������ݸ���
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
    
    //��ֹ��̨��ת
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
