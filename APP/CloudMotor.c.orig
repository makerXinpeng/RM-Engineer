#include "CloudMotor.h"

//电机编码值规整 0—8191
#define ECD_Format(ecd)         \
    {                           \
        if ((ecd) > ecd_range)  \
            (ecd) -= ecd_range; \
        else if ((ecd) < 0)     \
            (ecd) += ecd_range; \
    }

PidTypeDef GMYaw_Speedloop;
PidTypeDef GMYaw_Positionloop;
int16_t CloudOut[2];

//云台控制所有相关数据
static Gimbal_Control_t gimbal_control;

//云台数据更新
static void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update);
//计算云台电机相对中值的相对角度
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);

    
    
void CloudMotor_Config(void)
{
    static const fp32 YAW_SpeedPID[3]= {20,0,0};
    static const fp32 YAW_PositionPID[3]= {1.2,0,15};
    
    //遥控器数据指针获取
    gimbal_control.gimbal_rc_ctrl = get_remote_control_point();
    
    PID_Init(&GMYaw_Speedloop,PID_POSITION,YAW_SpeedPID,30000,5000);
    PID_Init(&GMYaw_Positionloop,PID_POSITION,YAW_PositionPID,30000,5000);
}
static void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update)
{
    if (gimbal_feedback_update == NULL)
    {
        return;
    }
    //云台数据更新
//    gimbal_feedback_update->gimbal_pitch_motor.absolute_angle = *(gimbal_feedback_update->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);
//    gimbal_feedback_update->gimbal_pitch_motor.relative_angle = motor_ecd_to_angle_change(gimbal_feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
//                                                                                          gimbal_feedback_update->gimbal_pitch_motor.offset_ecd);
//    gimbal_feedback_update->gimbal_pitch_motor.motor_gyro = *(gimbal_feedback_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);

//    gimbal_feedback_update->gimbal_yaw_motor.absolute_angle = *(gimbal_feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
    gimbal_feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(Half_ecd_range,GMYawEncoder.raw_value);

//    gimbal_feedback_update->gimbal_yaw_motor.motor_gyro = arm_cos_f32(gimbal_feedback_update->gimbal_pitch_motor.relative_angle) * (*(gimbal_feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET))
//                                                        - arm_sin_f32(gimbal_feedback_update->gimbal_pitch_motor.relative_angle) * (*(gimbal_feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));
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


void CloudMotor_Ctrl(void)
{
    GIMBAL_Feedback_Update(&gimbal_control);
    GMYaw_Speedloop.set=GIMBAL_PID_Calc(&GMYaw_Positionloop,GMYawEncoder.raw_value,GMYaw_Positionloop.set);
    GMYaw_Speedloop.out=PID_Calc(&GMYaw_Speedloop,GMYawEncoder.filter_rate,GMYaw_Speedloop.set);
    
}
int16_t* CloudMotor_Out(void)//将任务延后到发射任务时发出
{
    CloudOut[0] = GMYaw_Speedloop.out;
    CloudOut[1] = 0;
    return CloudOut;
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
    if(raw_err>0&&(raw_err)>(8190-raw_err))
        pid->error[0] = raw_err-8190;
    else if(raw_err<0&&(-raw_err)>(8190+raw_err))
        pid->error[0] = 8190+raw_err;
    else
        pid->error[0] = raw_err;

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
