#include "ChassisMotor.h"

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

//底盘运动数据
chassis_move_t chassis_move;
extern volatile Encoder CMEncoder[4];

//static void chassis_set_mode(chassis_move_t *chassis_move_mode);
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set);
//void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
static void chassis_set_contorl(chassis_move_t *chassis_move_control);

/**
  * @brief	    电机PID参数配置
  * @author	    cxp
  * @param      
  * @retval	    PID参数初始化
  * @date	    2019/3/15
  * @note	    宏定义见库文件ChassisMotor.h
  * @version	3.0.0 使用了chassis_move_t结构体
  * @history	1.0.0
                2.0.0 使用官方的PID_Init函数，将PID参数赋值放到了函数内部
  */
void chassis_init(void)
{
    //底盘速度环pid值
    static const fp32 Chassis_speed_PID[3]= {M3508_MOTOR_SPEED_PID_KP,M3508_MOTOR_SPEED_PID_KI,M3508_MOTOR_SPEED_PID_KD};
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    
    //获取遥控器指针
    chassis_move.chassis_RC=get_remote_control_point();
    //获取云台电机数据指针
    chassis_move.chassis_yaw_motor = get_yaw_motor_point();
    chassis_move.chassis_pitch_motor = get_pitch_motor_point();
    //初始化PID数据
    for(int i=0;i<4;i++)
        PID_Init(&chassis_move.motor_speed_pid[i],PID_POSITION,Chassis_speed_PID,M3508_MOTOR_SPEED_PID_MAX_OUT,M3508_MOTOR_SPEED_PID_MAX_IOUT);
    //用一阶滤波代替斜波函数生成
    first_order_filter_init(&chassis_move.chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move.chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

    //最大 最小速度
    chassis_move.vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move.vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move.vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move.vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    //更新一下数据
    chassis_feedback_update(&chassis_move);
}
/**
  * @brief		底盘电机控制函数，用于pid的实际计算 定时调用，周期4ms 实际值依靠编码器
  * @author		cxp
  * @param      
  * @retval		无
  * @date		2019/3/15
  * @note		3.0.0 将遥控做到函数内部，使用了chassis_move_t结构体,调用麦轮解算函数解算速度
  * @version	3.0.0
  * @history	1.0.0 使用自己的PID函数以及结构体
                2.0.0 版本使用了官方的PID函数，简化了程序
  */
void chassis_control_loop(void)
{
    uint8_t i=0;
    //底盘数据更新
    chassis_feedback_update(&chassis_move);
    //遥控器数据转换为底盘前进和平移的速度，缓启动
    chassis_rc_to_control_vector(&chassis_move.vx_set, &chassis_move.vy_set, &chassis_move);
    
    chassis_set_contorl(&chassis_move);
    //麦克纳姆轮运动分解
    chassis_vector_to_mecanum_wheel_speed(chassis_move.vx_set,chassis_move.vy_set,chassis_move.chassis_RC->rc.ch[0]);
    //PID计算
    for (i = 0; i < 4; i++)
        PID_Calc(&chassis_move.motor_speed_pid[i],chassis_move.motor_chassis[i].speed,chassis_move.motor_chassis[i].speed_set);
    //电流值赋值
    for (i = 0; i < 4; i++)
    {
        chassis_move.motor_chassis[i].give_current = (int16_t)(chassis_move.motor_speed_pid[i].out);
    }
    Set_ChassisMotor_Current(chassis_move.motor_chassis[0].give_current, 
                             chassis_move.motor_chassis[1].give_current, 
                             chassis_move.motor_chassis[2].give_current, 
                             chassis_move.motor_chassis[3].give_current);
}
/**
  * @brief		底盘控制函数
  * @author		cxp
  * @param      
  * @retval		无
  * @date		2019/3/20
  * @note		通过计算云台和底盘之间相对的角度，使前进方向为云台方向
  * @version	1.0.0
  * @history	
  */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
    //设置速度
    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
    vx_set=chassis_move_control->vx_set;
    vy_set=chassis_move_control->vy_set;
    
    fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
    //旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
    sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
    cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
    chassis_move_control->vx_set =  cos_yaw * vx_set - sin_yaw * vy_set;
    chassis_move_control->vy_set =  sin_yaw * vx_set + cos_yaw * vy_set;
    //设置控制相对云台角度
    //chassis_move_control->chassis_relative_angle_set = rad_format(0.25*PI);
    //计算旋转PID角速度
    chassis_move_control->wz_set = -PID_Calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw_motor->relative_angle, chassis_move_control->chassis_relative_angle_set);
    //速度限幅
    chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
    chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
}
/**
  * @brief		麦轮解算函数
  * @author		cxp
  * @param      
  * @retval		无
  * @date		2019/3/15
  * @note		内部调用
  * @version	1.0.0
  * @history	
  */
static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set)
{
    fp32 speed_change=1.0f;
    if(chassis_move.chassis_RC->key.v & KEY_PRESSED_OFFSET_SHIFT)
        speed_change=1.5f;
    else
        speed_change=2.0f;
    chassis_move.motor_chassis[0].speed_set = - vx_set + vy_set + wz_set * CHASSIS_WZ_RC_SEN;
    chassis_move.motor_chassis[1].speed_set =   vx_set + vy_set + wz_set * CHASSIS_WZ_RC_SEN;
    chassis_move.motor_chassis[2].speed_set = - vx_set - vy_set + wz_set * CHASSIS_WZ_RC_SEN;
    chassis_move.motor_chassis[3].speed_set =   vx_set - vy_set + wz_set * CHASSIS_WZ_RC_SEN;
    for(int i = 0; i < 4; i++)
        chassis_move.motor_chassis[i].speed_set /= speed_change;
}
/**
  * @brief		计算实际Vx，Vy，Wz的速度，单位分别为m/s，m/s，rad/s
  * @author		cxp
  * @param      
  * @retval		无
  * @date		2019/3/16
  * @note       注意电机逆时针转为正向，故右侧的轮子的值需要加负号
  * @version	1.0.0 
  * @history    
  */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        //更新电机速度，加速度是速度的PID积分
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * CMEncoder[i].rpm;
        chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
    }
    //更新底盘前进速度x，平移速度y，旋转速度wz，坐标系为右手系
    chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->vy = (chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_move_update->wz = -(chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) / (2*(CHASSIS_LENGTH+CHASSIS_WIDTH));
}
/**
  * @brief		将遥控器数据处理成底盘前进的vx速度，vy速度
  * @author		cxp
  * @param      
  * @retval		无
  * @date		2019/3/17
  * @note       
  * @version	1.0.0 
  * @history    
  */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }
    //遥控器原始通道值
    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
    //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;

    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
    {
        vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
    {
        vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
    }

    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
    {
        vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    {
        vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
    }

    //一阶低通滤波代替斜波作为底盘速度输入
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);

    //停止信号，不需要缓慢加速，直接减速到零
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}
