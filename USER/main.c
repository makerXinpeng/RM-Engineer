/**
  ***************************(C) COPYRIGHT 2019 HQURM***************************
  * @file       main.c/h
  * @brief      stm32初始化以及开始任务TIM6。h文件定义相关全局宏定义以及
  *             typedef 一些常用数据类型
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Feb-15-2019     陈新朋          1. 未加freeRTOS
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ***************************(C) COPYRIGHT 2019 HQURM***************************
  */
#include "main.h"

#include "sys.h"
#include "usart.h"
#include "delay.h"

#include "rc.h"
#include "can.h"
#include "rng.h"
#include "laser.h"
#include "timer.h"
#include "fric.h"
#include "power_ctrl.h "

#include "Encoder.h"
#include "Control_Task.h"

#include "ChassisMotor.h"
#include "shoot.h"

#include "remote_control.h"

//四个24v 输出 依次开启 间隔 709us
#define POWER_CTRL_ONE_BY_ONE_TIME 709

void BSP_init(void);

int main(void)
{
    BSP_init();
    while(1)
    {
    }
}
void BSP_init(void)
{
    delay_init(180); //参数180为系统时钟频率
    uart_init(115200);	//串口初始化波特率为115200
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4

    laser_configuration();
    
    CAN_Configure();
    Encoder_Start();

    TIM6_Configuration();

    remote_control_init();
    
    chassis_init();
    
    CloudMotor_Config();
    TriggerMotor_PID_Config();
    
    //24输出控制口 初始化
    power_ctrl_configuration();
    
    fric_PWM_configuration();
    
    //24v 输出 依次上电
    for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)
    {
        power_ctrl_on(i);
        delay_us(POWER_CTRL_ONE_BY_ONE_TIME);
    }
}

void TIM6_DAC_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM6,TIM_IT_Update)!= RESET)
    {
        TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
        TIM_ClearFlag(TIM6, TIM_FLAG_Update);
        Control_Task(get_remote_control_point());
    }
}
