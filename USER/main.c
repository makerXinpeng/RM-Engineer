/**
  ***************************(C) COPYRIGHT 2019 HQURM***************************
  * @file       main.c/h
  * @brief      stm32��ʼ���Լ���ʼ����TIM6��h�ļ��������ȫ�ֺ궨���Լ�
  *             typedef һЩ������������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Feb-15-2019     ������          1. δ��freeRTOS
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

//�ĸ�24v ��� ���ο��� ��� 709us
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
    delay_init(180); //����180Ϊϵͳʱ��Ƶ��
    uart_init(115200);	//���ڳ�ʼ��������Ϊ115200
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4

    laser_configuration();
    
    CAN_Configure();
    Encoder_Start();

    TIM6_Configuration();

    remote_control_init();
    
    chassis_init();
    
    CloudMotor_Config();
    TriggerMotor_PID_Config();
    
    //24������ƿ� ��ʼ��
    power_ctrl_configuration();
    
    fric_PWM_configuration();
    
    //24v ��� �����ϵ�
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
