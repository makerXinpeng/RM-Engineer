#include "can.h"
#include "Encoder.h"

#include "usart.h"

/**
  *****************************************************************************
  * 函数名 ：      can配置函数
  * 函数功能描述 ：配置can的基本函数
  * 函数参数 ：    无
  * 函数返回值 ：  无
  * 作者 ：        SLDX->wangX
  * 函数创建日期 ：2018年7月2日
  * 函数修改日期 ：
  * 修改人 ：
  * 修改原因 ：
  * 版本 ：
  * 历史版本 ：
  ******************************************************************************
  */
void CAN_FITER_Init(u8 FiterNum,u16 id_std)
{
    CAN_FilterInitTypeDef  can_filter;
    can_filter.CAN_FilterNumber=FiterNum;
    can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
    can_filter.CAN_FilterScale=CAN_FilterScale_32bit;
    can_filter.CAN_FilterIdHigh=(id_std<<5);
    can_filter.CAN_FilterIdLow=0x0000;
    can_filter.CAN_FilterMaskIdHigh=0xFFF0;
    can_filter.CAN_FilterMaskIdLow=0x0000;
    can_filter.CAN_FilterFIFOAssignment=0;//the message which pass the filter save in fifo0
    can_filter.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&can_filter);
}
void CAN_Configure()
{
    CAN_InitTypeDef        can;
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1);

    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOD, &gpio);

    nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel = CAN1_TX_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    CAN_DeInit(CAN1);
    CAN_StructInit(&can);

    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = DISABLE;
    can.CAN_AWUM = DISABLE;
    can.CAN_NART = DISABLE;
    can.CAN_RFLM = DISABLE;
    can.CAN_TXFP = ENABLE;
    can.CAN_Mode = CAN_Mode_Normal;
    can.CAN_SJW  = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_3tq;
    can.CAN_BS2 = CAN_BS2_5tq;
    can.CAN_Prescaler = 5;   //CAN BaudRate 45/(1+3+5)/3=1Mbps
    CAN_Init(CAN1, &can);
    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
    CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE);

    CAN_FITER_Init(0,0x201);
    CAN_FITER_Init(1,0x202);
    CAN_FITER_Init(2,0x203);
    CAN_FITER_Init(3,0x204);
    
    CAN_FITER_Init(4,0x205);
    CAN_FITER_Init(5,0x206);
    CAN_FITER_Init(6,0x207);
    
}

/**
  *****************************************************************************
  * 函数名 ：      can发送中断
  * 函数功能描述 ：can发送成功后进入此中断函数
  * 函数参数 ：    无
  * 函数返回值 ：  无
  * 作者 ：        SLDX->wangX
  * 函数创建日期 ：2018年7月2日
  * 函数修改日期 ：
  * 修改人 ：
  * 修改原因 ：
  * 版本 ：
  * 历史版本 ：
  ******************************************************************************
  */
void CAN1_TX_IRQHandler(void) //CAN TX
{
    if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET)
    {
        CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
    }
}
/**
  *****************************************************************************
  *函数名 ：       can接收中断
  *函数功能描述 ： 当数据到来进入接收中断
  *函数参数 ：     无
  *函数返回值 ：   无
  *作者 ：
  *函数创建日期 ：
  *函数修改日期 ：
  *修改人 ：
  *修改原因 ：
  *版本 ：
  *历史版本 ：
  ******************************************************************************
  */
void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg rx_message;
    if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FF0);
        CAN_ClearFlag(CAN1, CAN_FLAG_FF0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
        CanReceiveMsgProcess(&rx_message);
    }
}
/**
  *****************************************************************************
  *函数名 ：       云台电机电流发送函数
  *函数功能描述 ： 把电流数据通过can发送
  *函数参数 ：     无
  *函数返回值 ：   无
  *作者 ：
  *函数创建日期 ：
  *函数修改日期 ：
  *修改人 ：
  *修改原因 ：
  *版本 ：
  *历史版本 ：
  ******************************************************************************
  */
void Set_CloudMotor_Current(int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq,int16_t trigger_iq)
{
    CanTxMsg tx_message;
    tx_message.StdId = 0x1FF;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;

    tx_message.Data[0] = (unsigned char)(gimbal_yaw_iq >> 8);
    tx_message.Data[1] = (unsigned char)gimbal_yaw_iq;
    tx_message.Data[2] = (unsigned char)(gimbal_pitch_iq >> 8);
    tx_message.Data[3] = (unsigned char)gimbal_pitch_iq;
    tx_message.Data[4] = (unsigned char)(trigger_iq >> 8);
    tx_message.Data[5] = (unsigned char)trigger_iq;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CAN1,&tx_message);
}
/**
  *****************************************************************************
  *函数名 ：      底盘电机电流控制
  *函数功能描述 ：
  *函数参数 ：
  *函数返回值 ：
  *作者 ：
  *函数创建日期 ：
  *函数修改日期 ：
  *修改人 ：
  *修改原因 ：
  *版本 ：
  *历史版本 ：
  ******************************************************************************
  */
void Set_ChassisMotor_Current(int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
{
    CanTxMsg tx_message;
    tx_message.StdId = 0x200;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;

    tx_message.Data[0] = (uint8_t)(cm1_iq >> 8);
    tx_message.Data[1] = (uint8_t)cm1_iq;
    tx_message.Data[2] = (uint8_t)(cm2_iq >> 8);
    tx_message.Data[3] = (uint8_t)cm2_iq;
    tx_message.Data[4] = (uint8_t)(cm3_iq >> 8);
    tx_message.Data[5] = (uint8_t)cm3_iq;
    tx_message.Data[6] = (uint8_t)(cm4_iq >> 8);
    tx_message.Data[7] = (uint8_t)cm4_iq;
    CAN_Transmit(CAN1,&tx_message);
}
