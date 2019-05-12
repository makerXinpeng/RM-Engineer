#include "Encoder.h"
#include "can.h"
#include "main.h"
#include "usart.h"

volatile Encoder CMEncoder[4] = {{0,0,0,0,0,0,0,0,0,0},
                                 {0,0,0,0,0,0,0,0,0,0},
                                 {0,0,0,0,0,0,0,0,0,0},
                                 {0,0,0,0,0,0,0,0,0,0}};
volatile Encoder GMYawEncoder =  {0,0,0,0,0,0,0,0,0,0};
volatile Encoder GMPitchEncoder ={0,0,0,0,0,0,0,0,0,0};
volatile Encoder TREncoder =     {0,0,0,0,0,0,0,0,0,0};

/**
  *****************************************************************************
  *函数名 ：
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
void CanReceiveMsgProcess(CanRxMsg *message)
{
    switch(message->StdId)
    {
    case CAN_ID_CM1:
    {
        getEncoderData(&CMEncoder[0], message);
        break;
    }
    case CAN_ID_CM2:
    {
        getEncoderData(&CMEncoder[1], message);
        break;
    }
    case CAN_ID_CM3:
    {
        getEncoderData(&CMEncoder[2], message);
        break;
    }
    case CAN_ID_CM4:
    {
        getEncoderData(&CMEncoder[3], message);
        break;
    }
    case CAN_ID_PITCH:
    {
        getEncoderData(&GMPitchEncoder, message);
        break;
    }
    case CAN_ID_YAW:
    {
        getEncoderData(&GMYawEncoder, message);
        break;
    }
    case CAN_ID_TRIGGER:
    {
        getEncoderData(&TREncoder, message);
        break;
    }
    }
}

/**
  *****************************************************************************
  *函数名 ：
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
void getEncoderData(volatile Encoder *v, CanRxMsg * msg)
{
    int i=0;
    int32_t temp_sum = 0;
    v->last_raw_value = v->raw_value;
    v->raw_value = (msg->Data[0]<<8)|msg->Data[1];
    v->rpm = ((msg->Data[2]<<8)|(msg->Data[3]));
    v->diff = v->raw_value - v->last_raw_value;
    if(v->diff < -7500)    //两次编码器的反馈值差别太大，表示圈数发生了改变
    {
        v->round_cnt++;
        v->ecd_raw_rate = v->diff + 8192;
    }
    else if(v->diff>7500)
    {
        v->round_cnt--;
        v->ecd_raw_rate = v->diff - 8192;
    }
    else
    {
        v->ecd_raw_rate = v->diff;
    }
    //计算得到连续的编码器输出值
    v->ecd_value = v->raw_value + v->round_cnt * 8192;
    //计算得到角度值，范围正负无穷大
    v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*360/8192 + v->round_cnt * 360;
    v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
    if(v->buf_count == RATE_BUF_SIZE)
    {
        v->buf_count = 0;
    }
    //计算速度平均值
    for(i = 0; i < RATE_BUF_SIZE; i++)
    {
        temp_sum += v->rate_buf[i];
    }
    v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);
}

/**
  *****************************************************************************
  *函数名 ：
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
void Quad_Encoder_Configuration(void)
{
    GPIO_InitTypeDef gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOD,&gpio);

    GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4);
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_TIM4);
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);
    TIM_Cmd(TIM4,ENABLE);
}
/**
  *****************************************************************************
  *函数名 ：
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
void Encoder_Start(void)
{
    TIM4->CNT = 0x7fff;
}

/**
 *****************************************************************************
 *函数名 ：
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
int32_t cnt = 0;
int32_t GetQuadEncoderDiff(void)
{

    cnt = (TIM4->CNT)-0x7fff;

    return cnt;
}
