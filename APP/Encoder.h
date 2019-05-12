#ifndef _ENCODER_
#define _ENCODER_
#include "main.h"
#include "stm32f4xx.h"

#define RATE_BUF_SIZE 6
typedef struct {
    int32_t raw_value;   					 //编码器不经处理的原始值
    int32_t last_raw_value;					 //上一次的编码器原始值
    int32_t ecd_value;                       //经过处理后连续的编码器值
    int32_t diff;							 //两次编码器之间的差值
    int32_t temp_count;                      //计数用
    uint8_t buf_count;						 //滤波更新buf用
    int32_t ecd_bias;						 //初始编码器值
    int32_t ecd_raw_rate;					 //通过编码器计算得到的速度原始值
    int32_t rate_buf[RATE_BUF_SIZE];	     //buf，for filter
    int32_t round_cnt;						 //圈数
//    int32_t filter_rate;					 //速度
    float ecd_angle;						 //角度
    int16_t rpm;                             //每分钟转速
} Encoder;

void CanReceiveMsgProcess(CanRxMsg *message);
void getEncoderData(volatile Encoder *v, CanRxMsg * msg);

void Quad_Encoder_Configuration(void);
int32_t GetQuadEncoderDiff(void);
void Encoder_Start(void);

#endif
