#include "main.h"
#include "Control_Task.h"
#include "CloudMotor.h"
#include "Encoder.h"
#include "stm32f4xx.h"
#include "ChassisMotor.h"
#include "shoot.h"
#include "pid.h"
#include "fric.h"
#include "delay.h"

#include "usart.h"

#include "arm_math.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

static fp32 count[2]= {0,0};
void Control_Task(RC_ctrl_t *Rc)
{
    //ch    0   1   2   3
    //Í¨µÀ ÓÒx ÓÒy ×óx ×óy
    if(Rc->key.v & KEY_PRESSED_OFFSET_Q)
    {
        if(count[0]>-660)
        {
            count[0]-=10;
        }
        Rc->rc.ch[0]=count[0];
    }
    else
    {
        count[0]=0;
    }
    if(Rc->key.v & KEY_PRESSED_OFFSET_E)
    {
        if(count[1]<660)
        {
            count[1]+=10;
        }
        Rc->rc.ch[0]=count[1];
    }
    else
    {
        count[1]=0;
    }
    chassis_control_loop();
    CloudMotor_Ctrl();
    shoot_control_loop();
}
