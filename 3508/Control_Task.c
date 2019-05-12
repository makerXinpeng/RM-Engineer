#include "Control_Task.h"
#include "CloudMotor.h"
#include "stm32f4xx.h"
#include "ChassisMotor.h"
#include "shoot.h"
#include "usart.h"
#define STANDERD 80.0f
#define len 1.15f
#define wide 1.0f
float time=0;
int set=0;
void Control_Task(RC_ctrl_t *Rc)
{

    CM_LF_Speedloop.Set = -STANDERD * (Rc->rc.ch[3] * CHASSIS_VX_RC_SEN + Rc->rc.ch[2] * CHASSIS_VY_RC_SEN - Rc->rc.ch[0] * CHASSIS_WZ_RC_SEN * 0.3f * (wide + len));
    CM_LB_Speedloop.Set = -STANDERD * (Rc->rc.ch[3] * CHASSIS_VX_RC_SEN - Rc->rc.ch[2] * CHASSIS_VY_RC_SEN - Rc->rc.ch[0] * CHASSIS_WZ_RC_SEN * 0.3f * (wide + len));
    CM_RF_Speedloop.Set =  STANDERD * (Rc->rc.ch[3] * CHASSIS_VX_RC_SEN - Rc->rc.ch[2] * CHASSIS_VY_RC_SEN + Rc->rc.ch[0] * CHASSIS_WZ_RC_SEN * 0.3f * (wide + len));
    CM_RB_Speedloop.Set =  STANDERD * (Rc->rc.ch[3] * CHASSIS_VX_RC_SEN + Rc->rc.ch[2] * CHASSIS_VY_RC_SEN + Rc->rc.ch[0] * CHASSIS_WZ_RC_SEN * 0.3f * (wide + len));

    
    //GMYaw_Positionloop.Set = (float)(Rc->rc.ch[4]+660)*8192/1320;
    //²âÊÔÏìÓ¦ËÙ¶È
    
    if(time>=8190)
        time=0;
    if(time==7000||time==4000)
    {
        GMYaw_Positionloop.Set = time;
        set=time;
    }
    time+=2;
    
    TR_Speedloop.Set = (Rc->rc.s[0] == 1) ? 20 : 0;
    ChassisMotor_Ctrl();
    CloudMotor_Ctrl();
    TriggerMotor_Ctrl();
    ChassisMotor_Out();
    TriggerMotor_Out();
}
