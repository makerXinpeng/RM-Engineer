#ifndef _CONTROL_TASK_
#define _CONTROL_TASK_
#include "Remote_Control.h"
//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN 0.006f
//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN 0.005f
//�������yawģʽ�£�ң������yawң�ˣ�max 660�����ӵ�����Ƕȵı���
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//��������̨��ʱ�� ң������yawң�ˣ�max 660��ת���ɳ�����ת�ٶȵı���
#define CHASSIS_WZ_RC_SEN 0.01f


#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f


#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

#define MOTOR_DISTANCE_TO_CENTER 0.2f

//����3508���can���͵���ֵ
#define MAX_MOTOR_CAN_CURRENT 16000.0f

//m3508ת���ɵ����ٶ�(m/s)�ı������������� ����Ϊ���ܻ������Ҫ��������
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//���̵������ٶ�
#define MAX_WHEEL_SPEED 4.0f
//�����˶��������ǰ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_X 3.0f
//�����˶��������ƽ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Y 2.9f
//����������ת�ٶȣ�����ǰ�������ֲ�ͬ�趨�ٶȵı�����Ȩ 0Ϊ�ڼ������ģ�����Ҫ����
#define CHASSIS_WZ_SET_SCALE 0.1f


void Control_Task(RC_ctrl_t *rc);

#endif
