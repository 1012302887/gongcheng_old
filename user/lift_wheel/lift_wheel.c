#include "lift_wheel.h"
#include "stm32f4xx_hal.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "projdefs.h"

#include "usart1.h"
#include "rc.h"

#include "can_receive.h"
#include "pid.h"
#include "ramp.h"

//��ʼ�����̼����ٶ�б�º���
static ramp_t LFSpeedRamp = RAMP_GEN_DAFAULT;
float forward_back_speed, left_right_speed;

void lift_wheel_init(chassis_move_t *lift_wheel_init)
{
	if (lift_wheel_init == NULL)
	{
		return;
	}
	//�����ٶȻ�PIDֵ
	const static float motor_speed_pid[3] = {1000, 0, 0};
	 
	//��ȡ�ǵ��������ָ��
	lift_wheel_init->motor_chassis[0].chassis_motor_measure = get_Motor_Measure_Point(4);
	lift_wheel_init->motor_chassis[1].chassis_motor_measure = get_Motor_Measure_Point(5);
	
	//��ȡң��ָ�� 
	lift_wheel_init->chassis_RC = get_remote_control_point();
	
	//��ʼ�������ٶȻ�PID 
	//pmax imax 
	PID_Init(&lift_wheel_init->motor_speed_pid[0], PID_POSITION, motor_speed_pid, 10000, 0);
	PID_Init(&lift_wheel_init->motor_speed_pid[1], PID_POSITION, motor_speed_pid, 10000, 0);
	
	//����һ������
  lift_wheel_feedback_update(lift_wheel_init);
}

//�ǵ������ݸ���
void lift_wheel_feedback_update(chassis_move_t *lift_wheel_update)
{
	lift_wheel_update->motor_chassis[0].speed = lift_wheel_update->motor_chassis[0].chassis_motor_measure->filter_rate / 19.0f;
	lift_wheel_update->motor_chassis[1].speed = lift_wheel_update->motor_chassis[1].chassis_motor_measure->filter_rate / 19.0f;
}

static uint32_t lift_wheel_system_time = 0;
//���̿���PID����
void lift_wheel_control_loop(chassis_move_t *lift_wheel_control)
{
	//���µ������
	lift_wheel_feedback_update(lift_wheel_control);
	//ң���ұ߲����м䵲Ϊ����ģʽ
	if(lift_wheel_control->chassis_RC->rc.s[0] == 3)
	{
		//W��Sǰ��
		if(lift_wheel_control->chassis_RC->key.v & W)
		{
			lift_wheel_control->vx = forward_back_speed * ramp_calc(&LFSpeedRamp);
		}
		else if(lift_wheel_control->chassis_RC->key.v & S)
		{
			lift_wheel_control->vx = -forward_back_speed * ramp_calc(&LFSpeedRamp);
		}
		else
		{
			lift_wheel_control->vx =0;
			ramp_init(&LFSpeedRamp, 200);
		}
		
	}
	//ң�ز���������λΪң��ģʽ
	else
	{
		//ǰ��Ϊң���ұ�ͨ��0������Ϊң���ұ�ͨ��1
		lift_wheel_control->vx =  lift_wheel_control->chassis_RC->rc.ch[1] * 30.0f/660.0f;
	}
	
	//��ȡ��ǰϵͳʱ��
	lift_wheel_system_time = xTaskGetTickCount();
	
	//�������ϵͳ��ǰʱ���ȥ����ң���жϵ�ǰʱ�䣬˵��û���յ�ң���ź�
	if((lift_wheel_system_time - lift_wheel_control->chassis_RC->time) > 88)
	{
		lift_wheel_control->vx= 0;
	}
	
	//�����ٶ��趨
	lift_wheel_control->motor_chassis[0].speed_set = -lift_wheel_control->vx;
	lift_wheel_control->motor_chassis[1].speed_set =  lift_wheel_control->vx; 
	//����PID

	PID_Calc(&lift_wheel_control->motor_speed_pid[0], lift_wheel_control->motor_chassis[0].speed, lift_wheel_control->motor_chassis[0].speed_set);
	PID_Calc(&lift_wheel_control->motor_speed_pid[1], lift_wheel_control->motor_chassis[1].speed, lift_wheel_control->motor_chassis[1].speed_set);
	
	//��ֵ����ֵ
	lift_wheel_control->motor_chassis[0].give_current = (int16_t)(lift_wheel_control->motor_speed_pid[0].out);
	lift_wheel_control->motor_chassis[1].give_current = (int16_t)(lift_wheel_control->motor_speed_pid[1].out);
	//printf("%d\r\n",chassis_control->chassis_RC->rc.s[0]);
}
