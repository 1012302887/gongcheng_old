#include "lift_task.h"
#include "stm32f4xx_hal.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "usart1.h"

#include "rc.h"
#include "can_receive.h"
#include "pid.h"
#include "chassis_task.h"
#include "lift_wheel.h"

// ��������������������0-IDΪ7 �ұ��������1-IDΪ8

//��������״̬
lift_mode_e lift_mode = Init_MODE;
pinch_mode_e pinch_mode = PINCH_INIT;
//�ǵ�������ݽṹ
chassis_move_t lift_wheel;
//����������ݽṹ��
lift_move_t lift_move;

//��������ռ�ʣ����
uint32_t lift_high_water;
void lift_task(void *pvParameters)
{
	//����һ��ʱ��
  vTaskDelay(2000);
	//������ʼ��
	lift_init(&lift_move);
	//�ǵ������ʼ��
	lift_wheel_init(&lift_wheel);
	while(1)
	{
		//�������ݸ���
		lift_feedback_update(&lift_move);
		//��������PID����
		lift_control_loop(&lift_move);
		//�ǵ���
		lift_wheel_control_loop(&lift_wheel);
		//���͵���ֵ
	  CAN_CMD_LIFT(lift_wheel.motor_chassis[0].give_current, lift_wheel.motor_chassis[1].give_current, lift_move.motor_lift[0].give_current, lift_move.motor_lift[1].give_current);
		//Ni_Ming(0xf1,-lift_move.motor_lift[0].angle,lift_move.motor_lift[1].angle,-lift_move.motor_lift[0].angle_set,lift_move.motor_lift[1].angle_set);
		//����Ƶ��4ms
		vTaskDelay(8);
		lift_high_water = uxTaskGetStackHighWaterMark(NULL);
	}
}

//������ʼ��
void lift_init(lift_move_t *lift_init)
{
	if (lift_init == NULL)
	{
		return;
	}
	//�����ٶȻ�PIDֵ
	const static float lift_speed_pid[3] = {20, 3, 200};
	//����λ�û�PIDֵ
	const static float lift_pos_pid[3] = {8, 0, 0};
	
	//��ȡң��ָ�� 
	lift_init->lift_RC = get_remote_control_point();
	
	//��ȡ�����������ָ�� 
	lift_init->motor_lift[0].lift_motor_measure = get_Motor_Measure_Point(6);
	lift_init->motor_lift[1].lift_motor_measure = get_Motor_Measure_Point(7);

	//��ʼ�������ٶȻ�PID 
	for (uint8_t i = 0; i < 2; i++)
	{
		//outmax imax 7000
		PID_Init(&lift_init->motor_speed_pid[i], PID_POSITION, lift_speed_pid, 10000, 3000);
	}
	
	//��ʼ������λ�û�PID 
	for (uint8_t i = 0; i < 2; i++)
	{
		//outmax imax
		PID_Init(&lift_init->motor_pos_pid[i], PID_POSITION, lift_pos_pid, 1000, 0);
	}
	
	//����һ������
  lift_feedback_update(lift_init);
}

//�������ݸ���
void lift_feedback_update(lift_move_t *lift_update)
{
	//���µ���ٶ�
	lift_update->motor_lift[0].speed = lift_update->motor_lift[0].lift_motor_measure->filter_rate / 19.0f;
	lift_update->motor_lift[1].speed = lift_update->motor_lift[1].lift_motor_measure->filter_rate / 19.0f;
	
	//���µ���Ƕ�
	lift_update->motor_lift[0].angle = lift_update->motor_lift[0].lift_motor_measure->angle;
	lift_update->motor_lift[1].angle = lift_update->motor_lift[1].lift_motor_measure->angle;
	
	//������������״̬
	switch(lift_update->lift_RC->rc.s[0])
	{
		case 1:
		{
			lift_mode = Rc_MODE;//ң��ģʽ
			break;
		}			
		case 3:
		{
			lift_mode = Key_MODE;//����ģʽ
			if(lift_update->lift_RC->rc.s[1] == 1)
			{
				pinch_mode = PINCH_RISE;
			}
			else if(lift_update->lift_RC->rc.s[1] == 3)
			{
				pinch_mode = PINCH_INIT;
			}
			else if(lift_update->lift_RC->rc.s[1] == 2)
			{
				pinch_mode = PINCH_GIVE;
			}
			break;
		}
		case 2:
		{
			lift_mode = Stop_MODE;//ֹͣģʽ
			break;
		}
		default:
		{
			break;
		}
	}
	
	if(get_chassis_state() == STOP_MODE)
	{
		lift_mode = Stop_MODE;//ֹͣģʽ
	}
}

//ȡ�������߶�
static uint32_t pinch_height = 765;//780
//��������PID����
void lift_control_loop(lift_move_t *lift_control)
{	
	switch(lift_mode)
	{
		case Stop_MODE://ֹͣ״̬
		{
			//����λ������
			lift_control->motor_lift[1].angle_set = 100;
			lift_control->motor_lift[0].angle_set = -lift_control->motor_lift[1].angle_set;
			break;
		}
		case Rc_MODE://ң���ָ�״̬
		{
			//����λ������
			lift_control->motor_lift[1].angle_set += lift_control->lift_RC->rc.ch[3] * 0.002f;
			lift_control->motor_lift[0].angle_set = -lift_control->motor_lift[1].angle_set;		
			break;
		}
		case Key_MODE://����ģʽ
		{
				switch(pinch_mode)
				{
					case PINCH_INIT://��ʼ״̬
					{
						//����λ������
						lift_control->motor_lift[1].angle_set = 100;
						lift_control->motor_lift[0].angle_set = -lift_control->motor_lift[1].angle_set;
						break;
					}
					case PINCH_RISE://����
					{
						//����λ������
						lift_control->motor_lift[1].angle_set = pinch_height;
						lift_control->motor_lift[0].angle_set = -lift_control->motor_lift[1].angle_set;	
						break;
					}
					case PINCH_GIVE://����ģʽ
					{
						//����λ������
						lift_control->motor_lift[1].angle_set = pinch_height + 100;
						lift_control->motor_lift[0].angle_set = -lift_control->motor_lift[1].angle_set;	
						break;
					}
					default:
					{
						break;
					}
				}
			break;
		}
		default:
		{
			break;
		}
	}
	
	//ȡ�������޷�
	if(lift_control->motor_lift[1].angle_set > 1200)
	{
		lift_control->motor_lift[1].angle_set = 1200;
	}
	if(lift_control->motor_lift[1].angle_set < 0)
	{
		lift_control->motor_lift[1].angle_set = 0;
	}
	//ƽʱΪ������ٶ�P�ϴ󣬸�λ��ʼλ��Ϊ�˱����ṹ�ٶȽ���P��С
//	if(lift_control->motor_lift[1].angle_set < 400)
//	{
//		lift_control->motor_pos_pid[0].Kp = lift_control->motor_pos_pid[1].Kp = 5;
//		lift_control->motor_speed_pid[0].max_out = lift_control->motor_speed_pid[1].max_out = 2000;
//	}
//	else
//	{
//		lift_control->motor_speed_pid[0].max_out = lift_control->motor_speed_pid[1].max_out = 7000;
//		lift_control->motor_pos_pid[0].Kp = lift_control->motor_pos_pid[1].Kp = 8;
//	}
	
	//����PID
	for(uint8_t i = 0; i < 2; i++)
	{
		//λ�û�
		PID_Calc(&lift_control->motor_pos_pid[i], lift_control->motor_lift[i].angle, lift_control->motor_lift[i].angle_set);
//		//�ٶȻ�  ��λ��ʼλ��Ϊ�˱����ṹi��Ϊ0��i��������
//		if(lift_control->motor_lift[i].angle_set < 200)
//		{
//			lift_control->motor_speed_pid[i].Ki = 1;
//			lift_control->motor_speed_pid[i].Iout = 0;
//		}
//		else
//		{
//			lift_control->motor_speed_pid[i].Ki = 1;
//		}
		PID_Calc(&lift_control->motor_speed_pid[i], lift_control->motor_lift[i].speed, lift_control->motor_pos_pid[i].out);
	}
	
	if(lift_mode == Stop_MODE)
	{
		//�ٶȻ�PID��i���������0
		//lift_control->motor_speed_pid[0].Iout = lift_control->motor_speed_pid[1].Iout = 0;
		//��ֵ����ֵ
		lift_control->motor_lift[0].give_current = lift_control->motor_lift[1].give_current = 0;
	}
	else
	{
		//��ֵ����ֵ
		lift_control->motor_lift[0].give_current = lift_control->motor_speed_pid[0].out;
		lift_control->motor_lift[1].give_current = lift_control->motor_speed_pid[1].out;
	}
}

//��������״̬
uint8_t get_pinch_state(void)
{
	return pinch_mode;
}

