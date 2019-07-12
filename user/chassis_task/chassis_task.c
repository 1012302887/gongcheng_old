#include "chassis_task.h"
#include "stm32f4xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "projdefs.h"

#include "usart1.h"
#include "rc.h"
#include "tof.h"

#include "can_receive.h"
#include "pid.h"
#include "ramp.h"

//��ʼ�����̼����ٶ�б�º���
static ramp_t FBSpeedRamp = RAMP_GEN_DAFAULT;
static ramp_t LRSpeedRamp = RAMP_GEN_DAFAULT;

static chassis_mode_e chassis_mode = INIT_MODE;		

extern float forward_back_speed, left_right_speed;		
		
//�������ݽṹ��
chassis_move_t chassis_move;

//��������ռ�ʣ����
uint32_t chassis_high_water;
void chassis_task(void *pvParameters)
{
	//����һ��ʱ��
  vTaskDelay(2000);
	//���̳�ʼ��
	chassis_init(&chassis_move);
	while(1)
	{
		//�������ݸ���
		chassis_feedback_update(&chassis_move);
		//���̿���PID����
		chassis_control_loop(&chassis_move);
		//����������ѭ��
		CAN_CMD_CHASSIS(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,	chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
		//Ni_Ming(0xf1,chassis_move.gyro_data->v_z,0,0,0);
		//��������Ƶ��4ms	 
		vTaskDelay(2);
		chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
	}
}

//���̳�ʼ��
void chassis_init(chassis_move_t *chassis_init)
{
	if (chassis_init == NULL)
	{
		return;
	}
	//�����ٶȻ�PIDֵ
	const static float motor_speed_pid[3] = {700, 0, 0};
	//����λ�û�PIDֵ
	const static float motor_pos_pid[3] = {0, 0, 0};
	
	//��ȡ���̵������ָ��
	for (uint8_t i = 0; i < 4; i++)
	{  
		chassis_init->motor_chassis[i].chassis_motor_measure = get_Motor_Measure_Point(i);
	}
	
	//��ȡң��ָ�� 
	chassis_init->chassis_RC = get_remote_control_point();
	
	//��ȡ����������ָ��
	chassis_init->gyro_data = get_GYRO_Measure_Point();
	
	//��ȡ����tof����
	chassis_init->tof_measure = get_tof_Info_Measure_Point();
	
	//��ʼ�������ٶȻ�PID 
	for (uint8_t i = 0; i < 4; i++)
	{
		//pmax imax
		PID_Init(&chassis_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, 10000, 0);
	}
	
	//��ʼ������λ�û�PID 
	for (uint8_t i= 0; i < 4; i++)
	{
		//pmax imax
		PID_Init(&chassis_init->motor_pos_pid[i], PID_POSITION, motor_pos_pid, 0, 0);
	}
	 
	//��ʼ��Z��PID
	CHISSIS_PID_Init(&chassis_init->chassis_gryo_pid, 2000, 0, 12, 0, 0);//kp_out ki_out kp ki kd 20 60
	CHISSIS_PID_Init(&chassis_init->chassis_acc_pid, 60, 0, 0.3, 0, 0);
	
	//���������ʼ�����ʱ���������ǵĽǶ�
	chassis_init->gyro_angle_start = chassis_init->gyro_data->yaw;
	
	//����һ������
  chassis_feedback_update(chassis_init);
}

//�������ݸ���
void chassis_feedback_update(chassis_move_t *chassis_update)
{
	//���µ���ٶȣ����ٶ����ٶȵ�PID΢��
	chassis_update->motor_chassis[0].speed = chassis_update->motor_chassis[0].chassis_motor_measure->filter_rate / 19.0f;
	chassis_update->motor_chassis[1].speed = chassis_update->motor_chassis[1].chassis_motor_measure->filter_rate / 19.0f;
	chassis_update->motor_chassis[2].speed = chassis_update->motor_chassis[2].chassis_motor_measure->filter_rate / 19.0f;
	chassis_update->motor_chassis[3].speed = chassis_update->motor_chassis[3].chassis_motor_measure->filter_rate / 19.0f;
	chassis_update->vw_mouse = chassis_update->chassis_RC->mouse.x;
	chassis_update->tof_h = chassis_update->tof_measure->tof_h;
	
	//���µ���״̬
	switch(chassis_update->chassis_RC->rc.s[0])
	{
		case 1:
		{
			chassis_mode = RC_MODE;
			break;
		}			
		case 3:
		{
			chassis_mode = KEY_MODE;
			break;
		}
		case 2:
		{
			chassis_mode = STOP_MODE;
			break;
		}
		default:
		{
			break;
		}
	}
	
	//ʧȥң���źź����ģʽΪֹͣģʽ
	if(xTaskGetTickCount() - chassis_update->chassis_RC->time > 88)
	{
		chassis_mode = STOP_MODE;
	}
}

//���̿���PID����
void chassis_control_loop(chassis_move_t *chassis_control)
{
	switch(chassis_mode)
	{
		case RC_MODE://ң��ģʽ
		{
			chassis_control->vx =  chassis_control->chassis_RC->rc.ch[0] * 60.0f/660.0f;
			chassis_control->vy =  chassis_control->chassis_RC->rc.ch[1] * 60.0f/660.0f;
			chassis_control->vw_offset += chassis_control->chassis_RC->rc.ch[2] * 50/660 *  0.02;
			chassis_control->vw_set = chassis_control->vw_offset + chassis_control->gyro_angle_start;
			break;
		}			
		case KEY_MODE://����ģʽ
		{
			chassis_control->key_time++;//4msһ��
			if(chassis_control->chassis_RC->rc.s[1] == 1)
			{
				chassis_control->vy_offset = 12;
				chassis_control->vx_offset = 12;
			}
			else if(chassis_control->chassis_RC->key.v & SHIFT)//shitf����																
			{
				chassis_control->vy_offset = 60;
				chassis_control->vx_offset = 50;
			}
			else
			{
				chassis_control->vy_offset = 50;
				chassis_control->vx_offset = 40;
			}
		
			//W��Sǰ��
			if(chassis_control->chassis_RC->key.v & W)
			{
				chassis_control->vy = chassis_control->vy_offset * ramp_calc(&FBSpeedRamp);
			}
			else if(chassis_control->chassis_RC->key.v & S)
			{
				chassis_control->vy = -chassis_control->vy_offset * ramp_calc(&FBSpeedRamp);
			}
			else
			{
				chassis_control->vy =0;
				ramp_init(&FBSpeedRamp, 400);
			}
		
			//A��Dƽ��
			if(chassis_control->chassis_RC->key.v & A)
			{
				chassis_control->vx = -chassis_control->vx_offset * ramp_calc(&LRSpeedRamp);
			}
			else if(chassis_control->chassis_RC->key.v & D)
			{
				chassis_control->vx = chassis_control->vx_offset * ramp_calc(&LRSpeedRamp);
			}
			else
			{
				chassis_control->vx = 0;
				ramp_init(&LRSpeedRamp, 400);
			}
			
			//��ת
			if(chassis_control->chassis_RC->rc.s[1] == 1)//ȡ��״̬
			{
				chassis_control->vw_set = chassis_control->gyro_data->yaw;
			}
			else if((chassis_control->chassis_RC->key.v & CTRL) && (chassis_control->key_time - chassis_control->last_press_time >500))
			{
				chassis_control->last_press_time = chassis_control->key_time;
				chassis_control->vw_offset += 180;
			}
			else //������
			{
				if(chassis_control->vw_mouse >  30)chassis_control->vw_mouse = 30; 
				if(chassis_control->vw_mouse < -30)chassis_control->vw_mouse = -30;			
				chassis_control->vw_offset += chassis_control->vw_mouse * 0.015;
				chassis_control->vw_set = chassis_control->vw_offset + chassis_control->gyro_angle_start;
			}
			if((chassis_control->chassis_RC->key.v & Z) && (chassis_control->key_time - chassis_control->last_press_time >250))
			{
				chassis_control->last_press_time = chassis_control->key_time;
				if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1) == 0)
				{
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET); 
				}
				else
				{
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET); 
				}
			}
			break;
		}
		case STOP_MODE://ֹͣģʽ
		{
			chassis_control->vx = chassis_control->vy = 0;
			chassis_control->vw_set = chassis_control->gyro_data->yaw;
			chassis_control->gyro_angle_start = chassis_control->gyro_data->yaw;
			break;
		}
		default:
		{
			break;
		}
	}
	
	//Z��Ƕ�PID����
	PID_Calc(&chassis_control->chassis_gryo_pid, chassis_control->gyro_data->yaw, chassis_control->vw_set);
	//Z����ٶ�PID����
	chassis_control->vw = PID_Calc(&chassis_control->chassis_acc_pid, -chassis_control->gyro_data->v_z, chassis_control->chassis_gryo_pid.out);
	
	//�����ٶ��趨
	chassis_control->motor_chassis[0].speed_set = +(int16_t)chassis_control->vx - (int16_t)chassis_control->vy + (int16_t)chassis_control->vw;
	chassis_control->motor_chassis[1].speed_set = +(int16_t)chassis_control->vx + (int16_t)chassis_control->vy + (int16_t)chassis_control->vw;
	chassis_control->motor_chassis[2].speed_set = -(int16_t)chassis_control->vx - (int16_t)chassis_control->vy + (int16_t)chassis_control->vw;
	chassis_control->motor_chassis[3].speed_set = -(int16_t)chassis_control->vx + (int16_t)chassis_control->vy + (int16_t)chassis_control->vw;
	//����PID

	PID_Calc(&chassis_control->motor_speed_pid[0], chassis_control->motor_chassis[0].speed, chassis_control->motor_chassis[0].speed_set);
	PID_Calc(&chassis_control->motor_speed_pid[1], chassis_control->motor_chassis[1].speed, chassis_control->motor_chassis[1].speed_set);
	PID_Calc(&chassis_control->motor_speed_pid[2], chassis_control->motor_chassis[2].speed, chassis_control->motor_chassis[2].speed_set);
	PID_Calc(&chassis_control->motor_speed_pid[3], chassis_control->motor_chassis[3].speed, chassis_control->motor_chassis[3].speed_set);
	
	//��ֵ����ֵ
	chassis_control->motor_chassis[0].give_current = (int16_t)(chassis_control->motor_speed_pid[0].out);
	chassis_control->motor_chassis[1].give_current = (int16_t)(chassis_control->motor_speed_pid[1].out);
	chassis_control->motor_chassis[2].give_current = (int16_t)(chassis_control->motor_speed_pid[2].out);
	chassis_control->motor_chassis[3].give_current = (int16_t)(chassis_control->motor_speed_pid[3].out);
}

//���ص�������״̬
uint8_t get_chassis_state(void)
{
	return chassis_mode;
}

//����Z��PID��ʼ��
void CHISSIS_PID_Init(PidTypeDef *pid, float maxout, float max_iout, float kp, float ki, float kd)
{
	if (pid == NULL)
	{
			return;
	}
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;

	pid->set = 0.0f;

	pid->max_iout = max_iout;
	pid->max_out = maxout;
}
