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

// 车身正方向左边升降电机0-ID为7 右边升降电机1-ID为8

//升降任务状态
lift_mode_e lift_mode = Init_MODE;
pinch_mode_e pinch_mode = PINCH_INIT;
//登岛电机数据结构
chassis_move_t lift_wheel;
//升降电机数据结构体
lift_move_t lift_move;

//底盘任务空间剩余量
uint32_t lift_high_water;
void lift_task(void *pvParameters)
{
	//空闲一段时间
  vTaskDelay(2000);
	//升降初始化
	lift_init(&lift_move);
	//登岛电机初始化
	lift_wheel_init(&lift_wheel);
	while(1)
	{
		//升降数据更新
		lift_feedback_update(&lift_move);
		//升降控制PID计算
		lift_control_loop(&lift_move);
		//登岛轮
		lift_wheel_control_loop(&lift_wheel);
		//发送电流值
	  CAN_CMD_LIFT(lift_wheel.motor_chassis[0].give_current, lift_wheel.motor_chassis[1].give_current, lift_move.motor_lift[0].give_current, lift_move.motor_lift[1].give_current);
		//Ni_Ming(0xf1,-lift_move.motor_lift[0].angle,lift_move.motor_lift[1].angle,-lift_move.motor_lift[0].angle_set,lift_move.motor_lift[1].angle_set);
		//控制频率4ms
		vTaskDelay(8);
		lift_high_water = uxTaskGetStackHighWaterMark(NULL);
	}
}

//升降初始化
void lift_init(lift_move_t *lift_init)
{
	if (lift_init == NULL)
	{
		return;
	}
	//升降速度环PID值
	const static float lift_speed_pid[3] = {20, 3, 200};
	//升降位置环PID值
	const static float lift_pos_pid[3] = {8, 0, 0};
	
	//获取遥控指针 
	lift_init->lift_RC = get_remote_control_point();
	
	//获取升降电机数据指针 
	lift_init->motor_lift[0].lift_motor_measure = get_Motor_Measure_Point(6);
	lift_init->motor_lift[1].lift_motor_measure = get_Motor_Measure_Point(7);

	//初始化升降速度环PID 
	for (uint8_t i = 0; i < 2; i++)
	{
		//outmax imax 7000
		PID_Init(&lift_init->motor_speed_pid[i], PID_POSITION, lift_speed_pid, 10000, 3000);
	}
	
	//初始化底盘位置环PID 
	for (uint8_t i = 0; i < 2; i++)
	{
		//outmax imax
		PID_Init(&lift_init->motor_pos_pid[i], PID_POSITION, lift_pos_pid, 1000, 0);
	}
	
	//更新一下数据
  lift_feedback_update(lift_init);
}

//升降数据更新
void lift_feedback_update(lift_move_t *lift_update)
{
	//更新电机速度
	lift_update->motor_lift[0].speed = lift_update->motor_lift[0].lift_motor_measure->filter_rate / 19.0f;
	lift_update->motor_lift[1].speed = lift_update->motor_lift[1].lift_motor_measure->filter_rate / 19.0f;
	
	//更新电机角度
	lift_update->motor_lift[0].angle = lift_update->motor_lift[0].lift_motor_measure->angle;
	lift_update->motor_lift[1].angle = lift_update->motor_lift[1].lift_motor_measure->angle;
	
	//更新升降任务状态
	switch(lift_update->lift_RC->rc.s[0])
	{
		case 1:
		{
			lift_mode = Rc_MODE;//遥控模式
			break;
		}			
		case 3:
		{
			lift_mode = Key_MODE;//键盘模式
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
			lift_mode = Stop_MODE;//停止模式
			break;
		}
		default:
		{
			break;
		}
	}
	
	if(get_chassis_state() == STOP_MODE)
	{
		lift_mode = Stop_MODE;//停止模式
	}
}

//取弹升降高度
static uint32_t pinch_height = 765;//780
//升降控制PID计算
void lift_control_loop(lift_move_t *lift_control)
{	
	switch(lift_mode)
	{
		case Stop_MODE://停止状态
		{
			//升降位置输入
			lift_control->motor_lift[1].angle_set = 100;
			lift_control->motor_lift[0].angle_set = -lift_control->motor_lift[1].angle_set;
			break;
		}
		case Rc_MODE://遥控手杆状态
		{
			//升降位置输入
			lift_control->motor_lift[1].angle_set += lift_control->lift_RC->rc.ch[3] * 0.002f;
			lift_control->motor_lift[0].angle_set = -lift_control->motor_lift[1].angle_set;		
			break;
		}
		case Key_MODE://键盘模式
		{
				switch(pinch_mode)
				{
					case PINCH_INIT://初始状态
					{
						//升降位置输入
						lift_control->motor_lift[1].angle_set = 100;
						lift_control->motor_lift[0].angle_set = -lift_control->motor_lift[1].angle_set;
						break;
					}
					case PINCH_RISE://升高
					{
						//升降位置输入
						lift_control->motor_lift[1].angle_set = pinch_height;
						lift_control->motor_lift[0].angle_set = -lift_control->motor_lift[1].angle_set;	
						break;
					}
					case PINCH_GIVE://键盘模式
					{
						//升降位置输入
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
	
	//取弹输入限幅
	if(lift_control->motor_lift[1].angle_set > 1200)
	{
		lift_control->motor_lift[1].angle_set = 1200;
	}
	if(lift_control->motor_lift[1].angle_set < 0)
	{
		lift_control->motor_lift[1].angle_set = 0;
	}
	//平时为了提高速度P较大，复位初始位置为了保护结构速度降慢P较小
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
	
	//计算PID
	for(uint8_t i = 0; i < 2; i++)
	{
		//位置环
		PID_Calc(&lift_control->motor_pos_pid[i], lift_control->motor_lift[i].angle, lift_control->motor_lift[i].angle_set);
//		//速度环  复位初始位置为了保护结构i变为0，i积分清零
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
		//速度环PID的i积分输出清0
		//lift_control->motor_speed_pid[0].Iout = lift_control->motor_speed_pid[1].Iout = 0;
		//赋值电流值
		lift_control->motor_lift[0].give_current = lift_control->motor_lift[1].give_current = 0;
	}
	else
	{
		//赋值电流值
		lift_control->motor_lift[0].give_current = lift_control->motor_speed_pid[0].out;
		lift_control->motor_lift[1].give_current = lift_control->motor_speed_pid[1].out;
	}
}

//返回升降状态
uint8_t get_pinch_state(void)
{
	return pinch_mode;
}

