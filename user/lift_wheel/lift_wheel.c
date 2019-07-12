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

//初始化底盘键盘速度斜坡函数
static ramp_t LFSpeedRamp = RAMP_GEN_DAFAULT;
float forward_back_speed, left_right_speed;

void lift_wheel_init(chassis_move_t *lift_wheel_init)
{
	if (lift_wheel_init == NULL)
	{
		return;
	}
	//底盘速度环PID值
	const static float motor_speed_pid[3] = {1000, 0, 0};
	 
	//获取登岛电机数据指针
	lift_wheel_init->motor_chassis[0].chassis_motor_measure = get_Motor_Measure_Point(4);
	lift_wheel_init->motor_chassis[1].chassis_motor_measure = get_Motor_Measure_Point(5);
	
	//获取遥控指针 
	lift_wheel_init->chassis_RC = get_remote_control_point();
	
	//初始化底盘速度环PID 
	//pmax imax 
	PID_Init(&lift_wheel_init->motor_speed_pid[0], PID_POSITION, motor_speed_pid, 10000, 0);
	PID_Init(&lift_wheel_init->motor_speed_pid[1], PID_POSITION, motor_speed_pid, 10000, 0);
	
	//更新一下数据
  lift_wheel_feedback_update(lift_wheel_init);
}

//登岛轮数据更新
void lift_wheel_feedback_update(chassis_move_t *lift_wheel_update)
{
	lift_wheel_update->motor_chassis[0].speed = lift_wheel_update->motor_chassis[0].chassis_motor_measure->filter_rate / 19.0f;
	lift_wheel_update->motor_chassis[1].speed = lift_wheel_update->motor_chassis[1].chassis_motor_measure->filter_rate / 19.0f;
}

static uint32_t lift_wheel_system_time = 0;
//底盘控制PID计算
void lift_wheel_control_loop(chassis_move_t *lift_wheel_control)
{
	//更新电机数据
	lift_wheel_feedback_update(lift_wheel_control);
	//遥控右边拨杆中间挡为键盘模式
	if(lift_wheel_control->chassis_RC->rc.s[0] == 3)
	{
		//W和S前进
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
	//遥控拨杆其他挡位为遥控模式
	else
	{
		//前进为遥控右边通道0，左右为遥控右边通道1
		lift_wheel_control->vx =  lift_wheel_control->chassis_RC->rc.ch[1] * 30.0f/660.0f;
	}
	
	//获取当前系统时间
	lift_wheel_system_time = xTaskGetTickCount();
	
	//如果底盘系统当前时间减去进入遥控中断当前时间，说明没有收到遥控信号
	if((lift_wheel_system_time - lift_wheel_control->chassis_RC->time) > 88)
	{
		lift_wheel_control->vx= 0;
	}
	
	//底盘速度设定
	lift_wheel_control->motor_chassis[0].speed_set = -lift_wheel_control->vx;
	lift_wheel_control->motor_chassis[1].speed_set =  lift_wheel_control->vx; 
	//计算PID

	PID_Calc(&lift_wheel_control->motor_speed_pid[0], lift_wheel_control->motor_chassis[0].speed, lift_wheel_control->motor_chassis[0].speed_set);
	PID_Calc(&lift_wheel_control->motor_speed_pid[1], lift_wheel_control->motor_chassis[1].speed, lift_wheel_control->motor_chassis[1].speed_set);
	
	//赋值电流值
	lift_wheel_control->motor_chassis[0].give_current = (int16_t)(lift_wheel_control->motor_speed_pid[0].out);
	lift_wheel_control->motor_chassis[1].give_current = (int16_t)(lift_wheel_control->motor_speed_pid[1].out);
	//printf("%d\r\n",chassis_control->chassis_RC->rc.s[0]);
}
