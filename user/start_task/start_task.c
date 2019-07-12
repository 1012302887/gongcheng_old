#include "start_task.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "user_task.h"
#include "lift_task.h"
#include "chassis_task.h"

//启动任务句柄
#define START_TASK_PRIO 1
#define START_STK_SIZE 512
static TaskHandle_t StartTask_Handler;

//用户任务句柄
#define User_TASK_PRIO 4
#define User_STK_SIZE 256
static TaskHandle_t UserTask_Handler;

//升降任务句柄
#define Lift_TASK_PRIO 17
#define Lift_STK_SIZE 512
static TaskHandle_t liftTask_Handler;

//底盘任务句柄
#define Chassis_TASK_PRIO 18
#define Chassis_STK_SIZE 512
TaskHandle_t ChassisTask_Handler;

void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();
	
	 //升降任务
	 xTaskCreate((TaskFunction_t)lift_task,
                (const char *)"LiftTask",
                (uint16_t)Lift_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)Lift_TASK_PRIO,
                (TaskHandle_t *)&liftTask_Handler);
//	
	 //底盘任务
								xTaskCreate((TaskFunction_t)chassis_task,
                (const char *)"ChassisTask",
                (uint16_t)Chassis_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)Chassis_TASK_PRIO,
                (TaskHandle_t *)&ChassisTask_Handler);
								
		//用户任务
		xTaskCreate((TaskFunction_t)UserTask,
                (const char *)"UserTask",
                (uint16_t)User_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)User_TASK_PRIO,
                (TaskHandle_t *)&UserTask_Handler);

    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}

void startTast(void)
{
    xTaskCreate((TaskFunction_t)start_task,          //任务函数
                (const char *)"start_task",          //任务名称
                (uint16_t)START_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数
                (UBaseType_t)START_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&StartTask_Handler); //任务句柄
}
