#include "start_task.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "user_task.h"
#include "lift_task.h"
#include "chassis_task.h"

//����������
#define START_TASK_PRIO 1
#define START_STK_SIZE 512
static TaskHandle_t StartTask_Handler;

//�û�������
#define User_TASK_PRIO 4
#define User_STK_SIZE 256
static TaskHandle_t UserTask_Handler;

//����������
#define Lift_TASK_PRIO 17
#define Lift_STK_SIZE 512
static TaskHandle_t liftTask_Handler;

//����������
#define Chassis_TASK_PRIO 18
#define Chassis_STK_SIZE 512
TaskHandle_t ChassisTask_Handler;

void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();
	
	 //��������
	 xTaskCreate((TaskFunction_t)lift_task,
                (const char *)"LiftTask",
                (uint16_t)Lift_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)Lift_TASK_PRIO,
                (TaskHandle_t *)&liftTask_Handler);
//	
	 //��������
								xTaskCreate((TaskFunction_t)chassis_task,
                (const char *)"ChassisTask",
                (uint16_t)Chassis_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)Chassis_TASK_PRIO,
                (TaskHandle_t *)&ChassisTask_Handler);
								
		//�û�����
		xTaskCreate((TaskFunction_t)UserTask,
                (const char *)"UserTask",
                (uint16_t)User_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)User_TASK_PRIO,
                (TaskHandle_t *)&UserTask_Handler);

    vTaskDelete(StartTask_Handler); //ɾ����ʼ����
    taskEXIT_CRITICAL();            //�˳��ٽ���
}

void startTast(void)
{
    xTaskCreate((TaskFunction_t)start_task,          //������
                (const char *)"start_task",          //��������
                (uint16_t)START_STK_SIZE,            //�����ջ��С
                (void *)NULL,                        //���ݸ��������Ĳ���
                (UBaseType_t)START_TASK_PRIO,        //�������ȼ�
                (TaskHandle_t *)&StartTask_Handler); //������
}
