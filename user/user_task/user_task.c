#include "user_task.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "led.h"

uint32_t UserTaskStack;

void UserTask(void *pvParameters)
{
	while (1)
	{
		RED_LED(500);
		GREEN_LED(0);
		vTaskDelay(500);
		RED_LED(0);
		GREEN_LED(500);
		vTaskDelay(500);
		UserTaskStack = uxTaskGetStackHighWaterMark(NULL);
	}
}
