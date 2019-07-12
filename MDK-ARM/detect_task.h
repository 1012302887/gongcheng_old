#ifndef DETECT_Task_H
#define DETECT_Task_H
#include "stm32f4xx_hal.h"
typedef __packed struct
{
    uint32_t regTime;
    uint32_t Losttime;
    uint32_t overtime;
		uint8_t  lost_flag;
} error_t;
extern void DetectTask(void *pvParameters);
extern void DetectHook(uint8_t toe);
extern uint8_t Detect_Judeg(uint8_t toe);
extern void DetectInit(uint16_t i,uint32_t over_time);
#endif
