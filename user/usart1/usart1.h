#ifndef USART1_H
#define USART1_H
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart1;

void USART1_Init(void);
void Ni_Ming(uint8_t fun,float Pid_ref1,float Pid_ref2,float Pid_ref3,float Pid_ref4);
#endif
