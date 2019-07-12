#ifndef LED_H
#define LED_H

#include "stm32f4xx_hal.h"

void LED_Init(void);
void GREEN_LED(uint16_t bright);
void RED_LED(uint16_t bright);
void TIM4_Init(void);
#endif
