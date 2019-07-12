#ifndef CAN_H
#define CAN_H

#include "stm32f4xx_hal.h"

extern CAN_HandleTypeDef  hcan1;
extern CAN_HandleTypeDef  hcan2;

extern void CAN1_Init(void);
extern void CAN2_Init(void);
#endif
