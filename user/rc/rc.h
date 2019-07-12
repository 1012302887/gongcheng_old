#ifndef RC_H
#define RC_H
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef  UART2RxDMA_Handler;

typedef __packed struct
{
	__packed struct
	{
					int16_t ch[5];
					char s[2];
	} rc;
	__packed struct
	{
					int16_t x;
					int16_t y;
					int16_t z;
					uint8_t press_l;
					uint8_t press_r;
	} mouse;
	__packed struct
	{
					uint16_t v;
	} key;

	uint32_t time;
} RC_ctrl_t;

void USART2_Init(void);
extern const RC_ctrl_t *get_remote_control_point(void);
void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
#endif
