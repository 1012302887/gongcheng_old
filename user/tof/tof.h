#ifndef TOF_H
#define TOF_H

#include "stm32f4xx_hal.h"

typedef struct
{
	int32_t tof_h;   //tof距离
	int32_t str; //tof信号强度
}tof_data_t;

extern UART_HandleTypeDef huart4;

extern DMA_HandleTypeDef  UART4RxDMA_Handler;

//初始化工控串口
void Tof_Init(void);

//返回tof变量地址，通过指针方式获取原始数据
const tof_data_t *get_tof_Info_Measure_Point(void);

#endif
