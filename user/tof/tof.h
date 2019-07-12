#ifndef TOF_H
#define TOF_H

#include "stm32f4xx_hal.h"

typedef struct
{
	int32_t tof_h;   //tof����
	int32_t str; //tof�ź�ǿ��
}tof_data_t;

extern UART_HandleTypeDef huart4;

extern DMA_HandleTypeDef  UART4RxDMA_Handler;

//��ʼ�����ش���
void Tof_Init(void);

//����tof������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
const tof_data_t *get_tof_Info_Measure_Point(void);

#endif
