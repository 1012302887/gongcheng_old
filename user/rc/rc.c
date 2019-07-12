#include "rc.h"
#include "stm32f4xx_hal.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "chassis_task.h"

UART_HandleTypeDef huart2;
//ң�������Ʊ���
static RC_ctrl_t rc_ctrl;
//����ԭʼ���ݣ�Ϊ18���ֽڣ�����36���ֽڳ��ȣ���ֹDMA����Խ��
static uint8_t SBUS_rx_buf[2][36];

//����ң�������Ʊ�����ͨ��ָ�봫�ݷ�ʽ������Ϣ
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

// USART2 init function                 ң�ؽ��ܴ��ڣ�żУ�飬�����ʣ�100000
void USART2_Init(void)
{
  huart2.Instance = USART2;
	huart2.Init.BaudRate = 100000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;	
  HAL_UART_Init(&huart2);
	
	__HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart2,UART_IT_IDLE);
	SET_BIT(huart2.Instance->CR1, USART_CR1_IDLEIE);
	HAL_UART_Receive_DMA(&huart2, (uint8_t *)SBUS_rx_buf, 128);	
}

//DMA1 STREAM5
void USART2_IRQHandler(void)
{
	if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE) &&   /*��ȡUSART_SR�Ĵ�����IDLEλ*/
		  __HAL_UART_GET_IT_SOURCE(&huart2, UART_IT_IDLE))
	{
		uint16_t tmp = huart2.Instance->SR;  //��⵽������·��Ӳ����1�������0 ������USART_DR��USART_SR��
		tmp = huart2.Instance->DR;
		tmp--;
		CLEAR_BIT(huart2.Instance->SR, USART_SR_IDLE); //IDLEλ����
		__HAL_DMA_DISABLE(huart2.hdmarx);
		uint16_t temp = huart2.hdmarx->Instance->NDTR;  //Ҫ�����ʣ����������
		if((128 - temp) == 18)//�������鳤�� - �����ʣ����������
		{
			//����ң��������
			SBUS_TO_RC(SBUS_rx_buf[0], &rc_ctrl);
		}
		HAL_UART_Receive_DMA(&huart2, (uint8_t *)SBUS_rx_buf, 128);
		SET_BIT(huart2.Instance->CR1, USART_CR1_IDLEIE);
		DMA1->HIFCR = DMA_FLAG_DMEIF1_5 | DMA_FLAG_FEIF1_5 | DMA_FLAG_HTIF1_5 | DMA_FLAG_TCIF1_5 | DMA_FLAG_TEIF1_5;//DMA�ж���0
		__HAL_DMA_SET_COUNTER(huart2.hdmarx, 128); //����NDTRλ
		__HAL_DMA_ENABLE(huart2.hdmarx);
	} 
}

void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
	if (sbus_buf == NULL || rc_ctrl == NULL)
	{
			return;
	}

	rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
	rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
	rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
											 (sbus_buf[4] << 10)) &
											0x07ff;
	rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
	rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
	rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
	rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
	rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
	rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
	rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
	rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
	rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
	rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

	rc_ctrl->rc.ch[0] -= (uint16_t)1024;
	rc_ctrl->rc.ch[1] -= (uint16_t)1024;
	rc_ctrl->rc.ch[2] -= (uint16_t)1024;
	rc_ctrl->rc.ch[3] -= (uint16_t)1024;
	rc_ctrl->rc.ch[4] -= (uint16_t)1024;
	
	/* prevent remote control zero deviation */
  if(rc_ctrl->rc.ch[0] <= 10 && rc_ctrl->rc.ch[0] >= -10)
    rc_ctrl->rc.ch[0] = 0;
  if(rc_ctrl->rc.ch[1] <= 10 && rc_ctrl->rc.ch[1] >= -10)
    rc_ctrl->rc.ch[1] = 0;
  if(rc_ctrl->rc.ch[2] <= 10 && rc_ctrl->rc.ch[2] >= -10)
    rc_ctrl->rc.ch[2] = 0;
  if(rc_ctrl->rc.ch[3] <= 10 && rc_ctrl->rc.ch[3] >= -10)
    rc_ctrl->rc.ch[3] = 0;

	
	/* ң������ť
		1                  0
				 3				 1
			2     2		0     0
				 3				 1
													*/
	if(get_chassis_state())
	{
		rc_ctrl->time = xTaskGetTickCountFromISR();
	}
}
