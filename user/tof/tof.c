#include "tof.h"

UART_HandleTypeDef huart4;
static tof_data_t tof_data;
static uint8_t Tof_1_data[1][128];

void Tof_Init(void)
{
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&huart4);
	
	__HAL_UART_ENABLE_IT(&huart4,UART_IT_RXNE);
	__HAL_UART_ENABLE_IT(&huart4,UART_IT_IDLE);
	SET_BIT(huart4.Instance->CR1, USART_CR1_IDLEIE);
	HAL_UART_Receive_DMA(&huart4, (uint8_t *)Tof_1_data, 128);	
}

static uint32_t xiuer_r = 0;
void UART4_IRQHandler(void)
{
	if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_IDLE) && 
      __HAL_UART_GET_IT_SOURCE(&huart4, UART_IT_IDLE))
    {
			
      uint16_t tmp = huart4.Instance->DR;
      tmp = huart4.Instance->SR;
      tmp--;
      CLEAR_BIT(huart4.Instance->SR, USART_SR_IDLE);
			__HAL_DMA_DISABLE(huart4.hdmarx);
			
     	uint16_t  temp = huart4.hdmarx->Instance->NDTR;  
			if((128 - temp) == 9)//传输数组长度 - 传输的剩余数据项数
			{
				if(Tof_1_data[0][0] == 0x59)
				{
					if(Tof_1_data[0][1] == 0x59)
					{
						xiuer_r = Tof_1_data[0][5]<<8 | Tof_1_data[0][4];
						if((xiuer_r > 100) && (xiuer_r != 65535))
						tof_data.tof_h = Tof_1_data[0][3]<<8 | Tof_1_data[0][2];
						//printf("%d\r\n",tof_data.dis_r);
					}
				}
			}
		}
		HAL_UART_Receive_DMA(&huart4, (uint8_t *)Tof_1_data, 128);
		SET_BIT(huart4.Instance->CR1, USART_CR1_IDLEIE);
    DMA1->LIFCR = DMA_FLAG_DMEIF2_6 | DMA_FLAG_FEIF2_6 | DMA_FLAG_HTIF2_6 | DMA_FLAG_TCIF2_6 | DMA_FLAG_TEIF2_6; 
		__HAL_DMA_SET_COUNTER(huart4.hdmarx, 128);
		__HAL_DMA_ENABLE(huart4.hdmarx);
}

//返回tof变量地址，通过指针方式获取原始数据
const tof_data_t *get_tof_Info_Measure_Point(void)
{
	return &tof_data;
}
