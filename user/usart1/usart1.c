#include "usart1.h"
#include "stm32f4xx_hal.h"
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)	
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	USART1->DR = (uint8_t) ch;      
	return ch;
}
#endif 

UART_HandleTypeDef huart1;

DMA_HandleTypeDef  UART2RxDMA_Handler;
DMA_HandleTypeDef  UART4RxDMA_Handler;

//���ڵ���
void USART1_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	HAL_UART_Init(&huart1);
}

//ֻ����һ�����ڻص�����
void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(uartHandle->Instance==USART1)
  {
		__HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();  
    /* PA9     ------> USART1_TX
       PA10     ------> USART1_RX */
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);			
  }
	else if(uartHandle->Instance==USART2)//ң�ش���
  {
		__HAL_RCC_USART2_CLK_ENABLE();   
		__HAL_RCC_DMA1_CLK_ENABLE();
		__HAL_RCC_GPIOD_CLK_ENABLE();
	 /* PD5     ------> USART2_TX
			PD6     ------> USART2_RX */ 
		GPIO_InitStruct.Pin = GPIO_PIN_6;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
		
		//RX
		UART2RxDMA_Handler.Instance=DMA1_Stream5;                           
		UART2RxDMA_Handler.Init.Channel=DMA_CHANNEL_4;   									
		UART2RxDMA_Handler.Init.Direction = DMA_PERIPH_TO_MEMORY;
		UART2RxDMA_Handler.Init.PeriphInc = DMA_PINC_DISABLE;
		UART2RxDMA_Handler.Init.MemInc = DMA_MINC_ENABLE;
		UART2RxDMA_Handler.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		UART2RxDMA_Handler.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		UART2RxDMA_Handler.Init.Mode = DMA_CIRCULAR;
		UART2RxDMA_Handler.Init.Priority = DMA_PRIORITY_LOW;
		UART2RxDMA_Handler.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		HAL_DMA_Init(&UART2RxDMA_Handler);           
		
		__HAL_LINKDMA(uartHandle,hdmarx,UART2RxDMA_Handler);    
		
		HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(USART2_IRQn);				//ʹ��USART2�ж�ͨ��
	}
	else if(uartHandle->Instance==UART4)
  {
    __HAL_RCC_UART4_CLK_ENABLE(); 
		__HAL_RCC_DMA1_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();
   /* PA0     ------> UART4_TX
      PA1     ------> UART4_RX */  
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		
		//RX
		UART4RxDMA_Handler.Instance=DMA1_Stream2;                           
    UART4RxDMA_Handler.Init.Channel=DMA_CHANNEL_4;   									
    UART4RxDMA_Handler.Init.Direction = DMA_PERIPH_TO_MEMORY;
    UART4RxDMA_Handler.Init.PeriphInc = DMA_PINC_DISABLE;
    UART4RxDMA_Handler.Init.MemInc = DMA_MINC_ENABLE;
    UART4RxDMA_Handler.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    UART4RxDMA_Handler.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    UART4RxDMA_Handler.Init.Mode = DMA_CIRCULAR;
    UART4RxDMA_Handler.Init.Priority = DMA_PRIORITY_LOW;
    UART4RxDMA_Handler.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&UART4RxDMA_Handler);           
		
		__HAL_LINKDMA(uartHandle,hdmarx,UART4RxDMA_Handler);    
		
		HAL_NVIC_SetPriority(UART4_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(UART4_IRQn);
  }
}

//������λ������Э��
void Ni_Ming(uint8_t fun,float Pid_ref1,float Pid_ref2,float Pid_ref3,float Pid_ref4)
{
	uint8_t send_buf[21];
  unsigned char *p1,*p2,*p3,*p4;
  p1=(unsigned char *)&Pid_ref1;
	p2=(unsigned char *)&Pid_ref2;
	p3=(unsigned char *)&Pid_ref3;
	p4=(unsigned char *)&Pid_ref4;
	
	send_buf[0]=0XAA;	//֡ͷ
	send_buf[1]=0XAA;	//֡ͷ
	send_buf[2]=fun;	//������
	send_buf[3]=16;	//���ݳ���
  send_buf[4]=(unsigned char)(*(p1+3));
  send_buf[5]=(unsigned char)(*(p1+2));
  send_buf[6]=(unsigned char)(*(p1+1));
  send_buf[7]=(unsigned char)(*(p1+0));
	send_buf[8]=(unsigned char)(*(p2+3));
	send_buf[9]=(unsigned char)(*(p2+2));
	send_buf[10]=(unsigned char)(*(p2+1));
	send_buf[11]=(unsigned char)(*(p2+0));
	send_buf[12]=(unsigned char)(*(p3+3));
	send_buf[13]=(unsigned char)(*(p3+2));
	send_buf[14]=(unsigned char)(*(p3+1));
  send_buf[15]=(unsigned char)(*(p3+0));
	send_buf[16]=(unsigned char)(*(p4+3));
  send_buf[17]=(unsigned char)(*(p4+2));
  send_buf[18]=(unsigned char)(*(p4+1));
  send_buf[19]=(unsigned char)(*(p4+0));
	send_buf[20]=0;
	for(uint8_t i=0;i<20;i++)send_buf[20]+=send_buf[i];	//����У���
	for(uint8_t i=0;i<21;i++)
	{
		while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)==RESET){}; 
    USART1->DR=send_buf[i];
	}
}
