#include "can.h"
#include "stm32f4xx_hal.h"

CAN_HandleTypeDef  hcan1;
CAN_HandleTypeDef  hcan2;

void CAN1_Init(void)
{	
	CAN_FilterTypeDef  hcan1_filter;
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
	HAL_CAN_Init(&hcan1);
	
	hcan1_filter.FilterIdHigh=0X0000;     //32位ID
  hcan1_filter.FilterIdLow=0X0000;
  hcan1_filter.FilterMaskIdHigh=0X0000; //32位MASK
  hcan1_filter.FilterMaskIdLow=0X0000;  
  hcan1_filter.FilterFIFOAssignment=CAN_FILTER_FIFO0;//过滤器0关联到FIFO0
  hcan1_filter.FilterBank=0;          //过滤器0
  hcan1_filter.FilterMode=CAN_FILTERMODE_IDMASK;
  hcan1_filter.FilterScale=CAN_FILTERSCALE_32BIT;
  hcan1_filter.FilterActivation=ENABLE; //激活滤波器0
  hcan1_filter.SlaveStartFilterBank=14;
	
  HAL_CAN_ConfigFilter(&hcan1,&hcan1_filter);
	
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1,   CAN_IT_RX_FIFO0_MSG_PENDING);
}

void CAN2_Init(void)
{
	CAN_FilterTypeDef  hcan2_filter;
	hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
	HAL_CAN_Init(&hcan2);
	
	hcan2_filter.FilterIdHigh=0X0000;     
  hcan2_filter.FilterIdLow=0X0000;
  hcan2_filter.FilterMaskIdHigh=0X0000; 
  hcan2_filter.FilterMaskIdLow=0X0000;  
  hcan2_filter.FilterFIFOAssignment=CAN_FILTER_FIFO0;
  hcan2_filter.FilterBank=14;         
  hcan2_filter.FilterMode=CAN_FILTERMODE_IDMASK;
  hcan2_filter.FilterScale=CAN_FILTERSCALE_32BIT;
  hcan2_filter.FilterActivation=ENABLE; 
  hcan2_filter.SlaveStartFilterBank=14;
  HAL_CAN_ConfigFilter(&hcan2,&hcan2_filter);
	
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2,   CAN_IT_RX_FIFO0_MSG_PENDING);
}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(canHandle->Instance==CAN1)
  {
    __HAL_RCC_CAN1_CLK_ENABLE();
		__HAL_RCC_GPIOD_CLK_ENABLE();		
    /**CAN1 GPIO Configuration    
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn); 
  }
  else if(canHandle->Instance==CAN2)
  {
    __HAL_RCC_CAN2_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();	
    /**CAN2 GPIO Configuration    
    PB12     ------> CAN2_RX
    PB6     ------> CAN2_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn); 
  }
}

//CAN1接受中断
void CAN1_RX0_IRQHandler(void)
{
	HAL_CAN_IRQHandler(&hcan1);
}

//CAN2接收中断
void CAN2_RX0_IRQHandler(void)
{
	HAL_CAN_IRQHandler(&hcan2);
}
