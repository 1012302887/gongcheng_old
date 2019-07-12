#include "put_out.h"
#include "stm32f4xx_hal.h"

void Put_Out_Init(void)
{
 GPIO_InitTypeDef GPIO_Initure;
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();			
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	GPIO_Initure.Pin=GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5; 
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //GPIO_MODE_OUTPUT_PP
	GPIO_Initure.Pull=GPIO_PULLDOWN;          
	GPIO_Initure.Speed=GPIO_SPEED_HIGH;     
	//HAL_GPIO_Init(GPIOC,&GPIO_Initure);    
	
	GPIO_Initure.Pin=GPIO_PIN_0 | GPIO_PIN_1; 
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  
	GPIO_Initure.Pull=GPIO_PULLUP;          
	GPIO_Initure.Speed=GPIO_SPEED_HIGH;     
	HAL_GPIO_Init(GPIOB,&GPIO_Initure);
	
	GPIO_Initure.Pin=GPIO_PIN_2 | GPIO_PIN_3; 
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  
	GPIO_Initure.Pull=GPIO_PULLUP;          
	GPIO_Initure.Speed=GPIO_SPEED_HIGH;     
	//HAL_GPIO_Init(GPIOA,&GPIO_Initure);

//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);	 //�ϳ�     PC0
//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);	 //��ȡ     PC1
//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);	 //����     PC2
//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);	 //����     PC3
//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);  //����     PC4
//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);  //һ����   PC5
//	
//	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);  //����     PB0
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);  //ǰ��     PB1
//	
//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);  //����     PA2
//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);  //�������� PA3
	
	//����Թܳ�ʼ��
//  GPIO_Initure.Pin=GPIO_PIN_1 | GPIO_PIN_0;    //PE1 PE0
//	GPIO_Initure.Mode=GPIO_MODE_INPUT;      		 
//	GPIO_Initure.Pull=GPIO_PULLDOWN;       			 
//	GPIO_Initure.Speed=GPIO_SPEED_HIGH;     		
//	HAL_GPIO_Init(GPIOE,&GPIO_Initure);
}
