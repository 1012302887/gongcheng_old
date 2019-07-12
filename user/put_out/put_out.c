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

//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);	 //拖车     PC0
//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);	 //夹取     PC1
//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);	 //后腿     PC2
//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);	 //给弹     PC3
//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);  //弹开     PC4
//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);  //一级伸   PC5
//	
//	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);  //左移     PB0
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);  //前轮     PB1
//	
//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);  //右移     PA2
//	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);  //二级伸缩 PA3
	
	//红外对管初始化
//  GPIO_Initure.Pin=GPIO_PIN_1 | GPIO_PIN_0;    //PE1 PE0
//	GPIO_Initure.Mode=GPIO_MODE_INPUT;      		 
//	GPIO_Initure.Pull=GPIO_PULLDOWN;       			 
//	GPIO_Initure.Speed=GPIO_SPEED_HIGH;     		
//	HAL_GPIO_Init(GPIOE,&GPIO_Initure);
}
