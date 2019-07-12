#include "led.h"
#include "stm32f4xx_hal.h"
 /**TIM9 GPIO Configuration    
    PE5     ------> TIM9_CH1   ÂÌµÆ
    PE6     ------> TIM9_CH2 	 ºìµÆ
    */	
		
void LED_Init(void)
{
	TIM_OC_InitTypeDef sConfigOC;
	TIM_HandleTypeDef htim9;
	
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 168-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 1000;			//1000ms
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(&htim9);
	
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
 
	HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2);
		
	HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);  //ÂÌµÆ
	HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);  //ºìµÆ
}

/* TIM4 init function */
void TIM4_Init(void)//¿ØÖÆ¶æ»ú
{
  TIM_OC_InitTypeDef sConfigOC;
	TIM_HandleTypeDef htim4;
	
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 20000;			//20ms
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(&htim4);
	 
	//1900 2620 2250
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 2250;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
 
	HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);
	
  HAL_TIM_PWM_MspInit(&htim4);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1); 
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	__HAL_RCC_TIM4_CLK_ENABLE();
	__HAL_RCC_TIM9_CLK_ENABLE();	
  GPIO_InitTypeDef GPIO_InitStruct;
	if(htim->Instance==TIM4)
  {
    /**TIM4 GPIO Configuration    
    PD12     ------> TIM4_CH1
    PD13     ------> TIM4_CH2
    PD14     ------> TIM4_CH3
    PD15     ------> TIM4_CH4 
    */
		__HAL_RCC_GPIOD_CLK_ENABLE();	
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  }
  else if(htim->Instance==TIM9)
  {
    /**TIM9 GPIO Configuration    
    PE5     ------> TIM9_CH1   ÂÌµÆ
    PE6     ------> TIM9_CH2 	 ºìµÆ
    */
		__HAL_RCC_GPIOE_CLK_ENABLE();	
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  }
}

//¿ØÖÆºìµÆÁÁ¶È
void GREEN_LED(uint16_t bright)
{
	TIM9->CCR1 = bright;
}

//¿ØÖÆÂÌµÆÁÁ¶È
void RED_LED(uint16_t bright)
{
	TIM9->CCR2 = bright;
}
