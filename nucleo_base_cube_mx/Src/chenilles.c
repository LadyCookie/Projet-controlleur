#include "chenilles.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include "gpio.h"

TIM_HandleTypeDef htim4;

/** Initialize GPIOD PIN 0 for alternate output push-pull */
void Init_track(){

	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 143;
	htim4.Init.Period = 9999;
	
	HAL_TIM_PWM_MspInit(&htim4);
	
	// Enable the TIM interface clock
	__HAL_RCC_TIM4_CLK_ENABLE();

	// Configure the TIM
	HAL_TIM_PWM_Init(&htim4);
	
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitTypeDef GPIO_InitStruct2;
	GPIO_InitStruct2.Pin = GPIO_PIN_9;
  GPIO_InitStruct2.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct2.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct2);
	
	
	TIM_OC_InitTypeDef init_pwm;
	init_pwm.OCMode = TIM_OCMODE_PWM1;
	init_pwm.Pulse = 0;
	
	HAL_TIM_PWM_ConfigChannel(&htim4, &init_pwm, TIM_CHANNEL_3);
	HAL_TIM_PWM_ConfigChannel(&htim4, &init_pwm, TIM_CHANNEL_4);

	// Activation of the TIM peripheral
	HAL_TIM_Base_Start(&htim4);
	
	// Activation de l'interruption au niveau du timer (p516)
	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_4);
}

void Set_power(float percent){
	TIM_OC_InitTypeDef init_pwm;
	init_pwm.OCMode = TIM_OCMODE_PWM1;
	init_pwm.Pulse = percent*100;
	
	HAL_TIM_PWM_ConfigChannel(&htim4, &init_pwm, TIM_CHANNEL_3);
	HAL_TIM_PWM_ConfigChannel(&htim4, &init_pwm, TIM_CHANNEL_4);

	// Activation of the TIM peripheral
	HAL_TIM_Base_Start(&htim4);
	
	// Activation de l'interruption au niveau du timer (p516)
	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_4);
}
