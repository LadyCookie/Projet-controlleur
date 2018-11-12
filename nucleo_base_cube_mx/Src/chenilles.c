#include "chenilles.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include "gpio.h"

TIM_HandleTypeDef htim4;

int nb=25;
int active=0;
int sens=-1;
/**
- Horologe utilisé : TIM4
- Channel utilisé : 
	- Avancer
		- CC1 (PB6) 
		- CC2 (PB7)
	
	- Reculer 
		- CC3 (PB8)
		- CC4 (PB9)
**/
void TIM4_IRQHandler(void){
	HAL_TIM_IRQHandler(&htim4);
}

void Avancer(){
	sens=0;
	nb=0;
}
void Reculer(){
	sens=1;
	nb=0;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	active=(active+1)%3;
	if  (active != 0){
		return;
	}
	nb+=1;
	if (nb<=5){
		Set_power(10.0*(float)nb);
	}else{
		if (nb<=15){
			Set_power(50.0);
		}else{
			if (nb<=20){
				Set_power(10.0*(float)(20-nb));
			}else{
				sens=-1;
				Set_power(0.0);
			}
		}
	}
}

/** Initialize GPIOD PIN 0 for alternate output push-pull */
void Init_track(){
	// Enable the TIM interface clock
	__HAL_RCC_TIM4_CLK_ENABLE();
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 143;
	htim4.Init.Period = 9999;
	HAL_TIM_PWM_MspInit(&htim4);
	
	TIM4->DIER|=0x1;
	HAL_NVIC_EnableIRQ(TIM4_IRQn);
	HAL_NVIC_SetPriority(TIM4_IRQn,2,3);

	// Configure the TIM
	HAL_TIM_PWM_Init(&htim4);
	
	/** PB6 **/
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	/** PB7 **/
	GPIO_InitStruct.Pin = GPIO_PIN_7;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	/** PB8 **/
	GPIO_InitStruct.Pin = GPIO_PIN_8;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/** PB9 **/
	GPIO_InitStruct.Pin = GPIO_PIN_9;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	
	TIM_OC_InitTypeDef init_pwm;
	init_pwm.OCMode = TIM_OCMODE_PWM1;
	init_pwm.Pulse = 0;
	
	HAL_TIM_PWM_ConfigChannel(&htim4, &init_pwm, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&htim4, &init_pwm, TIM_CHANNEL_2);
	HAL_TIM_PWM_ConfigChannel(&htim4, &init_pwm, TIM_CHANNEL_3);
	HAL_TIM_PWM_ConfigChannel(&htim4, &init_pwm, TIM_CHANNEL_4);

	// Activation of the TIM peripheral
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);

	// Activation de l'interruption au niveau du timer (p516)
	/*HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_4);*/
}

void Set_power(float percent){

	if (sens == 0){
		/** Avancer **/
		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,percent*100);
		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2,percent*100);
		
		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,0);
		__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,0);

	}else{
		if (sens == 1){
			/** Reculer **/
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,0);
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2,0);

			
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,percent*100);
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,percent*100);
		}else{
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,0);
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2,0);
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,0);
			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_4,0);
		}
	}
	// Activation of the TIM peripheral
	HAL_TIM_Base_Start(&htim4);
}
