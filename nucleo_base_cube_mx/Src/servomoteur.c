#include "servomoteur.h"
//int max=1950; //1472   pour le bras
//int min=450; //352   pour le bras

void set_angle(float angle, TIM_HandleTypeDef *timer,uint32_t channel, int min, int max){
	//tick = (float)(timer->Init.Prescaler) / 72000000.;
	//duree_pulse = (1. + angle /100. )/1000.;
	//pulse = (duree_pulse / tick);
	
	//float per = 1. +  (angle / 100.);
	//pulse = timer->Init.Period * (per / 10.);
	
	 //pulse = ((angle*(2)) * 72*1000000) /(timer->Init.Period * timer->Init.Prescaler); //comparison = angle%age * periode 
	
	int pulsemax = ((max-min) * (angle / 100)) + min;
	
	__HAL_TIM_SET_COMPARE(timer,channel ,pulsemax);
}
	
