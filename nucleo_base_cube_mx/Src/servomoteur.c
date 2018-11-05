#include "servomoteur.h"
float pulse;
float tick;
float duree_pulse;

int max=1472;
int min=352;

void set_angle(float angle, TIM_HandleTypeDef *timer,uint32_t channel){
	//tick = (float)(timer->Init.Prescaler) / 72000000.;
	//duree_pulse = (1. + angle /100. )/1000.;
	//pulse = (duree_pulse / tick);
	
	//float per = 1. +  (angle / 100.);
	//pulse = timer->Init.Period * (per / 10.);
	
	 //pulse = ((angle*(2)) * 72*1000000) /(timer->Init.Period * timer->Init.Prescaler); //comparison = angle%age * periode 
	
	pulse = ((max-min) * (angle / 100)) + min;
	
	__HAL_TIM_SET_COMPARE(timer,channel ,pulse);
	
}
	
