#include "main.h"
#include "stm32f1xx_hal.h"
#include "gpio.h"

#ifndef SERVOMOTEUR_H
#define SERVOMOTEUR_H
void set_angle(float angle, TIM_HandleTypeDef *timer,uint32_t channel, int min, int max);

#endif
