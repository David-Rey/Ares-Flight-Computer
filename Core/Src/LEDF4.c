/*
 * LEDF4.c
 *
 *  Created on: Aug 2, 2021
 *      Author: David
 */

#include "main.h"

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

void LED_init(){
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_Delay(200);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_Delay(200);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_Delay(200);
	htim3.Instance->CCR2 = 100;
	htim8.Instance->CCR3 = 100;
	htim3.Instance->CCR4 = 100;
}

void RGB_set_intensity(uint8_t intensity){
	htim3.Instance->CCR2 = 100 - intensity;
	htim8.Instance->CCR3 = 100 - intensity;
	htim3.Instance->CCR4 = 100 - intensity;
}

void R_set_intensity(uint8_t intensity){
	htim3.Instance->CCR2 = 100 - intensity;
}

void G_set_intensity(uint8_t intensity){
	htim8.Instance->CCR3 = 100 - intensity;;
}

void B_set_intensity(uint8_t intensity){
	htim3.Instance->CCR4 = 100 - intensity;
}
