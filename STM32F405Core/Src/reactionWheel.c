/*
 * reactionWheel.c
 *
 *  Created on: Aug 6, 2021
 *      Author: David
 */

#include "main.h";

TIM_HandleTypeDef htim3;

void RW_counterCW(){
	HAL_GPIO_WritePin(RW2_GPIO_Port, RW2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RW1_GPIO_Port, RW1_Pin, GPIO_PIN_SET);
}

void RW_CW(){
	HAL_GPIO_WritePin(RW2_GPIO_Port, RW2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RW1_GPIO_Port, RW1_Pin, GPIO_PIN_RESET);
}

void RW_init(){
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	htim3.Instance->CCR1 = 100; // off
	RW_counterCW();
}

void RW_set_intensity(uint8_t intensity){ // 0 < intensity < 100
	htim3.Instance->CCR1 = 100 - intensity;
}

void set_direction(int8_t dir){ // -100 < dir < 100
	if (dir > 0){
		RW_counterCW();
		RW_set_intensity(dir);
	}else{
		RW_CW();
		RW_set_intensity(abs(dir));
	}
}

/*
  uint8_t k = 0;
  while (1){
	  if (k > 100){
		  k = 0;
	  }
	  k++;
	  htim3.Instance->CCR1 = k;
	  HAL_Delay(100);
  }
  */
