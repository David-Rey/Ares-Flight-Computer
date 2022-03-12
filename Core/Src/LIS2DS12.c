/*
 * LIS2DS12.c
 *
 *  Created on: Aug 2, 2021
 *      Author: David
 */

#include "stdint.h"
#include "main.h"
#include "LIS2DS12.h"

SPI_HandleTypeDef hspi3;
#define LIS_SPI &hspi3

void LIS_enable(){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, RESET);
}

void LIS_disable(){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, SET);
}

void LIS_write_byte(uint8_t reg_addr, uint8_t data){
	LIS_enable();
	HAL_SPI_Transmit(LIS_SPI, &reg_addr, 1, 10);
	HAL_SPI_Receive(LIS_SPI, &data, 1, 10);
	LIS_disable();
}

void LIS_read(uint8_t reg_addr, uint8_t *data, uint16_t len){
	LIS_enable();
	reg_addr = reg_addr | 0x80;
	HAL_SPI_Transmit(LIS_SPI, &reg_addr, 1, 10);
	HAL_SPI_Receive(LIS_SPI, data, len, 10);
	LIS_disable();
}

uint8_t LIS_init(){
	uint8_t id;
	LIS_read(WHO_AM_I, &id, 1);
	if (id == 0x43){
		LIS_write_byte(CTRL1, 0x30);
		return 1;
	}
	return 0;
}

void LIS_acc(LISrawData* raw_data){
	uint8_t data[6];
	LIS_read(OUT_X_L, data, 6);
	raw_data->x = (int16_t) ((data[1] << 8) | data[0]);
	raw_data->y = (int16_t) ((data[3] << 8) | data[2]);
	raw_data->z = (int16_t) ((data[5] << 8) | data[4]);
}
