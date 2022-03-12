/*
 * LIS2DS12.c
 *
 *  Created on: Jul 20, 2021
 *      Author: David
 */

#include "stdint.h"
#include "main.h"
#include "LIS2DS12.h"

I2C_HandleTypeDef hi2c2;

static const uint8_t ADDR = 0x1E << 1; // 8 bit address. The LSB is for read(1) and write(0)

void LIS_write_byte(uint8_t reg_addr, uint8_t data){
	uint8_t buf[2] = {reg_addr, data};
	HAL_I2C_Master_Transmit(&hi2c2, ADDR, buf, 2, 10);
}

void LIS_read(uint8_t reg_addr, uint8_t *data, uint16_t len){
	HAL_I2C_Master_Transmit(&hi2c2, ADDR, &reg_addr, 1, 10);
	HAL_I2C_Master_Receive(&hi2c2, ADDR | 0x01, data, len, 10);
}

void LIS_init(){
	LIS_write_byte(CTRL1, 0x30); // sets ODR to 50 Hz
}

void LIS_acc(LISrawData* raw_data){
	uint8_t data[6];
	LIS_read(OUT_X_L, data, 6);
	raw_data->x = (int16_t) ((data[1] << 8) | data[0]);
	raw_data->y = (int16_t) ((data[3] << 8) | data[2]);
	raw_data->z = (int16_t) ((data[5] << 8) | data[4]);
}

