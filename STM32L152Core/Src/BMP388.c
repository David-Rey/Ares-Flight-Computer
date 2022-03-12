/*
 * BMP388.c
 *
 *  Created on: Jul 11, 2021
 *      Author: David
 */

#include "stdint.h"
#include "BMP388.h"
#include "main.h"
#include "math.h"

SPI_HandleTypeDef hspi2;
#define BMP_SPI &hspi2

uint8_t IF_CONFIG_VAL		= 0x00;
uint8_t PWR_CTRL_VAL		= 0x33;
uint8_t OSR_VAL				= 0x03;
uint8_t ODR_VAL				= 0x02;

void BMP388_enable(){
	HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_RESET);
}

void BMP388_disable(){
	HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_SET);
}

void BMP_write(uint8_t reg_addr, uint8_t *data, uint16_t len){
	BMP388_enable();
	HAL_SPI_Transmit(BMP_SPI, &reg_addr, 1, 10);
	for (int i = 0; i < len; i++){
		HAL_SPI_Transmit(BMP_SPI, data, 1, 10);
		data++;
	}
	BMP388_disable();
}

void BMP_read(uint8_t reg_addr, uint8_t *data, uint16_t len){
	BMP388_enable();
	reg_addr = reg_addr | 0x80;
	uint8_t zero_byte = 0x00;
	HAL_SPI_Transmit(BMP_SPI, &reg_addr, 1, 10);
	HAL_SPI_Transmit(BMP_SPI, &zero_byte, 1, 10);
	for (int i = 0; i < len; i++){
		HAL_SPI_TransmitReceive(BMP_SPI, &zero_byte, data, 1, 10);
		data++;
	}
	BMP388_disable();
}

uint8_t BMP_init(){
	uint8_t chip_id = 0x00;
	BMP_read(BMP388_CHIP_ID, &chip_id, 1);
	if (chip_id != 0x50){
		return 0;
	}
	BMP_write(BMP388_IF_CONFIG, &IF_CONFIG_VAL, 1); //uint8_t if_conf = 0x00; // sets 4 wire SPI
	BMP_write(BMP388_PWR_CTRL, &PWR_CTRL_VAL, 1); // sets mode to normal and enables pressure and temp.
	BMP_write(BMP388_OSR, &OSR_VAL, 1); // sets 8x oversample rate for pressure and 1x for temp.
	BMP_write(BMP388_ODR, &ODR_VAL, 1); // subdivision factor for pressure and temp.
	get_parms();
	return 1;
}

void BMP_raw_temp_press(uint32_t *temperature, uint32_t *pressure){
	uint8_t data[6];
	BMP_read(BMP388_DATA_0, data, 6);
	*temperature = (int32_t)data[5] << 16 | (int32_t)data[4] << 8 | (int32_t)data[3];
	*pressure = (int32_t)data[2] << 16 | (int32_t)data[1] << 8 | (int32_t)data[0];
}

void BMP_temp_press(float *temperature, float *pressure){
	uint8_t data[6];
	BMP_read(BMP388_DATA_0, data, 6);
	int32_t adcTemp = (int32_t)data[5] << 16 | (int32_t)data[4] << 8 | (int32_t)data[3];
	int32_t adcPres = (int32_t)data[2] << 16 | (int32_t)data[1] << 8 | (int32_t)data[0];
	*temperature = compensate_temperature((float)adcTemp);
	*pressure = compensate_pressure((float)adcPres, *temperature) / 100.0f;
}

void get_parms(){
	BMP_read(BMP388_TRIM_PARAMS, (uint8_t*)&params, 21);
	floatParams.param_T1 = (float)params.param_T1 / powf(2.0f, -8.0f); // Calculate the floating point trim parameters
	floatParams.param_T2 = (float)params.param_T2 / powf(2.0f, 30.0f);
	floatParams.param_T3 = (float)params.param_T3 / powf(2.0f, 48.0f);
	floatParams.param_P1 = ((float)params.param_P1 - powf(2.0f, 14.0f)) / powf(2.0f, 20.0f);
	floatParams.param_P2 = ((float)params.param_P2 - powf(2.0f, 14.0f)) / powf(2.0f, 29.0f);
	floatParams.param_P3 = (float)params.param_P3 / powf(2.0f, 32.0f);
	floatParams.param_P4 = (float)params.param_P4 / powf(2.0f, 37.0f);
	floatParams.param_P5 = (float)params.param_P5 / powf(2.0f, -3.0f);
	floatParams.param_P6 = (float)params.param_P6 / powf(2.0f, 6.0f);
	floatParams.param_P7 = (float)params.param_P7 / powf(2.0f, 8.0f);
	floatParams.param_P8 = (float)params.param_P8 / powf(2.0f, 15.0f);
	floatParams.param_P9 = (float)params.param_P9 / powf(2.0f, 48.0f);
	floatParams.param_P10 = (float)params.param_P10 / powf(2.0f, 48.0f);
	floatParams.param_P11 = (float)params.param_P11 / powf(2.0f, 65.0f);

}

float compensate_temperature(float uncompTemp){
	float partial_data1 = uncompTemp - floatParams.param_T1;
	float partial_data2 = partial_data1 * floatParams.param_T2;
	return partial_data2 + partial_data1 * partial_data1 * floatParams.param_T3;
}

float compensate_pressure(float uncompPress, float temp){
	float partial_data1 = floatParams.param_P6 * temp;
	float partial_data2 = floatParams.param_P7 * temp * temp;
	float partial_data3 = floatParams.param_P8 * temp * temp * temp;
	float partial_out1 = floatParams.param_P5 + partial_data1 + partial_data2 + partial_data3;
	partial_data1 = floatParams.param_P2 * temp;
	partial_data2 = floatParams.param_P3 * temp * temp;
	partial_data3 = floatParams.param_P4 * temp * temp * temp;
	float partial_out2 = uncompPress * (floatParams.param_P1 + partial_data1 + partial_data2 + partial_data3);
	partial_data1 = uncompPress * uncompPress;
	partial_data2 = floatParams.param_P9 + floatParams.param_P10 * temp;
	partial_data3 = partial_data1 * partial_data2;
	float partial_data4 = partial_data3 + uncompPress * uncompPress * uncompPress * floatParams.param_P11;
	return partial_out1 + partial_out2 + partial_data4;
}

/*
void BMX_write_byte(uint8_t reg_addr, uint8_t data){
	BMP388_enable();
	HAL_SPI_Transmit(BMP_SPI, &reg_addr, 1, 10);
	HAL_SPI_Transmit(BMP_SPI, &data, 1, 10);
	BMP388_disable();
}
*/
