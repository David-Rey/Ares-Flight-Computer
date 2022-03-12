/*
 * BMX160.c
 *
 *  Created on: Jul 22, 2021
 *      Author: David
 */

#include "stdint.h"
#include "BMX160.h"
#include "main.h"

SPI_HandleTypeDef hspi2;
#define BMX_SPI &hspi2

void BMX160_enable(){
	HAL_GPIO_WritePin(BMX_CS_GPIO_Port, BMX_CS_Pin, GPIO_PIN_RESET);
}

void BMX160_disable(){
	HAL_GPIO_WritePin(BMX_CS_GPIO_Port, BMX_CS_Pin, GPIO_PIN_SET);
}

void BMX_write_byte(uint8_t reg_addr, uint8_t data){
	BMX160_enable();
	HAL_SPI_Transmit(BMX_SPI, &reg_addr, 1, 10);
	HAL_SPI_Transmit(BMX_SPI, &data, 1, 10);
	BMX160_disable();
}

void BMX_read(uint8_t reg_addr, uint8_t *data, uint16_t len){
	BMX160_enable();
	reg_addr = reg_addr | 0x80;
	HAL_SPI_Transmit(BMX_SPI, &reg_addr, 1, 10);
	for (int i = 0; i < len; i++){
		HAL_SPI_Receive(BMX_SPI, data, 1, 10);
		data++;
	}
	BMX160_disable();
}

uint8_t BMX_init(){
	uint8_t chip_id = 0x00;
	BMX_read(CHIP_ID, &chip_id, 1);
	if (chip_id != 0xd8){
		return 0;
	}

	// set acc, gyro and mag into normal mode
	BMX_write_byte(CMD, 0x11);
	HAL_Delay(100);
	BMX_write_byte(CMD, 0x15);
	HAL_Delay(100);
	BMX_write_byte(CMD, 0x19);

	// acc config
	BMX_write_byte(ACC_CONF, 0x27); // set refresh to 50 Hz
	BMX_write_byte(ACC_RANGE, 0x03); // sets range to +-2g

	// gyro config
	BMX_write_byte(GYR_CONF, 0x27); // set refresh to 50 Hz
	BMX_write_byte(GYR_RANGE, 0x03); // sets sensitivity
	HAL_Delay(100);

	// mag congif
	BMX_write_byte(MAG_IF_0, 0x80);
	HAL_Delay(50);
	BMX_write_byte(MAG_IF_3, 0x01);
	BMX_write_byte(MAG_IF_2, 0x4B);
	BMX_write_byte(MAG_IF_3, 0x01);
	BMX_write_byte(MAG_IF_2, 0x51);
	BMX_write_byte(MAG_IF_3, 0x0E);
	BMX_write_byte(MAG_IF_2, 0x52);
	BMX_write_byte(MAG_IF_3, 0x02);
	BMX_write_byte(MAG_IF_2, 0x4C);
	BMX_write_byte(MAG_IF_1, 0x42);
	BMX_write_byte(MAG_CONF, 0x05);
	BMX_write_byte(MAG_IF_0, 0x00);
	HAL_Delay(50);

	// sets interrupt
	BMX_write_byte(INT_EN_1, 0x10); // enables the data ready interrupt
	BMX_write_byte(INT_OUT_CTRL, 0x0A); // enables the INT1 for output
	BMX_write_byte(INT_LATCH, 0x00);
	BMX_write_byte(INT_MAP_1, 0x80); // sets data ready interrupt to INT1
	return 1;
}

void BMX_sensor_data(BMXrawData* raw_data, uint8_t DATA_ADDR){
	uint8_t data[6];
	BMX_read(DATA_ADDR, data, 6);
	raw_data->x = (int16_t) ((data[1] << 8) | data[0]);
	raw_data->y = (int16_t) ((data[3] << 8) | data[2]);
	raw_data->z = (int16_t) ((data[5] << 8) | data[4]);
}

void BMX_all_sensor_data(BMXrawData* acc, BMXrawData* gyr, BMXrawData* mag){
	acc->x = 0; acc->y = 0; acc->z = 0;
	gyr->x = 0; gyr->y = 0; gyr->z = 0;
	mag->x = 0; mag->y = 0; mag->z = 0;
	BMX_sensor_data(mag, DATA_0);
	BMX_sensor_data(gyr, DATA_8);
	BMX_sensor_data(acc, DATA_14);
}

void BMX_sensortime(uint32_t* time){
	uint8_t data[3];
	BMX_read(SENSOERTIME_0, data, 3);
	*time = (uint32_t)((data[2] << 16) | (data[1] << 8) | data[0]);
	//*time = t;

}


/*
void BMX_write(uint8_t reg_addr, uint8_t *data, uint16_t len){
	BMX160_enable();
	HAL_SPI_Transmit(BMX_SPI, &reg_addr, 1, 10);
	for (int i = 0; i < len; i++){
		HAL_SPI_Transmit(BMX_SPI, data, 1, 10);
		data++;
	}
	BMX160_disable();
}
*/

