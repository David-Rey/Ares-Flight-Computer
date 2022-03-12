/*
 * W25Q32.c
 *
 *  Created on: Jul 30, 2021
 *      Author: David
 */

#include "stdint.h"
#include "W25Q32.h"
#include "main.h"

#define DUMMY_BYTE 0xA5

SPI_HandleTypeDef hspi1;
#define FLASH_SPI &hspi1


void W25Q_enable(){
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
}

void W25Q_disable(){
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
}

uint8_t W25Q32_init(){
	HAL_Delay(10);
	W25Q_enable();
	//W25Q_reset();
	W25Q_enable();
	uint32_t id = W25Q_readID();
	W25Q_disable();
	if ((id & 0x000000FF) == 0x16){
		return 1;
	}
	return 0;
}

uint8_t W25Q_SPI(uint8_t data){
	uint8_t ret;
	HAL_SPI_TransmitReceive(FLASH_SPI, &data, &ret, 1, 10);
	return ret;
}

uint32_t W25Q_readID(){
	uint32_t Temp0, Temp1, Temp2;
	W25Q_SPI(0x9F);
	Temp0 = W25Q_SPI(DUMMY_BYTE);
	Temp1 = W25Q_SPI(DUMMY_BYTE);
	Temp2 = W25Q_SPI(DUMMY_BYTE);
	uint32_t ID = (Temp0 << 16) | (Temp1 << 8) | Temp2;
	return ID;
}

void W25Q_read(uint32_t reg_addr, uint8_t *data, uint16_t len){
	uint8_t byte1 = 0x03;
	uint8_t byte2 = (reg_addr & 0xFF0000) >> 16;
	uint8_t byte3 = (reg_addr & 0xFF00) >> 8;
	uint8_t byte4 = (reg_addr & 0xFF);
	//uint8_t byte5 = 0;

	W25Q_enable();
	HAL_SPI_Transmit(FLASH_SPI, &byte1, 1, 10);
	HAL_SPI_Transmit(FLASH_SPI, &byte2, 1, 10);
	HAL_SPI_Transmit(FLASH_SPI, &byte3, 1, 10);
	HAL_SPI_Transmit(FLASH_SPI, &byte4, 1, 10);
	//HAL_SPI_Transmit(FLASH_SPI, &byte5, 1, 10);
	for (int i = 0; i < len; i++){
		HAL_SPI_Receive(FLASH_SPI, data, 1, 10);
		data++;
	}
	W25Q_disable();
}

void W25Q_write(uint32_t reg_addr, uint8_t *data, uint16_t len){
	uint8_t byte1 = 0x02;
	uint8_t byte2 = (reg_addr & 0xFF0000) >> 16;
	uint8_t byte3 = (reg_addr & 0xFF00) >> 8;
	uint8_t byte4 = (reg_addr & 0xFF);

	W25Q_write_enable();
	W25Q_enable();
	HAL_SPI_Transmit(FLASH_SPI, &byte1, 1, 10);
	HAL_SPI_Transmit(FLASH_SPI, &byte2, 1, 10);
	HAL_SPI_Transmit(FLASH_SPI, &byte3, 1, 10);
	HAL_SPI_Transmit(FLASH_SPI, &byte4, 1, 10);
	HAL_SPI_Transmit(FLASH_SPI, data, len, 100);
	W25Q_disable();
}

/*
void W25Q_write_byte(uint32_t reg_addr, uint8_t data){
	//W25Q_write_enable();
	W25Q_enable();
	W25Q_SPI(0x02);
	W25Q_SPI((reg_addr & 0xFF0000) >> 16);
	W25Q_SPI((reg_addr & 0xFF00) >> 8);
	W25Q_SPI(reg_addr & 0xFF);
	W25Q_SPI(data);
	W25Q_disable();
	HAL_Delay(1);
}
*/
void W25Q_read_byte(uint32_t reg_addr, uint8_t *data){
	W25Q_enable();
	W25Q_SPI(0x0B);
	W25Q_SPI((reg_addr & 0xFF0000) >> 16);
	W25Q_SPI((reg_addr& 0xFF00) >> 8);
	W25Q_SPI(reg_addr & 0xFF);
	W25Q_SPI(0);
	*data = W25Q_SPI(DUMMY_BYTE);
	W25Q_disable();
}
void W25Q_write_enable(){
	W25Q_enable();
	W25Q_SPI(0x06);
	W25Q_disable();
	HAL_Delay(1);
}

void W25Q_write_disable(){
	W25Q_enable();
	W25Q_SPI(0x04);
	W25Q_disable();
	HAL_Delay(1);
}

void W25Q_chip_erase(){
	W25Q_enable();
	W25Q_SPI(0xC7);
	W25Q_disable();
	HAL_Delay(1);
}

void W25Q_reset(){
	W25Q_enable();
	W25Q_SPI(0x66);
	W25Q_disable();
	HAL_Delay(1);
	W25Q_enable();
	W25Q_SPI(0x99);
	W25Q_disable();
	HAL_Delay(1);
}


