/*
 * BMP388.h
 *
 *  Created on: Jul 11, 2021
 *      Author: David
 */

#ifndef SRC_BMP388_H_
#define SRC_BMP388_H_

void BMP388_enable();
void BMP388_disable();
void BMP_read(uint8_t reg_addr, uint8_t *data, uint16_t len);
void BMP_write(uint8_t reg_addr, uint8_t *data, uint16_t len);
uint8_t BMP_init();
void BMP_raw_temp_press(uint32_t *temperature, uint32_t *pressure);
void BMP_temp_press(float *temperature, float *pressure);
void get_parms();
float compensate_temperature(float uncompTemp);
float compensate_pressure(float uncompPress, float temp);


enum {
	BMP388_CHIP_ID			= 0x00,			// Chip ID register sub-address
	BMP388_ERR_REG			= 0x02,			// Error register sub-address
	BMP388_STATUS			= 0x03,			// Status register sub-address
	BMP388_DATA_0			= 0x04,			// Pressure eXtended Least Significant Byte (XLSB) register sub-address
	BMP388_DATA_1			= 0x05,			// Pressure Least Significant Byte (LSB) register sub-address
	BMP388_DATA_2			= 0x06,			// Pressure Most Significant Byte (MSB) register sub-address
	BMP388_DATA_3			= 0x07,			// Temperature eXtended Least Significant Byte (XLSB) register sub-address
	BMP388_DATA_4			= 0x08,			// Temperature Least Significant Byte (LSB) register sub-address
	BMP388_DATA_5			= 0x09,			// Temperature Most Significant Byte (MSB) register sub-address
	BMP388_SENSORTIME_0		= 0x0C,			// Sensor time register 0 sub-address
	BMP388_SENSORTIME_1		= 0x0D,			// Sensor time register 1 sub-address
	BMP388_SENSORTIME_2		= 0x0E,			// Sensor time register 2 sub-address
	BMP388_EVENT			= 0x10,			// Event register sub-address
	BMP388_INT_STATUS		= 0x11,			// Interrupt Status register sub-address
	BMP388_FIFO_LENGTH_0	= 0x12,			// FIFO Length Least Significant Byte (LSB) register sub-address
	BMP388_FIFO_LENGTH_1	= 0x13,			// FIFO Length Most Significant Byte (MSB) register sub-address
	BMP388_FIFO_DATA		= 0x14,			// FIFO Data register sub-address
	BMP388_FIFO_WTM_0		= 0x15,			// FIFO Water Mark Least Significant Byte (LSB) register sub-address
	BMP388_FIFO_WTM_1		= 0x16,			// FIFO Water Mark Most Significant Byte (MSB) register sub-address
	BMP388_FIFO_CONFIG_1	= 0x17,			// FIFO Configuration 1 register sub-address
	BMP388_FIFO_CONFIG_2	= 0x18,			// FIFO Configuration 2 register sub-address
	BMP388_INT_CTRL			= 0x19,			// Interrupt Control register sub-address
	BMP388_IF_CONFIG		= 0x1A,			// Interface Configuration register sub-address
	BMP388_PWR_CTRL			= 0x1B,			// Power Control register sub-address
	BMP388_OSR				= 0x1C,			// Oversampling register sub-address
	BMP388_ODR				= 0x1D,			// Output Data Rate register sub-address
	BMP388_CONFIG			= 0x1F,			// Configuration register sub-address
	BMP388_TRIM_PARAMS		= 0x31,   		// Trim parameter registers' base sub-address
	BMP388_CMD				= 0x7E			// Command register sub-address
};

struct {
	uint16_t param_T1;
	uint16_t param_T2;
	int8_t   param_T3;
	int16_t  param_P1;
	int16_t  param_P2;
	int8_t   param_P3;
	int8_t   param_P4;
	uint16_t param_P5;
	uint16_t param_P6;
	int8_t   param_P7;
	int8_t   param_P8;
	int16_t  param_P9;
	int8_t 	 param_P10;
	int8_t 	 param_P11;
} __attribute__ ((packed)) params;

struct {
	float param_T1;
	float param_T2;
	float param_T3;
	float param_P1;
	float param_P2;
	float param_P3;
	float param_P4;
	float param_P5;
	float param_P6;
	float param_P7;
	float param_P8;
	float param_P9;
	float param_P10;
	float param_P11;
} floatParams;

#endif /* SRC_BMP388_H_ */
