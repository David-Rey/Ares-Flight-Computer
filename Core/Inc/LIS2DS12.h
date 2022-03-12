/*
 * LIS2DS12.h
 *
 *  Created on: Jul 20, 2021
 *      Author: David
 */

#ifndef SRC_LIS2DS12_H_
#define SRC_LIS2DS12_H_

typedef struct{
	int16_t x;
	int16_t y;
	int16_t z;
}LISrawData;

typedef enum{
	WHO_AM_I		= 0x0F,
	CTRL1			= 0x20,
	OUT_T			= 0x26,
	STATUS			= 0x27,
	OUT_X_L			= 0x28,
	OUT_X_H			= 0x29,
	OUT_Y_L			= 0x2A,
	OUT_Y_H			= 0x2B,
	OUT_Z_L			= 0x2C,
	OUT_Z_H			= 0x2D
} lisRegisters;

void LIS_write_byte(uint8_t reg_addr, uint8_t data);
void LIS_read(uint8_t reg_addr, uint8_t *data, uint16_t length);
void LIS_init();
void LIS_acc(LISrawData* raw_data);

#endif /* SRC_LIS2DS12_H_ */
