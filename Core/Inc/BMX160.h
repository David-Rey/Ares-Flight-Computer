/*
 * BMX160.h
 *
 *  Created on: Jul 22, 2021
 *      Author: David
 */

#ifndef INC_BMX160_H_
#define INC_BMX160_H_

#define CHIP_ID_VAL 0xD8

typedef struct{
	int16_t x;
	int16_t y;
	int16_t z;
}BMXrawData;

typedef enum
{
	CHIP_ID  		= 0x00, // Should return 0xEA
	ERR_REG         = 0x02,  // Bit 7 enable DMP, bit 3 reset DMP
	PMU_STATUS      = 0x03,  // Bit 7 enable DMP, bit 3 reset DMP
	DATA_0   		= 0x04,
	DATA_1   		= 0x05,
	DATA_2   		= 0x06,
	DATA_3   		= 0x07,
	DATA_4   		= 0x08,
	DATA_5   		= 0x09,
	DATA_6   		= 0x0A,
	DATA_7   		= 0x0B,
	DATA_8   		= 0x0C,
	DATA_9   		= 0x0D,
	DATA_10   		= 0x0E,
	DATA_11   		= 0x0F,
	DATA_12   		= 0x10,
	DATA_13   		= 0x11,
	DATA_14   		= 0x12,
	DATA_15   		= 0x13,
	DATA_16  		= 0x14,
	DATA_17  		= 0x15,
	DATA_18   		= 0x16,
	DATA_19   		= 0x17,
	SENSOERTIME_0	= 0x18,
	SENSOERTIME_1	= 0x19,
	SENSOERTIME_2	= 0x1A,
	STATUS	   		= 0x1B,
	INT_STATUS_0	= 0x1C,
	INT_STATUS_1	= 0x1D,
	INT_STATUS_2	= 0x1E,
	INT_STATUS_3	= 0x1F,
	TEMPERATURE_0	= 0x20,
	TEMPERATURE_1	= 0x21,
	FIFO_LENGTH_0	= 0x22,
	FIFO_LENGTH_1	= 0x23,
	FIFO_DATA		= 0x24,
	ACC_CONF		= 0x40,
	ACC_RANGE		= 0x41,
	GYR_CONF		= 0x42,
	GYR_RANGE		= 0x43,
	MAG_CONF		= 0x44,
	FIFO_DOWNS		= 0x45,
	FIFO_CONFIG_0	= 0x46,
	FIFO_CONFIG_1	= 0x47,
	MAG_IF_0		= 0x4C,
	MAG_IF_1		= 0x4D,
	MAG_IF_2		= 0x4E,
	MAG_IF_3		= 0x4F,
	INT_EN_0		= 0x50,
	INT_EN_1		= 0x51,
	INT_EN_2		= 0x52,
	INT_OUT_CTRL	= 0x53,
	INT_LATCH		= 0x54,
	INT_MAP_0		= 0x55,
	INT_MAP_1		= 0x56,
	INT_MAP_2		= 0x57,
	INT_DATA_0		= 0x58,
	INT_DATA_1		= 0x59,
	INT_LOWHIGH_0	= 0x5A,
	INT_LOWHIGH_1	= 0x5B,
	INT_LOWHIGH_2	= 0x5C,
	INT_LOWHIGH_3	= 0x5D,
	INT_LOWHIGH_4	= 0x5E,
	INT_MOTION_0	= 0x5F,
	INT_MOTION_1	= 0x60,
	INT_MOTION_2	= 0x61,
	INT_MOTION_3	= 0x62,
	INT_TAP_0		= 0x63,
	INT_TAP_1		= 0x64,
	INT_ORIENT_0	= 0x65,
	INT_ORIENT_1	= 0x66,
	INT_FLAT_0		= 0x67,
	INT_FLAT_1		= 0x68,
	FOC_COMF		= 0x69,
	COMF			= 0x6A,
	IF_CONF			= 0x6B,
	PMU_TRIGGER		= 0x6C,
	SELF_TEST		= 0x6D,
	NV_CONF			= 0x70,
	OFFSET_0		= 0x71,
	OFFSET_1		= 0x72,
	OFFSET_2		= 0x73,
	OFFSET_3		= 0x74,
	OFFSET_4		= 0x75,
	OFFSET_5		= 0x76,
	OFFSET_6		= 0x77,
	STEP_CNT_0		= 0x78,
	STEP_CNT_1		= 0x79,
	STEP_CONF_0		= 0x7A,
	STEP_CONF_1		= 0x7B,
	CMD				= 0x7E
} bmx160Registers;


void BMX160_enable();
void BMX160_disable();
void BMX_read(uint8_t reg_addr, uint8_t *data, uint16_t len);
void BMX_write(uint8_t reg_addr, uint8_t *data, uint16_t len);
void BMX_write_byte(uint8_t reg_addr, uint8_t data);
uint8_t BMX_init();
void BMX_sensor_data(BMXrawData* raw_data, uint8_t DATA_ADDR);
void BMX_all_sensor_data(BMXrawData* acc, BMXrawData* gyr, BMXrawData* mag);

#endif /* INC_BMX160_H_ */
