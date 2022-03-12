/*
 * W25Q32.h
 *
 *  Created on: Jul 30, 2021
 *      Author: David
 */

#ifndef INC_W25Q32_H_
#define INC_W25Q32_H_

void W25Q_enable();
void W25Q_disable();
uint8_t W25Q32_init();
uint32_t W25Q_readID();
void W25Q_write_CMD(uint8_t reg_addr);
void W25Q_read(uint32_t reg_addr, uint8_t *data, uint16_t len);
void W25Q_write(uint32_t reg_addr, uint8_t *data, uint16_t len);
void W25Q_write_byte(uint32_t reg_addr, uint8_t data);
void W25Q_read_byte(uint32_t reg_addr, uint8_t *data);

void W25Q_write_enable();
void W25Q_write_disable();
void W25Q_chip_erase();
void W25Q_reset();

#endif /* INC_W25Q32_H_ */
