/*
 * i2c_master.h
 *
 *  Created on: Jan 17, 2024
 *      Author: gvigelet
 */

#ifndef INC_I2C_MASTER_H_
#define INC_I2C_MASTER_H_

#include "main.h"
#include <stdio.h>
#include <stdbool.h>

void I2C_scan(void);
uint8_t send_buffer_to_slave(uint8_t slave_addr, uint8_t* pBuffer, uint16_t buf_len);
uint8_t read_status_register_of_slave(uint8_t slave_addr, uint8_t* pBuffer, uint16_t max_len);
uint8_t read_data_register_of_slave(uint8_t slave_addr, uint8_t* pBuffer, size_t rx_len);

uint16_t I2C_read_CDCE6214_reg(uint8_t i2c_addr, uint16_t reg_addr);
bool I2C_write_CDCE6214_reg(uint8_t i2c_addr, uint16_t reg_addr, uint16_t reg_val);

float MAX31875_ReadTemperature(void);

#endif /* INC_I2C_MASTER_H_ */
