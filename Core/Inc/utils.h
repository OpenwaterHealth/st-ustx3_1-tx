/*
 * utils.h
 *
 *  Created on: Jan 3, 2024
 *      Author: gvigelet
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_


#include "main.h"
#include <stdint.h>


extern TIM_HandleTypeDef htim3;

uint16_t util_crc16(uint8_t* buf, uint32_t size);
uint16_t util_hw_crc16(uint8_t* buf, uint32_t size);
uint8_t crc_test(void);
void get_unique_identifier(uint32_t* uid);
uint32_t fnv1a_32(const uint8_t *data, size_t len);

#endif /* INC_UTILS_H_ */
