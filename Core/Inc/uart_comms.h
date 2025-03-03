/*
 * uart_comms.h
 *
 *  Created on: May 13, 2024
 *      Author: gvigelet
 */

#ifndef INC_UART_COMMS_H_
#define INC_UART_COMMS_H_

#include "main.h"  // This should contain your HAL includes and other basic includes.
#include "common.h"
#include <stdio.h>
#include <stdbool.h>

void comms_host_start(void);
void comms_host_check_received(void);
bool comms_onewire_slave_start(void);
void comms_onewire_check_received(void);
bool comms_onewire_master_sendreceive(UartPacket* pSendPacket, UartPacket* pRetPacket);
void comms_handle_ow_master_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);
void comms_handle_ow_master_TxCpltCallback(UART_HandleTypeDef *huart);
void comms_handle_ow_slave_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size);
void comms_handle_ow_slave_TxCpltCallback(UART_HandleTypeDef *huart);
void CDC_handle_TxCpltCallback();
void set_module_ID(uint8_t id);
uint8_t get_module_ID();

DEVICE_ROLE get_device_role();
void set_device_role(DEVICE_ROLE role);
bool configure_master();
bool configure_slave();

#endif /* INC_UART_COMMS_H_ */
