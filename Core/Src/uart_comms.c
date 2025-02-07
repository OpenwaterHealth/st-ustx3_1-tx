/*
 * uart_comms.c
 *
 *  Created on: Mar 11, 2024
 *      Author: gvigelet
 */

#include "if_commands.h"
#include "main.h"
#include "uart_comms.h"
#include "utils.h"
#include <string.h>
#include "usbd_cdc_if.h"

#define ONEWIRE_TIMEOUT 250

// Private variables
uint8_t rxBuffer[COMMAND_MAX_SIZE];
uint8_t txBuffer[COMMAND_MAX_SIZE];
uint8_t owBuffer[COMMAND_MAX_SIZE];

volatile uint32_t ptrReceive;
volatile uint8_t rx_flag = 0;
volatile uint8_t tx_flag = 0;
volatile uint8_t rx_ow_slave_flag = 0;
volatile uint8_t tx_ow_slave_flag = 0;
volatile uint8_t rx_ow_master_flag = 0;
volatile uint8_t tx_ow_master_flag = 0;
volatile uint16_t ow_packetid = 0;

static uint8_t module_ID = 0;

static DEVICE_ROLE my_device_role = ROLE_UNDEFINED;

static void comms_interface_send(UartPacket* pResp)
{
	// while (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
	memset(owBuffer, 0, sizeof(owBuffer));
	int bufferIndex = 0;

	txBuffer[bufferIndex++] = OW_START_BYTE;
	txBuffer[bufferIndex++] = pResp->id >> 8;
	txBuffer[bufferIndex++] = pResp->id & 0xFF;
	txBuffer[bufferIndex++] = OW_RESP;
	txBuffer[bufferIndex++] = pResp->command;
	txBuffer[bufferIndex++] = pResp->addr;
	txBuffer[bufferIndex++] = pResp->reserved;
	txBuffer[bufferIndex++] = (pResp->data_len) >> 8;
	txBuffer[bufferIndex++] = (pResp->data_len) & 0xFF;
	if(pResp->data_len > 0)
	{
		memcpy(&txBuffer[bufferIndex], pResp->data, pResp->data_len);
		bufferIndex += pResp->data_len;
	}
	uint16_t crc = util_crc16(&txBuffer[1], pResp->data_len + 8);
	txBuffer[bufferIndex++] = crc >> 8;
	txBuffer[bufferIndex++] = crc & 0xFF;

	txBuffer[bufferIndex++] = OW_END_BYTE;

	CDC_Transmit_FS(txBuffer, bufferIndex);

	while(!tx_flag);
}

static bool comms_onewire_master_send(uint8_t tx_id, uint8_t cmd, uint8_t *pData, uint16_t data_len)
{
    uint32_t start_time = HAL_GetTick();  // Get the current time
	memset(txBuffer, 0, sizeof(txBuffer));
	ow_packetid++;
	if(ow_packetid==0){
		ow_packetid = 1;
	}

	int bufferIndex = 0;

	owBuffer[bufferIndex++] = OW_START_BYTE;
	owBuffer[bufferIndex++] = ow_packetid >> 8;
	owBuffer[bufferIndex++] = ow_packetid & 0xFF;
	owBuffer[bufferIndex++] = OW_ONE_WIRE;
	owBuffer[bufferIndex++] = cmd;
	owBuffer[bufferIndex++] = tx_id;  // transmiter module ID
	owBuffer[bufferIndex++] = 0;
	owBuffer[bufferIndex++] = data_len >> 8;
	owBuffer[bufferIndex++] = data_len & 0xFF;
	if(data_len > 0)
	{
		memcpy(&owBuffer[bufferIndex], pData, data_len);
		bufferIndex += data_len;
	}
	uint16_t crc = util_crc16(&owBuffer[1], data_len + 8);
	owBuffer[bufferIndex++] = crc >> 8;
	owBuffer[bufferIndex++] = crc & 0xFF;

	tx_ow_master_flag = 0;

	if (HAL_HalfDuplex_EnableTransmitter(&huart3) != HAL_OK) {
        // Setup Error
        Error_Handler();
	}
    if (HAL_UART_Transmit_IT(&huart3, (uint8_t *)owBuffer, bufferIndex) != HAL_OK) {
        // Transmission Error
        Error_Handler();
    }

    while(!tx_ow_master_flag){
        if ((HAL_GetTick() - start_time) >= ONEWIRE_TIMEOUT) {  // Check if 100ms have passed
            return false;  // Timeout occurred
        }
    }

    return true;
}

void set_module_ID(uint8_t id) {
    module_ID = id;
}

uint8_t get_module_ID() {
    return module_ID;
}

void comms_host_start(void)
{

	memset(rxBuffer, 0, sizeof(rxBuffer));
	ptrReceive = 0;

	CDC_FlushRxBuffer_FS();

    rx_flag = 0;
    tx_flag = 0;

	CDC_ReceiveToIdle(rxBuffer, COMMAND_MAX_SIZE);

}

void comms_host_check_received(void)
{
	UartPacket cmd;
	UartPacket resp;
    uint16_t calculated_crc;

	if(!rx_flag) return;

    int bufferIndex = 0;

    if(rxBuffer[bufferIndex++] != OW_START_BYTE) {
        // Send NACK doesn't have the correct start byte
    	resp.id = OW_CMD_NOP;
        resp.data_len = 0;
        resp.packet_type = OW_NAK;
        goto NextDataPacket;
    }

    cmd.id = (rxBuffer[bufferIndex] << 8 | (rxBuffer[bufferIndex+1] & 0xFF ));
    bufferIndex+=2;
    cmd.packet_type = rxBuffer[bufferIndex++];
    cmd.command = rxBuffer[bufferIndex++];
    cmd.addr = rxBuffer[bufferIndex++];
    cmd.reserved = rxBuffer[bufferIndex++];

    // Extract payload length
    cmd.data_len = (rxBuffer[bufferIndex] << 8 | (rxBuffer[bufferIndex+1] & 0xFF ));
    bufferIndex+=2;

    // Check if data length is valid
    if (cmd.data_len > COMMAND_MAX_SIZE - bufferIndex && rxBuffer[COMMAND_MAX_SIZE-1] != OW_END_BYTE) {
        // Send NACK response due to no end byte
    	// data can exceed buffersize but every buffer must have a start and end packet
    	// command that will send more data than one buffer will follow with data packets to complete the request
    	resp.id = cmd.id;
    	resp.addr = 0;
    	resp.reserved = 0;
        resp.data_len = 0;
        resp.packet_type = OW_NAK;
        goto NextDataPacket;
    }

    // Extract data pointer
    cmd.data = &rxBuffer[bufferIndex];
    if (cmd.data_len > COMMAND_MAX_SIZE)
    {
    	bufferIndex=COMMAND_MAX_SIZE-3; // [3 bytes from the end should be the crc for a continuation packet]
    }else{
    	bufferIndex += cmd.data_len; // move pointer to end of data
    }

    // Extract received CRC
    cmd.crc = (rxBuffer[bufferIndex] << 8 | (rxBuffer[bufferIndex+1] & 0xFF ));
    bufferIndex+=2;

    // Calculate CRC for received data

    if (cmd.data_len > COMMAND_MAX_SIZE)
    {
    	calculated_crc = util_crc16(&rxBuffer[1], COMMAND_MAX_SIZE-3);
    }
    else
    {
    	calculated_crc = util_crc16(&rxBuffer[1], cmd.data_len + 8);
    }

    // Check CRC
    if (cmd.crc != calculated_crc) {
        // Send NACK response due to bad CRC
    	resp.id = cmd.id;
    	resp.addr = 0;
    	resp.reserved = 0;
        resp.data_len = 0;
        resp.packet_type = OW_BAD_CRC;
        goto NextDataPacket;
    }

    // Check end byte
    if (rxBuffer[bufferIndex++] != OW_END_BYTE) {
    	resp.id = cmd.id;
        resp.data_len = 0;
    	resp.addr = 0;
    	resp.reserved = 0;
        resp.packet_type = OW_NAK;
        goto NextDataPacket;
    }

	resp = process_if_command(cmd);

NextDataPacket:
	comms_interface_send(&resp);
	memset(rxBuffer, 0, sizeof(rxBuffer));
	ptrReceive=0;
	rx_flag = 0;
	CDC_ReceiveToIdle(rxBuffer, COMMAND_MAX_SIZE);
}

void comms_onewire_enum_slaves(void)
{

}

bool comms_onewire_slave_start()
{
	memset(owBuffer, 0, sizeof(owBuffer));
    rx_ow_slave_flag = 0;
    tx_ow_slave_flag = 0;

    if(HAL_HalfDuplex_EnableReceiver(&huart2) != HAL_OK) {
    	// Receive Error
		Error_Handler();
    }

	if (HAL_UARTEx_ReceiveToIdle_IT(&huart2, owBuffer, COMMAND_MAX_SIZE) != HAL_OK) {
		// Receive Error
		Error_Handler();
	}

	return true;
}

DEVICE_ROLE get_device_role()
{
	return my_device_role;
}

bool promote_to_master()
{
	bool bRet = false;
	if(my_device_role == ROLE_MASTER){
		// already configured
		return true;
	}
	// configure master
	my_device_role = ROLE_MASTER;
	HAL_GPIO_WritePin(SYSTEM_RDY_GPIO_Port, SYSTEM_RDY_Pin, GPIO_PIN_RESET);
	return bRet;
}

bool demote_to_slave()
{
	bool bRet = false;
	if(my_device_role == ROLE_SLAVE){
		// already configured
		return true;
	}else if(my_device_role == ROLE_MASTER){
		// deconfigure master first

	}
	// configure slave
	my_device_role = ROLE_SLAVE;
	HAL_GPIO_WritePin(SYSTEM_RDY_GPIO_Port, SYSTEM_RDY_Pin, GPIO_PIN_SET);
	return bRet;
}

// Callback functions
void comms_handle_ow_slave_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance == USART2) {
        // Notify the task
    	rx_ow_slave_flag = 1;
    }
}

void comms_handle_ow_slave_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        // Notify the task
    	tx_ow_slave_flag = 1;
    }
}


void comms_handle_ow_master_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance == USART3) {
        // Notify the task
    	rx_ow_master_flag = 1;
    }
}

void comms_handle_ow_master_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        // Notify the task
    	tx_ow_master_flag = 1;
    }
}

void CDC_handle_RxCpltCallback(uint16_t len) {
	rx_flag = 1;
}

void CDC_handle_TxCpltCallback() {
	tx_flag = 1;
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {

    if (huart->Instance == USART2) {

    }else if (huart->Instance == USART3) {

    }
}
