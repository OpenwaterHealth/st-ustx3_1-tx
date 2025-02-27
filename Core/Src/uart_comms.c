/*
 * uart_comms.c
 *
 *  Created on: Mar 11, 2024
 *      Author: gvigelet
 */

#include "if_commands.h"
#include "main.h"
#include "uart_comms.h"
#include "module_manager.h"
#include "utils.h"
#include <string.h>
#include "usbd_cdc_if.h"

#define ONEWIRE_TIMEOUT 25
#define TX_TIMEOUT 25

// Private variables
uint8_t rxBuffer[COMMAND_MAX_SIZE];
uint8_t txBuffer[COMMAND_MAX_SIZE];
uint8_t owBuffer[COMMAND_MAX_SIZE];
uint8_t owTxBuffer[COMMAND_MAX_SIZE];

volatile uint32_t ptrReceive;
volatile uint8_t rx_flag = 0;
volatile uint8_t tx_flag = 0;
volatile uint8_t rx_ow_slave_flag = 0;
volatile uint8_t tx_ow_slave_flag = 0;
volatile uint8_t rx_ow_master_flag = 0;
volatile uint8_t tx_ow_master_flag = 0;
volatile uint16_t ow_packetid = 0;

static uint16_t ow_packet_count;
static UartPacket ow_send_packet;
static UartPacket ow_receive_packet;

static uint8_t module_ID = 0;

static DEVICE_ROLE my_device_role = ROLE_UNDEFINED;

static uint16_t get_ow_next_packetID(){
	ow_packet_count++;
	if(ow_packet_count == 0) ow_packet_count = 1;
	return ow_packet_count;
}

static void buffer_to_packet(uint8_t* pBuffer, UartPacket* pPacket) {

    int bufferIndex = 0;
    uint16_t calculated_crc;

    if(pBuffer[bufferIndex++] != OW_START_BYTE) {
        // Send NACK doesn't have the correct start byte
    	pPacket->id = 0xFFFF;
    	pPacket->data_len = 0;
    	pPacket->packet_type = OW_NAK;
        return;
    }

    pPacket->id = (pBuffer[bufferIndex] << 8 | (pBuffer[bufferIndex+1] & 0xFF ));
    bufferIndex+=2;
    pPacket->packet_type = pBuffer[bufferIndex++];
    pPacket->command = pBuffer[bufferIndex++];
    pPacket->addr = pBuffer[bufferIndex++];
    pPacket->reserved = pBuffer[bufferIndex++];

    // Extract payload length
    pPacket->data_len = (pBuffer[bufferIndex] << 8 | (pBuffer[bufferIndex+1] & 0xFF ));
    bufferIndex+=2;

    // Check if data length is valid
    if (pPacket->data_len > COMMAND_MAX_SIZE - bufferIndex && pBuffer[COMMAND_MAX_SIZE-1] != OW_END_BYTE) {
        // Send NACK response due to no end byte
    	pPacket->data_len = 0;
        pPacket->packet_type = OW_ERROR;
        return;
    }

    // Extract data pointer
    pPacket->data = &pBuffer[bufferIndex];
    if (pPacket->data_len > COMMAND_MAX_SIZE)
    {
    	bufferIndex=COMMAND_MAX_SIZE-3; // [3 bytes from the end should be the crc for a continuation packet]
    }else{
    	bufferIndex += pPacket->data_len; // move pointer to end of data
    }

    // Extract received CRC
    pPacket->crc = (pBuffer[bufferIndex] << 8 | (pBuffer[bufferIndex+1] & 0xFF ));
    bufferIndex+=2;

    // Calculate CRC for received data

    if (pPacket->data_len > COMMAND_MAX_SIZE)
    {
    	calculated_crc = util_crc16(&pBuffer[1], COMMAND_MAX_SIZE-3);
    }
    else
    {
    	calculated_crc = util_crc16(&pBuffer[1], pPacket->data_len + 8);
    }

    // Check CRC
    if (pPacket->crc != calculated_crc) {
        // Send NACK response due to bad CRC
    	pPacket->reserved = OW_BAD_CRC;
    	pPacket->data_len = 0;
        pPacket->packet_type = OW_ERROR;
        return;
    }

    // Check end byte
    if (pBuffer[bufferIndex++] != OW_END_BYTE) {
    	pPacket->data_len = 0;
    	pPacket->reserved = 0;
    	pPacket->packet_type = OW_ERROR;
        return;
    }
}

static void comms_interface_send(UartPacket* pResp)
{
    tx_flag = 0;  // Clear the flag before starting transmission

    memset(txBuffer, 0, sizeof(txBuffer));
    int bufferIndex = 0;

    // Build the packet header
    txBuffer[bufferIndex++] = OW_START_BYTE;
    txBuffer[bufferIndex++] = pResp->id >> 8;
    txBuffer[bufferIndex++] = pResp->id & 0xFF;
    txBuffer[bufferIndex++] = OW_RESP;
    txBuffer[bufferIndex++] = pResp->command;
    txBuffer[bufferIndex++] = pResp->addr;
    txBuffer[bufferIndex++] = pResp->reserved;
    txBuffer[bufferIndex++] = (pResp->data_len) >> 8;
    txBuffer[bufferIndex++] = (pResp->data_len) & 0xFF;

    // Check for possible buffer overflow (optional)
    if ( (bufferIndex + pResp->data_len + 4) > sizeof(txBuffer) ) {
        // Handle error: packet too large for txBuffer
        return;
    }

    // Add data payload if any
    if(pResp->data_len > 0)
    {
        memcpy(&txBuffer[bufferIndex], pResp->data, pResp->data_len);
        bufferIndex += pResp->data_len;
    }

    // Compute CRC over the packet from index 1 for (pResp->data_len + 8) bytes
    uint16_t crc = util_crc16(&txBuffer[1], pResp->data_len + 8);
    txBuffer[bufferIndex++] = crc >> 8;
    txBuffer[bufferIndex++] = crc & 0xFF;

    // Add the end byte
    txBuffer[bufferIndex++] = OW_END_BYTE;

    // Initiate transmission via USB CDC
    CDC_Transmit_FS(txBuffer, bufferIndex);

    // Wait for the transmit complete flag with a timeout to avoid infinite loop.
    uint32_t start_time = HAL_GetTick();

    while(!tx_flag)
    {
        if ((HAL_GetTick() - start_time) >= TX_TIMEOUT)
        {
            // Timeout handling: Log error and break out or reset the flag.
            // printf("TX Timeout\r\n");
            break;
        }
    }
}

static bool comms_onewire_send(UartPacket* pResp)
{
    uint32_t start_time = 0;
    int bufferIndex = 0;
    uint8_t device_role = get_device_role();

    /* Clear the transmission buffer */
    memset(owTxBuffer, 0, sizeof(owTxBuffer));

    /* Ensure packet ID is set */
    if(pResp->id == 0)
        pResp->id = get_ow_next_packetID();

    /* Build the packet */
    owTxBuffer[bufferIndex++] = OW_START_BYTE;
    owTxBuffer[bufferIndex++] = pResp->id >> 8;
    owTxBuffer[bufferIndex++] = pResp->id & 0xFF;
    owTxBuffer[bufferIndex++] = pResp->packet_type;
    owTxBuffer[bufferIndex++] = pResp->command;
    owTxBuffer[bufferIndex++] = pResp->addr;
    owTxBuffer[bufferIndex++] = pResp->reserved;
    owTxBuffer[bufferIndex++] = (pResp->data_len) >> 8;
    owTxBuffer[bufferIndex++] = (pResp->data_len) & 0xFF;

    /* Check for data payload overflow */
    if(pResp->data_len > 0)
    {
        /* (Optionally, check that bufferIndex + pResp->data_len is within bounds) */
        memcpy(&owTxBuffer[bufferIndex], pResp->data, pResp->data_len);
        bufferIndex += pResp->data_len;
    }

    /* Calculate CRC over the packet (from index 1, covering 8 header bytes + payload) */
    uint16_t crc = util_crc16(&owTxBuffer[1], pResp->data_len + 8);
    owTxBuffer[bufferIndex++] = crc >> 8;
    owTxBuffer[bufferIndex++] = crc & 0xFF;

    /* Append the end byte */
    owTxBuffer[bufferIndex++] = OW_END_BYTE;

    /* Reset the appropriate transmission flag */
    if(device_role == ROLE_MASTER)
    {
        tx_ow_master_flag = 0;
    }
    else
    {
        tx_ow_slave_flag = 0;
    }

    /* Optional: Print the buffer for debugging */
    // printBuffer(owTxBuffer, pResp->data_len + 12);

    /* Enable the transmitter in half-duplex mode */
    if(HAL_HalfDuplex_EnableTransmitter(&huart3) != HAL_OK)
    {
        /* Setup Error */
        Error_Handler();
    }

    /* Start transmission in interrupt mode */
    if(HAL_UART_Transmit_IT(&huart3, (uint8_t *)owTxBuffer, bufferIndex) != HAL_OK)
    {
        /* Transmission Error */
        Error_Handler();
    }

    start_time = HAL_GetTick();  // Record the start time

    /* Wait for transmission to complete (flag set in the callback) */
    if(device_role == ROLE_MASTER)
    {
        while(!tx_ow_master_flag)
        {
            if ((HAL_GetTick() - start_time) >= ONEWIRE_TIMEOUT)
            {
                // printf("Timed Out\r\n");
                return false;  // Timeout occurred
            }
            // Optionally add a small delay here to yield CPU time.
        }
    }
    else
    {
        while(!tx_ow_slave_flag)
        {
            if ((HAL_GetTick() - start_time) >= ONEWIRE_TIMEOUT)
            {
                // printf("Timed Out\r\n");
                return false;  // Timeout occurred
            }
            // Optionally add a small delay here.
        }
    }

    return true;
}

static void comms_onewire_receive(UartPacket* pRetPacket)
{
    uint32_t start_time = 0;

    // Clear the buffer
    memset(owBuffer, 0, sizeof(owBuffer));

    // Clear the flag before starting the reception
    rx_ow_master_flag = 0;  // Ensure rx_ow_master_flag is declared as volatile

    // Enable receiver in half-duplex mode
    if(HAL_HalfDuplex_EnableReceiver(&huart2) != HAL_OK) {
        // Receive Error
        Error_Handler();
    }

    // Start reception until idle (non-blocking, interrupt-driven)
    if (HAL_UARTEx_ReceiveToIdle_IT(&huart2, owBuffer, COMMAND_MAX_SIZE) != HAL_OK) {
        // Receive Error
        Error_Handler();
    }

    // Start the timeout counter
    start_time = HAL_GetTick();
    while(!rx_ow_master_flag) {
        if ((HAL_GetTick() - start_time) >= ONEWIRE_TIMEOUT) {
            // printf("Timed Out\r\n");
            pRetPacket->packet_type = OW_ERROR;
            pRetPacket->data_len = 0;
            return;  // Timeout occurred
        }
    }

    // Process the received data
    buffer_to_packet(owBuffer, pRetPacket);
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
    	resp.id = 0xFFFF;
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
        resp.packet_type = OW_ERROR;
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
    	resp.reserved = OW_BAD_CRC;
        resp.data_len = 0;
        resp.packet_type = OW_ERROR;
        goto NextDataPacket;
    }

    // Check end byte
    if (rxBuffer[bufferIndex++] != OW_END_BYTE) {
    	resp.id = cmd.id;
        resp.data_len = 0;
    	resp.addr = 0;
    	resp.reserved = 0;
        resp.packet_type = OW_ERROR;
        goto NextDataPacket;
    }

	process_if_command(&cmd, &resp);
	// printf("CMD Packet:\r\n");
	// print_uart_packet(&cmd);

NextDataPacket:
	comms_interface_send(&resp);
	memset(rxBuffer, 0, sizeof(rxBuffer));
	ptrReceive=0;
	rx_flag = 0;
	CDC_ReceiveToIdle(rxBuffer, COMMAND_MAX_SIZE);
}

void comms_onewire_check_received()
{
    uint16_t calculated_crc;

	if(!rx_ow_slave_flag) return;

    memset((void*)&ow_send_packet, 0, sizeof(ow_send_packet));
    memset((void*)&ow_receive_packet, 0, sizeof(ow_receive_packet));

    int bufferIndex = 0;

    if(owBuffer[bufferIndex++] != OW_START_BYTE) {
        // Send NACK doesn't have the correct start byte
    	ow_send_packet.id = OW_CMD_NOP;
    	ow_send_packet.data_len = 0;
    	ow_send_packet.reserved = OW_INVALID_PACKET;
    	ow_send_packet.packet_type = OW_ERROR;
        goto NextOneWirePacket;
    }


    ow_receive_packet.id = (owBuffer[bufferIndex] << 8 | (owBuffer[bufferIndex+1] & 0xFF ));
    bufferIndex+=2;
    ow_receive_packet.packet_type = owBuffer[bufferIndex++];
    ow_receive_packet.command = owBuffer[bufferIndex++];
    ow_receive_packet.addr = owBuffer[bufferIndex++];
    ow_receive_packet.reserved = owBuffer[bufferIndex++];

    // Extract payload length
    ow_receive_packet.data_len = (owBuffer[bufferIndex] << 8 | (owBuffer[bufferIndex+1] & 0xFF ));
    bufferIndex+=2;

    // Check if data length is valid
    if (ow_receive_packet.data_len > COMMAND_MAX_SIZE - bufferIndex && owBuffer[COMMAND_MAX_SIZE-1] != OW_END_BYTE) {
        // Send NACK response due to no end byte
    	// data can exceed buffersize but every buffer must have a start and end packet
    	// command that will send more data than one buffer will follow with data packets to complete the request
    	ow_send_packet.id = ow_receive_packet.id;
    	ow_send_packet.addr = 0;
    	ow_send_packet.reserved = OW_INVALID_PACKET;
    	ow_send_packet.data_len = 0;
    	ow_send_packet.packet_type = OW_ERROR;
        goto NextOneWirePacket;
    }

    // Extract data pointer
    ow_receive_packet.data = &owBuffer[bufferIndex];
    if (ow_receive_packet.data_len > COMMAND_MAX_SIZE)
    {
    	bufferIndex=COMMAND_MAX_SIZE-3; // [3 bytes from the end should be the crc for a continuation packet]
    }else{
    	bufferIndex += ow_receive_packet.data_len; // move pointer to end of data
    }

    // Extract received CRC
    ow_receive_packet.crc = (owBuffer[bufferIndex] << 8 | (owBuffer[bufferIndex+1] & 0xFF ));
    bufferIndex+=2;

    // Calculate CRC for received data

    if (ow_receive_packet.data_len > COMMAND_MAX_SIZE)
    {
    	calculated_crc = util_crc16(&owBuffer[1], COMMAND_MAX_SIZE-3);
    }
    else
    {
    	calculated_crc = util_crc16(&owBuffer[1], ow_receive_packet.data_len + 8);
    }

    // Check CRC
    if (ow_receive_packet.crc != calculated_crc) {
        // Send NACK response due to bad CRC
    	ow_send_packet.id = ow_send_packet.id;
    	ow_send_packet.addr = 0;
    	ow_send_packet.reserved = OW_BAD_CRC;
    	ow_send_packet.data_len = 0;
    	ow_send_packet.packet_type = OW_ERROR;
        goto NextOneWirePacket;
    }

    //print_uart_packet(&ow_receive_packet);
	process_if_command(&ow_receive_packet, &ow_send_packet);

NextOneWirePacket:
	HAL_Delay(1);
	if(!comms_onewire_send(&ow_send_packet))
	{
		printf("failed to send onewire response\r\n");
	}
	memset(owBuffer, 0, sizeof(owBuffer));
    rx_ow_slave_flag = 0;
    if(HAL_HalfDuplex_EnableReceiver(&huart2) != HAL_OK) {
    	// Receive Error
		Error_Handler();
    }

	if (HAL_UARTEx_ReceiveToIdle_IT(&huart2, owBuffer, COMMAND_MAX_SIZE) != HAL_OK) {
		// Receive Error
		Error_Handler();
	}
}

DEVICE_ROLE get_device_role()
{
	return my_device_role;
}

void set_device_role(DEVICE_ROLE role)
{
	my_device_role = role;
}
bool comms_onewire_slave_start()
{
	ow_packet_count = 0;
	memset(owBuffer, 0, sizeof(owBuffer));
    rx_ow_slave_flag = 0;
    tx_ow_slave_flag = 0;
    HAL_UART_Abort(&huart2);
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

bool configure_master()
{
	ow_packet_count = 0;
	bool bRet = false;
	set_module_ID(0);
    HAL_UART_Abort(&huart3);
    rx_ow_slave_flag = 0;
    tx_ow_slave_flag = 0;

    comms_host_start();

	// configure master
	printf("Configured as MASTER\r\n");

    ModuleManager_Init();
    // Register the master module
    ModuleManager_RegisterMaster(0x00);
	printf("Enumerating any attached slave transmitters\r\n");
	// register found slaves

    memset((void*)&ow_send_packet, 0, sizeof(ow_send_packet));

	ow_send_packet.id = 0; // so it pulls the next id
	ow_send_packet.packet_type = OW_ONE_WIRE;
	ow_send_packet.command = OW_CMD_PING;

	if(comms_onewire_send(&ow_send_packet)){
		comms_onewire_receive(&ow_receive_packet);
		if(ow_receive_packet.packet_type != OW_ERROR){
			printf("Onewire txrx success\r\n");
		}else{
			printf("Onewire txrx failure\r\n");
		}
	}else{
		printf("Onewire tx failed\r\n");
	}

	//print_uart_packet(&ow_receive_packet);
	return bRet;
}

bool configure_slave()
{
	bool bRet = false;
	CDC_Stop_ReceiveToIdle();
	ModuleManager_DeInit();

	// configure slave
	comms_onewire_slave_start();
	printf("Configured as Slave\r\n");
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
