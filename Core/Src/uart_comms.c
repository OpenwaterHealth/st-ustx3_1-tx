/*
 * uart_comms.c
 *
 *  Created on: Mar 11, 2024
 *      Author: gvigelet
 */

#include "if_commands.h"
#include "main.h"
#include "module_manager.h"
#include "uart_comms.h"
#include "utils.h"
#include <string.h>
#include "usbd_cdc_if.h"

#define ONEWIRE_TIMEOUT 1000
#define TX_TIMEOUT 1000

// Private variables
uint8_t rxBuffer[COMMAND_MAX_SIZE];
uint8_t txBuffer[COMMAND_MAX_SIZE];
uint8_t owRxBuffer[ONEWIRE_MAX_SIZE];
uint8_t owTxBuffer[ONEWIRE_MAX_SIZE];

volatile uint32_t ptrReceive;
volatile uint8_t rx_flag = 0;
volatile uint8_t tx_flag = 0;
volatile uint8_t rx_ow_callin_flag = 0;
volatile uint8_t tx_ow_callin_flag = 0;
volatile uint8_t rx_ow_callout_flag = 0;
volatile uint8_t tx_ow_callout_flag = 0;
volatile uint16_t ow_packetid = 0;

static uint16_t ow_packet_count;
static UartPacket ow_send_packet;
static UartPacket ow_receive_packet;

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
            break;
        }
    }
}

static bool comms_callout_onewire_send(UartPacket* pResp)
{
    uint32_t start_time = 0;
    int bufferIndex = 0;

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

    tx_ow_callout_flag = 0;

    /* Optional: Print the buffer for debugging */
    // printBuffer(owTxBuffer, pResp->data_len + 12);

    /* Enable the transmitter in half-duplex mode */
    if(HAL_HalfDuplex_EnableTransmitter(&CALL_OUT_UART) != HAL_OK)
    {
        /* Setup Error */
        Error_Handler();
    }

    /* Start transmission in interrupt mode */
    if(HAL_UART_Transmit_IT(&CALL_OUT_UART, (uint8_t *)owTxBuffer, bufferIndex) != HAL_OK)
    {
        /* Transmission Error */
        Error_Handler();
    }

    start_time = HAL_GetTick();  // Record the start time

    /* Wait for transmission to complete (flag set in the callback) */
    while(!tx_ow_callout_flag)
    {
        if ((HAL_GetTick() - start_time) >= ONEWIRE_TIMEOUT)
        {;
            return false;  // Timeout occurred
        }
        // Optionally add a small delay here to yield CPU time.
    }

    return true;
}


static bool comms_callin_onewire_send(UartPacket* pResp)
{
    uint32_t start_time = 0;
    int bufferIndex = 0;

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

    tx_ow_callin_flag = 0;

    /* Optional: Print the buffer for debugging */
    // printBuffer(owTxBuffer, pResp->data_len + 12);

    /* Enable the transmitter in half-duplex mode */
    if(HAL_HalfDuplex_EnableTransmitter(&CALL_IN_UART) != HAL_OK)
    {
        /* Setup Error */
        Error_Handler();
    }

    /* Start transmission in interrupt mode */
    if(HAL_UART_Transmit_IT(&CALL_IN_UART, (uint8_t *)owTxBuffer, bufferIndex) != HAL_OK)
    {
        /* Transmission Error */
        Error_Handler();
    }

    start_time = HAL_GetTick();  // Record the start time

    /* Wait for transmission to complete (flag set in the callback) */
    while(!tx_ow_callin_flag)
    {
        if ((HAL_GetTick() - start_time) >= ONEWIRE_TIMEOUT)
        {
            return false;  // Timeout occurred
        }
        // Optionally add a small delay here to yield CPU time.
    }

    return true;
}

static void comms_callout_onewire_receive(UartPacket* pRetPacket)
{
    uint32_t start_time = 0;

    // Clear the buffer
    memset(owRxBuffer, 0, sizeof(owRxBuffer));

    // Clear the flag before starting the reception
    rx_ow_callout_flag = 0;  // Ensure rx_ow_callout_flag is declared as volatile

    // Enable receiver in half-duplex mode
    if(HAL_HalfDuplex_EnableReceiver(&CALL_OUT_UART) != HAL_OK) {
        // Receive Error
        Error_Handler();
    }

    // Start reception until idle (non-blocking, interrupt-driven)
    if (HAL_UARTEx_ReceiveToIdle_IT(&CALL_OUT_UART, owRxBuffer, COMMAND_MAX_SIZE) != HAL_OK) {
        // Receive Error
        Error_Handler();
    }

    // Start the timeout counter
    start_time = HAL_GetTick();
    while(!rx_ow_callout_flag) {
        if ((HAL_GetTick() - start_time) >= ONEWIRE_TIMEOUT) {
            pRetPacket->packet_type = OW_TIMEOUT;
            pRetPacket->data_len = 0;
            return;  // Timeout occurred
        }
    }

    // Process the received data
    buffer_to_packet(owRxBuffer, pRetPacket);
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

	if(!rx_ow_callin_flag) return;

    memset((void*)&ow_send_packet, 0, sizeof(ow_send_packet));
    memset((void*)&ow_receive_packet, 0, sizeof(ow_receive_packet));

    int bufferIndex = 0;

    if(owRxBuffer[bufferIndex++] != OW_START_BYTE) {
        // Send NACK doesn't have the correct start byte
    	ow_send_packet.id = OW_CMD_NOP;
    	ow_send_packet.data_len = 0;
    	ow_send_packet.reserved = OW_INVALID_PACKET;
    	ow_send_packet.packet_type = OW_ERROR;
        goto NextOneWirePacket;
    }


    ow_receive_packet.id = (owRxBuffer[bufferIndex] << 8 | (owRxBuffer[bufferIndex+1] & 0xFF ));
    bufferIndex+=2;
    ow_receive_packet.packet_type = owRxBuffer[bufferIndex++];
    ow_receive_packet.command = owRxBuffer[bufferIndex++];
    ow_receive_packet.addr = owRxBuffer[bufferIndex++];
    ow_receive_packet.reserved = owRxBuffer[bufferIndex++];

    // Extract payload length
    ow_receive_packet.data_len = (owRxBuffer[bufferIndex] << 8 | (owRxBuffer[bufferIndex+1] & 0xFF ));
    bufferIndex+=2;

    // Check if data length is valid
    if (ow_receive_packet.data_len > COMMAND_MAX_SIZE - bufferIndex && owRxBuffer[COMMAND_MAX_SIZE-1] != OW_END_BYTE) {
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
    ow_receive_packet.data = &owRxBuffer[bufferIndex];
    if (ow_receive_packet.data_len > COMMAND_MAX_SIZE)
    {
    	bufferIndex=COMMAND_MAX_SIZE-3; // [3 bytes from the end should be the crc for a continuation packet]
    }else{
    	bufferIndex += ow_receive_packet.data_len; // move pointer to end of data
    }

    // Extract received CRC
    ow_receive_packet.crc = (owRxBuffer[bufferIndex] << 8 | (owRxBuffer[bufferIndex+1] & 0xFF ));
    bufferIndex+=2;

    // Calculate CRC for received data

    if (ow_receive_packet.data_len > COMMAND_MAX_SIZE)
    {
    	calculated_crc = util_crc16(&owRxBuffer[1], COMMAND_MAX_SIZE-3);
    }
    else
    {
    	calculated_crc = util_crc16(&owRxBuffer[1], ow_receive_packet.data_len + 8);
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

	process_if_command(&ow_receive_packet, &ow_send_packet);

NextOneWirePacket:
	HAL_Delay(1);
	if(!comms_callin_onewire_send(&ow_send_packet))
	{
		// printf("failed to send onewire response\r\n");
	}
	memset(owRxBuffer, 0, sizeof(owRxBuffer));
    rx_ow_callin_flag = 0;
    if(HAL_HalfDuplex_EnableReceiver(&CALL_IN_UART) != HAL_OK) {
    	// Receive Error
		Error_Handler();
    }

	if (HAL_UARTEx_ReceiveToIdle_IT(&CALL_IN_UART, owRxBuffer, COMMAND_MAX_SIZE) != HAL_OK) {
		// Receive Error
		Error_Handler();
	}
}

bool comms_onewire_slave_start()
{
	ow_packet_count = 0;
	memset(owRxBuffer, 0, sizeof(owRxBuffer));
    rx_ow_callin_flag = 0;
    tx_ow_callin_flag = 0;
    HAL_UART_Abort(&CALL_IN_UART);
    if(HAL_HalfDuplex_EnableReceiver(&CALL_IN_UART) != HAL_OK) {
    	// Receive Error
		Error_Handler();
    }

	if (HAL_UARTEx_ReceiveToIdle_IT(&CALL_IN_UART, owRxBuffer, COMMAND_MAX_SIZE) != HAL_OK) {
		// Receive Error
		Error_Handler();
	}

	return true;
}

bool configure_master()
{
	ow_packet_count = 0;
	bool bRet = true;
	set_module_ID(0);
    HAL_UART_Abort(&CALL_OUT_UART);
    rx_ow_callin_flag = 0;
    tx_ow_callin_flag = 0;

	// configure master
    ModuleManager_Init();
    // Register the master module
    ModuleManager_RegisterMaster(0x00);

	return bRet;
}

bool configure_slave()
{
	bool bRet = false;
	CDC_Stop_ReceiveToIdle();
	ModuleManager_DeInit();
	set_device_role(ROLE_SLAVE);
	// configure slave
	comms_onewire_slave_start();
	return bRet;
}

#define BASE_I2C_ADDRESS 0x20   // Starting address for slaves
#define MAX_SLAVES       6      // Maximum number of slaves in the chain

bool enumerate_slaves()
{
    uint8_t next_address = BASE_I2C_ADDRESS;
    uint8_t slave_count = 1;
    bool bRet = true;

	// register found slaves

    while(slave_count < MAX_SLAVES)
    {
        // Clear the send packet structure before use.
        memset((void*)&ow_send_packet, 0, sizeof(ow_send_packet));

        // Prepare the discovery message.
        ow_send_packet.packet_type = OW_ONE_WIRE;
        ow_send_packet.command = OW_CMD_DISCOVERY;
        ow_send_packet.reserved = slave_count;
        ow_send_packet.addr = next_address;

        // Send the discovery packet to the current slave.
        if (comms_callout_onewire_send(&ow_send_packet))
        {
            // Wait for a response.
            comms_callout_onewire_receive(&ow_receive_packet);

			// Check for an error response.
			if (ow_receive_packet.packet_type != OW_ERROR && ow_receive_packet.packet_type != OW_TIMEOUT)
			{
				//printf("Slave found at I2C address 0x%02X\r\n", next_address);
				// Record the slave if necessary, e.g. store the address.
				ModuleManager_AddSlave(next_address);
				slave_count++;
				next_address++;  // Move on to the next address.

			}
			else
			{
				//printf("Received error from slave at address 0x%02X\r\n", next_address);
				bRet = false;
	            break;
			}
        }
        else
        {
            //printf("Transmission failed for slave at address 0x%02X\r\n", next_address);
            bRet = false;
            break;
        }

        HAL_Delay(50);
    }

    return bRet;

}

// Callback functions
void comms_handle_ow_CallIn_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance == CALL_IN_UART.Instance) {
        // Notify the task
    	rx_ow_callin_flag = 1;
    }
}

void comms_handle_ow_CallIn_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == CALL_IN_UART.Instance) {
        // Notify the task
    	tx_ow_callin_flag = 1;
    }
}


void comms_handle_ow_CallOut_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance == CALL_OUT_UART.Instance) {
        // Notify the task
    	rx_ow_callout_flag = 1;
    }
}

void comms_handle_ow_CallOut_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == CALL_OUT_UART.Instance) {
        // Notify the task
    	tx_ow_callout_flag = 1;
    }
}

void CDC_handle_RxCpltCallback(uint16_t len) {
	rx_flag = 1;
}

void CDC_handle_TxCpltCallback() {
	tx_flag = 1;
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {

    if (huart->Instance == CALL_OUT_UART.Instance) {

    }
}

// In your main.c or elsewhere:
void pulsetrain_complete_callback(uint32_t train_count) {
    // Called after pulse trains are done
}

void sequence_complete_callback(void) {
    // Called after sequence is completed
}

void pulse_complete_callback(uint32_t pulse_count) {
    // Called after pulse is complete
}
