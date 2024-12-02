/*
 * if_commands.c
 *
 *  Created on: Nov 15, 2024
 *      Author: GeorgeVigelette
 */

#include "main.h"
#include "if_commands.h"
#include "common.h"
#include "i2c_master.h"
#include "i2c_protocol.h"
#include "trigger.h"
#include "tx7332.h"

#include <stdio.h>
#include <string.h>

extern uint8_t FIRMWARE_VERSION_DATA[3];
extern TX7332 tx[2];

static uint32_t id_words[3] = {0};
static char retTriggerJson[0xFF];
uint8_t receive_afe_status[I2C_STATUS_SIZE] = {0};
uint8_t receive_afe_buff[I2C_BUFFER_SIZE] = {0};
uint8_t send_afe_buff[I2C_BUFFER_SIZE] = {0};

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
static void process_afe_read_status(UartPacket *uartResp, UartPacket cmd);
static void process_afe_read(UartPacket *uartResp, UartPacket cmd);

static void print_uart_packet(const UartPacket* packet) {
    printf("ID: 0x%04X\r\n", packet->id);
    printf("Packet Type: 0x%02X\r\n", packet->packet_type);
    printf("Command: 0x%02X\r\n", packet->command);
    printf("Data Length: %d\r\n", packet->data_len);
    printf("CRC: 0x%04X\r\n", packet->crc);
    printf("Data: ");
    for (int i = 0; i < packet->data_len; i++) {
        printf("0x%02X ", packet->data[i]);
    }
    printf("\r\n");
}

static void process_afe_read(UartPacket *uartResp, UartPacket cmd)
{
	uint16_t rx_len =  cmd.command;
	// I2C_TX_Packet afe_data_packet;
	uint8_t slave_addr = cmd.addr;
	if(found_address_count == 0){
		printf("No AFE's found\r\n");
		uartResp->id = cmd.id;
		uartResp->packet_type = OW_ERROR;
		uartResp->command = cmd.command;
		return;
	}else{
		uartResp->id = cmd.id;
		uartResp->packet_type = cmd.packet_type;
		uartResp->command = cmd.command;
		uartResp->data_len = 0;
	}

	rx_len = read_data_register_of_slave(slave_addr, receive_afe_buff, rx_len);
	// printf("Received %d Bytes \r\n", rx_len);
	// printBuffer(receive_afe_buff, rx_len);
	uartResp->data_len = rx_len;
	uartResp->data = receive_afe_buff;
	//i2c_packet_fromBuffer(receive_afe_buff, &afe_data_packet);
	// i2c_tx_packet_print(&afe_data_packet);
}

static void process_basic_command(UartPacket *uartResp, UartPacket cmd)
{
	switch (cmd.command)
	{
	case OW_CMD_NOP:
		uartResp->command = OW_CMD_NOP;
		break;
	case OW_CMD_PING:
		uartResp->command = OW_CMD_PING;
		break;
	case OW_CMD_PONG:
		uartResp->command = OW_CMD_PONG;
		break;
	case OW_CMD_VERSION:
		uartResp->command = OW_CMD_VERSION;
		uartResp->data_len = sizeof(FIRMWARE_VERSION_DATA);
		uartResp->data = FIRMWARE_VERSION_DATA;
		break;
	case OW_CMD_HWID:
		uartResp->command = OW_CMD_HWID;
		id_words[0] = HAL_GetUIDw0();
		id_words[1] = HAL_GetUIDw1();
		id_words[2] = HAL_GetUIDw2();
		uartResp->data_len = 16;
		uartResp->data = (uint8_t *)&id_words;
		break;
	case OW_CMD_ECHO:
		// exact copy
		uartResp->id = cmd.id;
		uartResp->packet_type = cmd.packet_type;
		uartResp->command = cmd.command;
		uartResp->data_len = cmd.data_len;
		uartResp->data = cmd.data;
		break;
	case OW_CMD_TOGGLE_LED:
		uartResp->id = cmd.id;
		uartResp->packet_type = cmd.packet_type;
		uartResp->command = cmd.command;
		HAL_GPIO_TogglePin(TRANSMIT_LED_GPIO_Port, TRANSMIT_LED_Pin);
		break;
	default:
		uartResp->data_len = 0;
		uartResp->packet_type = OW_UNKNOWN;
		// uartResp.data = (uint8_t*)&cmd.tag;
		break;
	}
}

static void process_afe_send(UartPacket *uartResp, UartPacket cmd)
{
	uint16_t send_len = 0;
	I2C_TX_Packet send_afe_packet;
	uint8_t slave_addr = cmd.addr;
	// initialize send packet
	send_afe_packet.id = cmd.id;
	send_afe_packet.cmd = cmd.command;
	send_afe_packet.reserved = cmd.reserved;
	send_afe_packet.data_len = cmd.data_len;

	if(found_address_count == 0){
		printf("No AFE's found\r\n");
		uartResp->id = cmd.id;
		uartResp->packet_type = OW_ERROR;
		uartResp->command = cmd.command;
		return;
	}else{
		uartResp->id = cmd.id;
		uartResp->command = cmd.command;
		uartResp->data_len = 0;
	}

	if(send_afe_packet.data_len==0){
		send_afe_packet.pData = NULL;
	}else{
		send_afe_packet.pData = cmd.data;
	}

	send_len = i2c_packet_toBuffer(&send_afe_packet, send_afe_buff);
	send_buffer_to_slave(slave_addr, send_afe_buff, send_len);
	HAL_Delay(2);
	process_afe_read_status(uartResp, cmd);
}


static void process_afe_read_status(UartPacket *uartResp, UartPacket cmd)
{
	uint16_t rx_len = 0;
	uint8_t slave_addr = cmd.addr;
	if(found_address_count == 0){
		printf("No AFE's found\r\n");
		uartResp->id = cmd.id;
		uartResp->packet_type = OW_ERROR;
		uartResp->command = cmd.command;
		return;
	}else{
		uartResp->id = cmd.id;
		uartResp->packet_type = cmd.packet_type;
		uartResp->command = cmd.command;
		uartResp->data_len = 0;
	}

	rx_len = read_status_register_of_slave(slave_addr, receive_afe_status, I2C_STATUS_SIZE);
	//printf("Received %d Bytes \r\n", rx_len);
	uartResp->data_len = rx_len;
	uartResp->data = receive_afe_status;
}


static void CONTROLLER_ProcessCommand(UartPacket *uartResp, UartPacket cmd)
{
	switch (cmd.command)
	{
		case OW_CMD_PING:
			uartResp->command = cmd.command;
			uartResp->addr = cmd.addr;
			uartResp->reserved = cmd.reserved;
			break;
		case OW_CMD_PONG:
			uartResp->command = cmd.command;
			uartResp->addr = cmd.addr;
			uartResp->reserved = cmd.reserved;
			break;
		case OW_CMD_VERSION:
			uartResp->command = cmd.command;
			uartResp->addr = cmd.addr;
			uartResp->reserved = cmd.reserved;
			uartResp->data_len = sizeof(FIRMWARE_VERSION_DATA);
			uartResp->data = FIRMWARE_VERSION_DATA;
			break;
		case OW_CMD_ECHO:
			// exact copy
			uartResp->id = cmd.id;
			uartResp->packet_type = cmd.packet_type;
			uartResp->command = cmd.command;
			uartResp->addr = cmd.addr;
			uartResp->reserved = cmd.reserved;
			uartResp->data_len = cmd.data_len;
			uartResp->data = cmd.data;
			break;
		case OW_CMD_TOGGLE_LED:
			uartResp->id = cmd.id;
			uartResp->packet_type = cmd.packet_type;
			uartResp->command = cmd.command;
			//HAL_GPIO_TogglePin(HB_LEDn_GPIO_Port, HB_LEDn_Pin); //no led pins declared
			break;
		case OW_CMD_HWID:
			uartResp->command = OW_CMD_HWID;
			uartResp->addr = cmd.addr;
			uartResp->reserved = cmd.reserved;
			id_words[0] = HAL_GetUIDw0();
			id_words[1] = HAL_GetUIDw1();
			id_words[2] = HAL_GetUIDw2();
			uartResp->data_len = 16;
			uartResp->data = (uint8_t *)&id_words;
			break;
		case OW_CTRL_SCAN_I2C:
			uartResp->id = cmd.id;
			uartResp->packet_type = cmd.packet_type;
			uartResp->command = cmd.command;
			uartResp->addr = cmd.addr;
			uartResp->reserved = cmd.reserved;
			//found_address_count = I2C_scan(found_addresses, MAX_FOUND_ADDRESSES, false);
			//uartResp->data_len = found_address_count;
			//uartResp->data = found_addresses;
			break;
		case OW_CTRL_START_SWTRIG:
			uartResp->command = cmd.command;
			uartResp->addr = cmd.addr;
			uartResp->reserved = cmd.reserved;
			uartResp->data_len = 0;
			start_trigger_pulse();
			break;
		case OW_CTRL_STOP_SWTRIG:
			uartResp->command = cmd.command;
			uartResp->addr = cmd.addr;
			uartResp->reserved = cmd.reserved;
			uartResp->data_len = 0;
			stop_trigger_pulse();
			break;
		case OW_CTRL_SET_SWTRIG:
			uartResp->command = cmd.command;
			uartResp->addr = cmd.addr;
			uartResp->reserved = cmd.reserved;
			uartResp->data_len = 0;

			if(!set_trigger_data((char *)cmd.data, cmd.data_len))
			{
				uartResp->packet_type = OW_ERROR;
			}

			break;
		case OW_CTRL_GET_SWTRIG:
			// refresh state
			if(!get_trigger_data(retTriggerJson, 0xFF))
			{
				uartResp->packet_type = OW_ERROR;
				break;
			}
			uartResp->command = cmd.command;
			uartResp->addr = cmd.addr;
			uartResp->reserved = cmd.reserved;
			uartResp->data_len = strlen(retTriggerJson);
			uartResp->data = (uint8_t *)retTriggerJson;
			break;
		case OW_CMD_RESET:
			uartResp->command = cmd.command;
			uartResp->addr = cmd.addr;
			uartResp->reserved = cmd.reserved;
			uartResp->data_len = 0;
		    // Reset the board
		    //NVIC_SystemReset();
			break;
		case OW_CTRL_ENUM_TX7332:
			// send array of tx chip counts 0,1,2,3,4,...
			printf("Enumerate TX7332 ICs %d \r\n", ARRAY_SIZE(tx));
			uartResp->command = cmd.command;
			uartResp->addr = cmd.addr;
			uartResp->reserved = (uint8_t)ARRAY_SIZE(tx);
			uartResp->data = NULL;
			break;
		default:
			uartResp->addr = 0;
			uartResp->reserved = 0;
			uartResp->data_len = 0;
			uartResp->packet_type = OW_UNKNOWN;
			break;
	}

}
#if 0
static void JSON_ProcessCommand(UartPacket *uartResp, UartPacket cmd)
{
	// json parser
    jsmn_parser parser;
    parser.size = sizeof(parser);
    jsmn_init(&parser, NULL);
    jsmntok_t t[16];
    jsmnerr_t ret = jsmn_parse(&parser, (char *)cmd.data, cmd.data_len, t,
				 sizeof(t) / sizeof(t[0]), NULL);
    printf("Found %d Tokens\r\n", ret);
	switch (cmd.command)
	{
	case OW_CMD_NOP:
		uartResp->command = OW_CMD_NOP;
		break;
	case OW_CMD_ECHO:
		// exact copy
		uartResp->id = cmd.id;
		uartResp->packet_type = cmd.packet_type;
		uartResp->command = cmd.command;
		uartResp->data_len = cmd.data_len;
		uartResp->data = cmd.data;
		break;
	default:
		uartResp->data_len = 0;
		uartResp->packet_type = OW_UNKNOWN;
		break;
	}
}
#endif

UartPacket process_if_command(UartPacket cmd)
{
	UartPacket uartResp;
	// I2C_TX_Packet i2c_packet;
	(void)print_uart_packet;

	uartResp.id = cmd.id;
	uartResp.packet_type = OW_RESP;
	uartResp.addr = 0;
	uartResp.reserved = 0;
	uartResp.data_len = 0;
	uartResp.data = 0;
	switch (cmd.packet_type)
	{
	case OW_CMD:
		process_basic_command(&uartResp, cmd);
		break;
	case OW_JSON:
		//JSON_ProcessCommand(&uartResp, cmd);
		break;
	case OW_CONTROLLER:
		//process by the USTX Controller
		CONTROLLER_ProcessCommand(&uartResp, cmd);
		break;
	case OW_AFE_STATUS:
		process_afe_read_status(&uartResp, cmd);
		break;
	case OW_AFE_READ:
		process_afe_read(&uartResp, cmd);
		break;
	case OW_AFE_SEND:
		process_afe_send(&uartResp, cmd);
		break;
#if 0
	case OW_I2C_PASSTHRU:

		print_uart_packet(&cmd);

        // printBuffer(cmd.data, 10);
		i2c_packet_fromBuffer(cmd.data, &i2c_packet);
		// i2c_tx_packet_print(&i2c_packet);

		HAL_Delay(20);
		send_buffer_to_slave(cmd.command, cmd.data, 10);

		break;
#endif
	default:
		uartResp.data_len = 0;
		uartResp.packet_type = OW_UNKNOWN;
		// uartResp.data = (uint8_t*)&cmd.tag;
		break;
	}

	return uartResp;

}

