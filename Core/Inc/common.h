/*
 * common.h
 *
 *  Created on: Apr 3, 2024
 *      Author: gvigelet
 */

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

#include <stdint.h>

#define COMMAND_MAX_SIZE 2048

typedef enum {
	OW_START_BYTE = 0xAA,
	OW_END_BYTE = 0xDD,
} USTX_ProtocolTypes;

typedef enum {
	OW_ACK = 0xE0,
	OW_NAK = 0xE1,
	OW_CMD = 0xE2,
	OW_RESP = 0xE3,
	OW_DATA = 0xE4,
	OW_JSON = 0xE5,
	OW_TX7332 = 0xE6,
	OW_AFE_READ = 0xE7,
	OW_AFE_SEND = 0xE8,
	OW_I2C_PASSTHRU = 0xE9,
	OW_CONTROLLER = 0xEA,
	OW_BAD_PARSE = 0xEC,
	OW_BAD_CRC = 0xED,
	OW_UNKNOWN = 0xEE,
	OW_ERROR = 0xEF,

} UartPacketTypes;

typedef enum {
	OW_CODE_SUCCESS = 0x00,
	OW_CODE_IDENT_ERROR = 0xFD,
	OW_CODE_DATA_ERROR = 0xFE,
	OW_CODE_ERROR = 0xFF,
} UstxErrorCodes;

typedef enum {
	OW_CMD_PING = 0x00,
	OW_CMD_PONG = 0x01,
	OW_CMD_VERSION = 0x02,
	OW_CMD_ECHO = 0x03,
	OW_CMD_TOGGLE_LED = 0x04,
	OW_CMD_HWID = 0x05,
	OW_CMD_NOP = 0x0E,
	OW_CMD_RESET = 0x0F,
} UstxGlobalCommands;

typedef enum {
	OW_CTRL_SCAN_I2C = 0x10,
	OW_CTRL_WRITE_I2C = 0x11,
	OW_CTRL_READ_I2C = 0x12,
	OW_CTRL_SET_SWTRIG = 0x13,
	OW_CTRL_GET_SWTRIG = 0x14,
	OW_CTRL_START_SWTRIG = 0x15,
	OW_CTRL_STOP_SWTRIG = 0x16,
	OW_CTRL_STATUS_SWTRIG = 0x17,
	OW_CTRL_SET_HV = 0x18,
	OW_CTRL_GET_HV = 0x19,
} UstxControllerCommands;

typedef enum {
	OW_TX7332_STATUS = 0x20,
	OW_TX7332_ENUM = 0x21,
	OW_TX7332_WREG = 0x22,
	OW_TX7332_RREG = 0x23,
	OW_TX7332_WBLOCK = 0x24,
	OW_TX7332_DEMO = 0x2D,
	OW_TX7332_RESET = 0x2F,
} UstxTX7332Commands;

typedef enum {
	OW_AFE_STATUS = 0x30,
	OW_AFE_ENUM_TX7332 = 0x31,
} UstxAfeCommands;

typedef struct  {
	uint16_t id;
	uint8_t packet_type;
	uint8_t command;
	uint8_t addr;
	uint8_t reserved;
	uint16_t data_len;
	uint8_t* data;
	uint16_t crc;
} UartPacket;

#endif /* INC_COMMON_H_ */
