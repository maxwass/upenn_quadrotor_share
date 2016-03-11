/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2015-2015 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */

/**
 * @file
 * @brief Bridge USB <-> SPI, protocol encode/decod and internal bridge register
 * Protocol frame format :
 * 0x55 0xAA 0x55 0xAA <CMD> <ADDR_LSB> <NB_BYTES_LSB> <NB_BYTES_MSB> <DATA...>
 */

// We can not processs warning as error on whole code since Arduino libraries
#pragma GCC diagnostic push
#pragma GCC diagnostic warning "-Werror"

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <string.h>

#include "Bridge.h"
#include "ArduinoEmdAdapter.h"
#include "BridgeVersion.h"

/* Private define ------------------------------------------------------------*/

// synchronization header values
#define BRIDGE_PROTO_SYNC_1             0x55
#define BRIDGE_PROTO_SYNC_2             0xAA
#define BRIDGE_PROTO_SYNC_3             0x55
#define BRIDGE_PROTO_SYNC_4             0xAA


// Sensor register write operation on the SPI
#define BRIDGE_PROTO_CMD_WR_SENSOR      0x02
// Sensor register read operation on the SPI
#define BRIDGE_PROTO_CMD_RD_SENSOR      0x03
// Register read bridge internal register
#define BRIDGE_PROTO_CMD_RD_BRIDGE_REG  0x05

// bit to add in command for answer from I/O expander
#define BRIDGE_PROTO_CMD_ACK_MASK       0x80

// protocol header size
#define BRIDGE_PROTO_HEADER_SIZE        8

// bridge internal register address
enum
{
	REG_WHO_AM_I                = 0x00,
	REG_TEST                    = 0x01,
	REG_FW_VERSION_MAJOR        = 0x02,
	REG_FW_VERSION_MINOR        = 0x03,
	REG_FW_VERSION_REVISION     = 0x04,
	REG_FW_VERSION_SUFFIX       = 0x05,
	REG_FW_NOT_USED_2           = 0x06,
	REG_FW_ADAPTER_MAX_SIZE_03  = 0x07,
	REG_FW_ADAPTER_MAX_SIZE_02  = 0x08,
	REG_FW_ADAPTER_MAX_SIZE_01  = 0x09,
	REG_FW_ADAPTER_MAX_SIZE_00  = 0x0A,
};

// Firmware info
#define FW_WHO_AM_I           0xCD
#define FW_VERSION_MAJOR      BRIDGE_VERSION_MAJOR
#define FW_VERSION_MINOR      BRIDGE_VERSION_MINOR
#define FW_VERSION_REVISION   BRIDGE_VERSION_REVISION

#if (INV_ARDUINO_ADAPTER_SERIF_MAX_TRANSACTION_SIZE + BRIDGE_PROTO_HEADER_SIZE) > BRIDGE_MAX_DATA_RCV
#error invalid INV_ARDUINO_ADAPTER_SERIF_MAX_TRANSACTION_SIZE size is too big (should be <= to BRIDGE_MAX_DATA_RCV - BRIDGE_PROTO_HEADER_SIZE)
#endif

/* Private typedef ------------------------------------------------------------*/


// Protocol State Machine
typedef enum
{
  BRIDGE_PROTO_STATE_IDLE = 0,
  BRIDGE_PROTO_STATE_SYNC_1,   // equivalent of BRIDGE_PROTO_STATE_IDLE
  BRIDGE_PROTO_STATE_SYNC_2,
  BRIDGE_PROTO_STATE_SYNC_3,
  BRIDGE_PROTO_STATE_SYNC_4,
  BRIDGE_PROTO_STATE_CMD,
  BRIDGE_PROTO_STATE_ADDR,
  BRIDGE_PROTO_STATE_NB_BYTE_0,
  BRIDGE_PROTO_STATE_NB_BYTE_1,
  BRIDGE_PROTO_STATE_DATA
} bridge_proto_state_t;

/* Private variables ---------------------------------------------------------*/

static bridge_proto_state_t bridge_proto_state = BRIDGE_PROTO_STATE_IDLE;

// command value
static uint8_t bridge_cmd;

// command register address
static uint8_t bridge_addr;

// command data number of bytes
static uint16_t bridge_nb_bytes;

// command number of data bytes received
static uint16_t bridge_nb_bytes_rcv;

// buffer for data bytes received
static uint8_t bridge_data_rcv[BRIDGE_MAX_DATA_RCV];

/* Public function prototypes ------------------------------------------------*/

static void (*sSendData)(uint8_t *data, uint16_t len);
static void (*sInitDeviceIoHAL)(void);
static void (*sTransferDeviceIoHALregWrite)(uint8_t reg, uint8_t *data, uint16_t len);
static void (*sTransferDeviceIoHALregRead)(uint8_t reg, uint8_t *data, uint16_t len);

/* Private function code -----------------------------------------------------*/


/** @brief Read data from sensor and answer request
 *  @return     0 on success, negative value on error
 */
static int bridge_answer_rd_sensor(void)
{
	const uint8_t cmdHeader[] = {BRIDGE_PROTO_SYNC_1, BRIDGE_PROTO_SYNC_2, BRIDGE_PROTO_SYNC_3, BRIDGE_PROTO_SYNC_4, (BRIDGE_PROTO_CMD_RD_SENSOR | BRIDGE_PROTO_CMD_ACK_MASK)};
	uint16_t len = 0;
	uint8_t data[BRIDGE_MAX_DATA_RCV + BRIDGE_PROTO_HEADER_SIZE];

	for (len = 0; len < sizeof(cmdHeader); len++)
	{
		data[len] = cmdHeader[len];
	}
	
	data[len++] = bridge_addr;
	data[len++] = ((uint8_t)(bridge_nb_bytes & 0xff));
	data[len++] = ((uint8_t)((bridge_nb_bytes >> 8) & 0xff));

	sTransferDeviceIoHALregRead(bridge_addr, &data[len++], bridge_nb_bytes);
	
	if(len > sizeof(data))
		return -1;
	
	sSendData(data, bridge_nb_bytes + BRIDGE_PROTO_HEADER_SIZE);
	return 0;
}


/** @brief Read bridge register and answer request
 *  @return     0 on success, negative value on error
 */
static int bridge_answer_rd_reg(void)
{
	const uint8_t cmdHeader[] = {BRIDGE_PROTO_SYNC_1, BRIDGE_PROTO_SYNC_2, BRIDGE_PROTO_SYNC_3, BRIDGE_PROTO_SYNC_4, (BRIDGE_PROTO_CMD_RD_BRIDGE_REG | BRIDGE_PROTO_CMD_ACK_MASK)};
	uint8_t currentRegAddr;
	uint16_t len = 0;
	uint16_t curentsize;
	uint8_t data[BRIDGE_MAX_DATA_RCV + BRIDGE_PROTO_HEADER_SIZE];
	bool readEnd;
	
	for (len = 0; len < sizeof(cmdHeader); len++)
	{
		data[len] = cmdHeader[len];
	}
	
	data[len++] = bridge_addr;
	currentRegAddr = bridge_addr;
	data[len++] = ((uint8_t)(bridge_nb_bytes & 0xff));
	data[len++] = ((uint8_t)((bridge_nb_bytes >> 8) & 0xff));
	curentsize = bridge_nb_bytes;
	
	if (curentsize > (sizeof(data) - len))
	{
		return -1;
	}
	
	readEnd = false;
	while (readEnd == false)
	{
		switch (currentRegAddr)
		{
			case REG_WHO_AM_I :
				data[len++] = FW_WHO_AM_I;
				break;
			case REG_TEST :
				data[len++] = 0;
				break;
			case REG_FW_VERSION_MAJOR :
				// set major version
				data[len++] = FW_VERSION_MAJOR;
				break;
			case REG_FW_VERSION_MINOR :
				data[len++] = FW_VERSION_MINOR;
				break;
			case REG_FW_VERSION_REVISION :
				data[len++] = FW_VERSION_REVISION;
				break;
			case REG_FW_VERSION_SUFFIX :
				data[len++] = BRIDGE_VERSION_SUFFIX;
				break;
			case REG_FW_NOT_USED_2 :
				data[len++] = 0;
				break;
			case REG_FW_ADAPTER_MAX_SIZE_03 :
				data[len++] = ((INV_ARDUINO_EMD_ADAPTER_SERIF_MAX_TRANSACTION_SIZE >> 24) & 0xFF);
				break;
			case REG_FW_ADAPTER_MAX_SIZE_02 :
				data[len++] = ((INV_ARDUINO_EMD_ADAPTER_SERIF_MAX_TRANSACTION_SIZE >> 16) & 0xFF);
				break;
			case REG_FW_ADAPTER_MAX_SIZE_01 :
				data[len++] = ((INV_ARDUINO_EMD_ADAPTER_SERIF_MAX_TRANSACTION_SIZE >> 8) & 0xFF);
				break;
			case REG_FW_ADAPTER_MAX_SIZE_00 :
				data[len++] = (INV_ARDUINO_EMD_ADAPTER_SERIF_MAX_TRANSACTION_SIZE & 0xFF);
				// no more register, fill rest of data with 0
				memset(&data[len], 0, (curentsize - 1));
				len += (curentsize - 1);
				readEnd = true;
				break;
			default :
				break;
		}
		
		// next register
		currentRegAddr++;
		
		// decrease size to read
		curentsize--;
		if(curentsize == 0)
		{
			readEnd = true;
		}
	}
	
	sSendData(data, bridge_nb_bytes + BRIDGE_PROTO_HEADER_SIZE);
	
	return 0;
}


 /** @brief Answer write sensor request
 *  @return     0 on success, negative value on error
 */
static int bridge_answer_wr_sensor(void)
{
	const uint8_t cmdHeader[] = {BRIDGE_PROTO_SYNC_1, BRIDGE_PROTO_SYNC_2, BRIDGE_PROTO_SYNC_3, BRIDGE_PROTO_SYNC_4, (BRIDGE_PROTO_CMD_WR_SENSOR | BRIDGE_PROTO_CMD_ACK_MASK)};
	uint16_t len = 0;
	uint8_t data[BRIDGE_PROTO_HEADER_SIZE];

	for (len = 0; len < sizeof(cmdHeader); len++)
	{
		data[len] = cmdHeader[len];
	}
	data[len++] = bridge_addr;
	data[len++] = ((uint8_t)(bridge_nb_bytes & 0xff));
	data[len++] = ((uint8_t)((bridge_nb_bytes >> 8) & 0xff));
	
	sSendData(data, len);
	
	return 0;
}


 /** @brief Process command data, send data to sensor
 *  @return     none
 */
static void bridge_process_data(void) 
{
	switch(bridge_cmd)
	{
		case BRIDGE_PROTO_CMD_WR_SENSOR:     
			sTransferDeviceIoHALregWrite(bridge_addr, bridge_data_rcv, bridge_nb_bytes_rcv);
			break;

		default:
			break;
	}
}


/** @brief Receive bridge data form PC
 *  @param[in]  c    byte received
 *  @return     0 on success, negative value on error
 */
int bridge_receive_byte(uint8_t c)
{
	switch (bridge_proto_state)
	{
		case BRIDGE_PROTO_STATE_IDLE:
		case BRIDGE_PROTO_STATE_SYNC_1:
			if (c == BRIDGE_PROTO_SYNC_1)
			{
				bridge_proto_state = BRIDGE_PROTO_STATE_SYNC_2;
			}
			else
			{
				bridge_proto_state = BRIDGE_PROTO_STATE_IDLE;
			}
			break;

		case BRIDGE_PROTO_STATE_SYNC_2:
			if (c == BRIDGE_PROTO_SYNC_2)
			{
				bridge_proto_state = BRIDGE_PROTO_STATE_SYNC_3;
			}
			else
			{
				bridge_proto_state = BRIDGE_PROTO_STATE_IDLE;
			}
			break;
      
		case BRIDGE_PROTO_STATE_SYNC_3:
			if (c == BRIDGE_PROTO_SYNC_3)
			{
				bridge_proto_state = BRIDGE_PROTO_STATE_SYNC_4;
			}
			else
			{
				bridge_proto_state = BRIDGE_PROTO_STATE_IDLE;
			}
			break;

		case BRIDGE_PROTO_STATE_SYNC_4:
			if (c == BRIDGE_PROTO_SYNC_4)
			{
				bridge_proto_state = BRIDGE_PROTO_STATE_CMD;
			}
			else
			{
				bridge_proto_state = BRIDGE_PROTO_STATE_IDLE;
			}
			break;

		case BRIDGE_PROTO_STATE_CMD:
			bridge_cmd = c;
			bridge_proto_state = BRIDGE_PROTO_STATE_ADDR;
			break;
    
		case BRIDGE_PROTO_STATE_ADDR:
			bridge_addr = c;
			bridge_proto_state = BRIDGE_PROTO_STATE_NB_BYTE_0;
			break;

		case BRIDGE_PROTO_STATE_NB_BYTE_0:
			bridge_nb_bytes = (uint16_t)c;
			bridge_proto_state = BRIDGE_PROTO_STATE_NB_BYTE_1;
			break;

		case BRIDGE_PROTO_STATE_NB_BYTE_1:
			bridge_nb_bytes |= (uint16_t)(c << 8);
			bridge_nb_bytes_rcv = 0;

		switch(bridge_cmd)
		{
			case BRIDGE_PROTO_CMD_RD_SENSOR :
				// Answer sensor read request
				if(bridge_answer_rd_sensor() != 0)
					return -1;
				bridge_proto_state = BRIDGE_PROTO_STATE_IDLE;
			break;
			case BRIDGE_PROTO_CMD_RD_BRIDGE_REG :
				//Request read register
				if(bridge_answer_rd_reg() != 0)
					return -1;
				bridge_proto_state = BRIDGE_PROTO_STATE_IDLE;
			break;

			default :
				bridge_proto_state = BRIDGE_PROTO_STATE_DATA;
			break;
		}
		break;

		case BRIDGE_PROTO_STATE_DATA:
			bridge_data_rcv[bridge_nb_bytes_rcv] = c;
			bridge_nb_bytes_rcv++;        
			if (bridge_nb_bytes_rcv == bridge_nb_bytes)
			{
				bridge_process_data(); 
				if(bridge_answer_wr_sensor() != 0)
					return -1;
				bridge_proto_state = BRIDGE_PROTO_STATE_IDLE;
			}
			break;
	}
	return 0;
}

/* Public function code ------------------------------------------------------*/


void bridge_init(void (*sendData)(uint8_t *data, uint16_t len),
                                const bridge_device_io_hal_callback_t *DeviceIoHalCallback)
{
	// instantiate send data out callback
	sSendData = sendData; 
	
	// instantiate sensor HAL callback
	sInitDeviceIoHAL = DeviceIoHalCallback->init;
	sTransferDeviceIoHALregWrite = DeviceIoHalCallback->regWrite;
	sTransferDeviceIoHALregRead = DeviceIoHalCallback->regRead;
	
	// initialize Device Io HAL
	sInitDeviceIoHAL();
}

int bridge_rcv_data(const uint8_t *data, uint16_t len)
{
	int i;
	
	for(i = 0; i < len; i++)
	{
		if(bridge_receive_byte(data[i]) != 0)
		{
			return -1;
		}
		
	}
	return 0;
}


uint8_t bridge_is_processing_data(void)
{
	if (bridge_proto_state == BRIDGE_PROTO_STATE_IDLE) 
	{
		return 0;
	}
	else 
	{
		return -1;
	}
}
