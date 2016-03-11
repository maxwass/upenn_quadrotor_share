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
*********************************************************************************
* @file InvnBridge.cpp
*********************************************************************************
*/

// Warning are processed as error
#pragma GCC diagnostic push
#pragma GCC diagnostic warning "-Werror"

/* Includes ------------------------------------------------------------------*/

#include <SPI.h>
#include "DeviceIoHal.h"
#include "InvnBridge.h"

/* Private define ------------------------------------------------------------*/

// Choose which serial has to be used (Serial1, Serial, SerialUSB)
#define INVN_BRIDGE_SERIAL Serial

/* InvnBridge private function code ------------------------------------------------------*/

void invn_bridge_send_data(uint8_t *data, uint16_t len);
void invn_bridge_init(void);
void Invn_Bridge_regWrite(uint8_t reg, uint8_t *buf, uint16_t bufSize);
void invn_bridge_reg_read(uint8_t reg, uint8_t *buf, uint16_t bufSize);
void invn_bridge_close(void);
 
 /* Public variables ---------------------------------------------------------*/
 
/**
* Function called for sensor hal access
*/
static bridge_device_io_hal_callback_t DeviceIoHalCallback = 
{
	.init     = invn_bridge_init, 
	.regWrite = Invn_Bridge_regWrite, 
	.regRead  = invn_bridge_reg_read, 
	.close    = invn_bridge_close,
};

 /* Invn_Bridge methodes function code ------------------------------------------------------*/


/** @brief Constructor for Invn_Bridge
 *  @return     none
 */
InvnBridge::InvnBridge(void)
{
}


/** @brief Init for InvnBridge
 *  @return     none
 */
void InvnBridge::init(void)
{
	// open serial port TX0/RX0 with FTDI cable to communicate to TestApp
	INVN_BRIDGE_SERIAL.begin(921600);
	
	// Init bridge
	bridge_init(invn_bridge_send_data, &DeviceIoHalCallback);
}


/** @brief Process for InvnBridge
 *  @return     none
 */
void InvnBridge::process(void)
{
	// process interrupt pending
	bridge_is_processing_data();
	
	// read UART buffer to compute bytes received
	uint16_t nbByteAvailable = INVN_BRIDGE_SERIAL.available();
	
	if (nbByteAvailable > 0) {
		uint16_t i;
		
		// Test buffer overflow
		for(i = 0; (i < nbByteAvailable) && (i < BRIDGE_MAX_DATA_RCV); i++) {
			recvBuffer[i] = (uint8_t)INVN_BRIDGE_SERIAL.read();
		}
		
		// process command received
		bridge_rcv_data(recvBuffer, nbByteAvailable);
	}
}

/* Public function code ------------------------------------------------------*/


/** @brief Send data method InvnBridge
 *  @param[in]  data  data to send
 *  @param[in]  len   length of packet
 *  @return     none
 */
void invn_bridge_send_data(uint8_t *data, uint16_t len)
{
	INVN_BRIDGE_SERIAL.write(data, len);
	INVN_BRIDGE_SERIAL.flush();
}


/** @brief Low level driver initialization to communicate to the sensor
 *  @return     none
 */
void invn_bridge_init(void)
{
	inv_device_io_hal_init();
}


/** @brief Write sensor registers
 *  @param[in]  reg      register address to send out
 *  @param[in]  buf      data buffer to send out over SPI
 *  @param[in]  bufSize  byte size to transfer
 *  @return     none
 */
void Invn_Bridge_regWrite(uint8_t reg, uint8_t *buf, uint16_t bufSize)
{
	inv_device_io_hal_reg_write(reg, buf, bufSize);
}


/** @brief Read sensor registers
 *  @param[in]  reg      register address to send out over SPI
 *  @param[in]  buf      buffer to completed by data received over SPI
 *  @param[in]  bufSize  byte size to transfer
 *  @return     none
 */
void invn_bridge_reg_read(uint8_t reg, uint8_t *buf, uint16_t bufSize)
{
	inv_device_io_hal_reg_read(reg, buf, bufSize);
}


/** @brief Low level driver close
 *  @return     none
 */
void invn_bridge_close(void)
{
	inv_device_io_hal_close();
}
