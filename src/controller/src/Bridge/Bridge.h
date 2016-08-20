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
 
#ifndef BRIDGE_H
#define BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

// Fifo Protocol API is include in the remote sensor protocol library
#include "Invn/FifoProtocol/FifoProtocol.h"
#include <stdint.h>
#include <stdbool.h>

// maximum number of data bytes received 
#define BRIDGE_MAX_DATA_RCV       4200

// Low level sensor driver function used by the remote sensor protocol library
typedef struct
{
  // Initialyze low level driver to communicate with sensor
  void (*init)(void);
  // Send data to sensor
  void (*regWrite)(uint8_t reg, uint8_t *data, uint16_t len);
  // Read data from sensor
  void (*regRead)(uint8_t reg, uint8_t *data, uint16_t len);
  // close low level driver to communicate with sensor
  void (*close)(void);
} bridge_device_io_hal_callback_t;


/** @brief Initialize bridgeprotocol
 *  @param[in]  sendData  function used to send data to the host : data: data to send, len size of data
 *  @param[in]  DeviceIoHalCallback : Low level sensor driver function used by the bridge
 *  @return     none
 */
void bridge_init(void (*sendData)(uint8_t *data, uint16_t len),
                                const bridge_device_io_hal_callback_t *DeviceIoHalCallback);


/** @brief Give new received data to the bridge, called by application when new host data is received
 *  @param[in]  data  data received
 *  @param[in]  len   size of data
 *  @return     0 on success, negative value on error
 */
int bridge_rcv_data(const uint8_t *data, uint16_t len);


/** @brief Test if the bridge is processing a frame or if it's idle
 *  @return     0 bridge idle, -1 otherwise 
 */
uint8_t bridge_is_processing_data(void);

#ifdef __cplusplus
}
#endif

#endif /* BRIDGE_H */ 
