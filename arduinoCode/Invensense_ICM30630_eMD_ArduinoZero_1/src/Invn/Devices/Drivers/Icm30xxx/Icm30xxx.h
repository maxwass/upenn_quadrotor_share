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

/** @defgroup DriverIcm30xxx Icm30xxx driver
 *  @brief    Low-level driver for ICM30xxx devices
 *  @ingroup  Drivers
 *  @{
 */

#ifndef _INV_ICM30XXX_H_
#define _INV_ICM30XXX_H_

#include "Invn/InvBool.h"
#include "Invn/InvError.h"

#include "Invn/Devices/Drivers/Icm30xxx/Icm30xxxCmd.h"
#include "Invn/Devices/Drivers/Icm30xxx/Icm30xxxCtrl.h"
#include "Invn/Devices/Drivers/Icm30xxx/Icm30xxxDefs.h"
#include "Invn/Devices/Drivers/Icm30xxx/Icm30xxxFifo.h"
#include "Invn/Devices/Drivers/Icm30xxx/Icm30xxxLoad.h"
#include "Invn/Devices/Drivers/Icm30xxx/Icm30xxxSetup.h"
#include "Invn/Devices/Drivers/Icm30xxx/Icm30xxxSerif.h"
#include "Invn/Devices/Drivers/Icm30xxx/Icm30xxxTransport.h"
#include "Invn/Devices/Drivers/Icm30xxx/Icm30xxxUartFifo.h"

#include <assert.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief ICM30xxx variant identifier
 */
enum inv_icm30xxx_variant {
	INV_ICM30XXX_VARIANT_ICM30630,
	INV_ICM30XXX_VARIANT_ICM30670,
};

/** @brief Data handler prototype definition
 *  @param[in] context   some context handler
 *  @param[in] sensorid  FIFO Protocol sensor id
 *  @param[in] timestamp sensor event timestamp in tick
 *  @param[in] status    FIFO Protocol status value
 *  @param[in] accurcay  FIFO Protocol accuracy value
 *  @param[in] data      pointer to FIFO Protocol sensor data buffer
 *  @param[in] size      size of FIFO Protocol sensor data buffer
 */
typedef void (*inv_icm30xxx_data_handler_t)(void * context, uint8_t sensorid,
		uint32_t timestamp, uint8_t status, uint8_t accuracy,
		const uint8_t * data, uint16_t size);

/** @brief ICM30xxx FW version strcuture
 */
typedef struct inv_icm30xxx_fw_version {
	uint8_t major, minor, patch;
	char    hash[8];
	uint32_t crc;
} inv_icm30xxx_fw_version_t;

/** @brief ICM30xxx driver states definition
 */
typedef struct inv_icm30xxx {
	struct inv_icm30xxx_serif serif;
	int                       variant;
	struct {
		uint8_t bank;
		uint8_t fifo_idx;
	} cache;
	struct {
		inv_bool_t en;
		int        cnt;
		uint8_t    lpen_val;
	} lp_inhibit;
	/** @brief HW Fifo states structure */
	struct inv_icm30xxx_fifo_state {
		uint8_t   fifo_idx;
		uint8_t   pkt_size;
		uint8_t   pkt_size_byte;
		uint8_t   size;
		uint16_t  size_byte;
		uint8_t   wm;
		uint16_t  wm_byte;
		uint8_t * buffer;
		uint32_t  buffer_max;
		uint32_t  buffer_len;
	} fifo_states[INV_ICM30XXX_FIFO_ID_MAX];
	struct {
		uint8_t * buffer;
		uint32_t  buffer_len;
	} memory;
	struct {
		inv_icm30xxx_data_handler_t data;
		void *                    context;
	} handler;
	struct {
		uint8_t    sensorid;
		uint8_t    command;
		inv_bool_t event;
		union {
			struct inv_icm30xxx_fw_version fwversion;
			float bias[3];
			float gain[9];
			uint32_t clock_rate;
			struct {
				uint8_t who, what;
				int32_t arg;
			} load;
			uint8_t ping;
			struct {
				uint32_t timestamp;
				uint8_t  data[64];
				uint16_t len;
			} sensor_data;
		} payload;
	} response;
} inv_icm30xxx_t;


/** @brief Hook for low-level system sleep() function to be implemented by upper layer
 *  @param[in] ms number of millisecond the calling thread should sleep
 */
extern void inv_icm30xxx_sleep(int ms);


/** @brief Reset and initialize driver states
 *  @param[in] s          handle to driver states structure
 *  @param[in] variant    ICM30xxx variant id (must be a value from enum inv_icm30xxx_variant)
 *  @param[in] buffer     pointer to static buffer used by the driver to temporary store data.
 *  @param[in] buffer_len pointer to static buffer used by the driver to temporary store data.
 */
static inline void inv_icm30xxx_reset_states(struct inv_icm30xxx * s,
		int variant, const struct inv_icm30xxx_serif * serif,
		uint8_t * buffer, uint32_t  buffer_len,
		inv_icm30xxx_data_handler_t data_handler, void * context
)
{
	assert(buffer_len >= 4096);

	memset(s, 0, sizeof(*s));
	s->variant           = variant;
	s->serif             = *serif;
	s->cache.bank        = 0xFF;
	s->cache.fifo_idx    = 0xFF;
	s->memory.buffer     = buffer;
	s->memory.buffer_len = buffer_len;
	s->handler.data      = data_handler;
	s->handler.context   = context;
	inv_icm30xxx_lp_inhibit_reset(s);
}

#ifdef __cplusplus
}
#endif

#endif /* _INV_ICM30XXX_H_ */

/** @} */
