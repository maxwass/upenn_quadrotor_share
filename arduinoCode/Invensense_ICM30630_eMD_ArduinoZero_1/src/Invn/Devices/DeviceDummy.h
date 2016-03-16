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

/** @defgroup DeviceDummy DeviceDummy
 *	@brief    Dummy implementation for device interface
 *  @ingroup  Device
 *	@{
 */

#ifndef _INV_DEVICE_DUMMY_H_
#define _INV_DEVICE_DUMMY_H_

#include "Invn/Devices/Device.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct inv_device_dummy
{
	inv_device_t         base;
	struct inv_device_dummy_sensor_state {
		inv_bool_t         active;
		uint32_t           period;
		uint32_t           timeout;
		uint64_t           next_time;
		inv_sensor_event_t event;
	} sensor_states[INV_SENSOR_TYPE_MAX*2];
	struct {
		float bias[3];
	} sensor_cfg[3];
	uint64_t (*fget_us_time)(void);
} inv_device_dummy_t;

void inv_device_dummy_init(inv_device_dummy_t * self, 
		const inv_host_serif_t * serif, const inv_sensor_listener_t * listener,
		uint64_t (*fget_us_time)(void));

static inline inv_device_t * inv_device_dummy_get_base(inv_device_dummy_t * self)
{
	return (self) ? &self->base : 0;
}

int inv_device_dummy_whoami(void * context, uint8_t * whoami);

int inv_device_dummy_reset(void * context);

int inv_device_dummy_setup(void * context);

int inv_device_dummy_cleanup(void * context);

int inv_device_dummy_load(void * context, int what,
		const uint8_t * image, uint32_t size, inv_bool_t verify, inv_bool_t force);

int inv_device_dummy_get_fw_info(void * context,
		struct inv_fw_version * version);

int inv_device_dummy_set_running_state(void * context, inv_bool_t state);

int inv_device_dummy_ping_sensor(void * context, int sensor);

int inv_device_dummy_start_sensor(void * context, int sensor);

int inv_device_dummy_stop_sensor(void * context, int sensor);

int inv_device_dummy_set_sensor_period(void * context,
		int sensor, uint32_t period);

int inv_device_dummy_set_sensor_timeout(void * context,
		int sensor, uint32_t timeout);

int inv_device_dummy_flush_sensor(void * context, int sensor);

int inv_device_dummy_set_sensor_bias(void * context, int sensor,
		const float bias[3]);

int inv_device_dummy_get_sensor_bias(void * context, int sensor,
		float bias[3]);

int inv_device_dummy_set_sensor_mounting_matrix(void * context,
		int sensor, const float matrix[9]);

int inv_device_dummy_get_sensor_data(void * context, int sensor,
		inv_sensor_event_t * event);

int inv_device_dummy_poll(void * context);

int inv_device_dummy_self_test(void * context, int sensor);

#ifdef __cplusplus
}
#endif

#endif /* _INV_DEVICE_DUMMY_H_ */
