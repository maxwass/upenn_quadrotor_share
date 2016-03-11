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

/** @defgroup DeviceIcm30xxx DeviceIcm30xxx
 *	@brief    Concrete implementation of the 'Device' interface for Icm30xxx devices
 *
 *            See @ref ExampleDeviceIcm30630EMD.c example.
 *
 *  @ingroup  Device
 *	@{
 */

#ifndef _INV_DEVICE_ICM30XXX_H_
#define _INV_DEVICE_ICM30XXX_H_

#include "Invn/Devices/Device.h"
#include "Invn/Devices/Drivers/Icm30xxx/Icm30xxx.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Enum describing possible type of image that can be loaded
 */
enum inv_device_icm30xxx_image_type {
	INV_DEVICE_ICM30XXX_IMAGE_TYPE_M0_FLASH,
	INV_DEVICE_ICM30XXX_IMAGE_TYPE_M0_SRAM,
	INV_DEVICE_ICM30XXX_IMAGE_TYPE_DMP3,
	INV_DEVICE_ICM30XXX_IMAGE_TYPE_DMP4,
};

/** @brief States for Icm30xxx device
 */
typedef struct inv_device_icm30xxx
{
	inv_device_t         base;
	struct inv_icm30xxx    icm30xxx_states;
	inv_bool_t           debug_en;
	uint32_t             clock_rate;
	void                 (* irq_callback)(void * context, int int_num);
    void *               irq_callback_context;
} inv_device_icm30xxx_t;

/** @brief constructor-like function for Icm30xxx based device
 *
 *  Will initialize inv_device_icm30xxx_t object states to default value
 *
 *  Should not be called directly. Prefer specialized functions such as
 *  inv_device_icm30630_init() and inv_device_icm30670_init() to construct
 *  the object.
 *
 *  Will initialize inv_device_icm30xxx_t object states to default value.
 *  You must provide a static buffer to the driver. Buffer must be >= 4kB.
 *  Bigger buffer increases performance a bit by reducing number of request to
 *  the device for some actions.
 *
 *  @warning An option is provided to enable target debugging with JTAG/SWD.
 *  Attempting to debug the target whitout enabling this option can dammage it.
 *
 *  @param[in] self         handle to device
 *  @param[in] serif        reference to Host Serial Interface object
 *  @param[in] listener     reference to Sensor Event Listener object
 *  @param[in] buffer       reference to static buffer
 *  @param[in] buffer_size  static buffer size (must be >= 4kB)
 *  @param[in] jtag_en      enable JTAG/SWD debugging capability
 *  @param[in] chip_variant specify target chip from the Icm30xxx family
 */
void inv_device_icm30xxx_init(inv_device_icm30xxx_t * self,
		const inv_host_serif_t * serif, const inv_sensor_listener_t * listener,
		uint8_t * buffer, uint32_t buffer_size, inv_bool_t jtag_en, int chip_variant);


/** @brief constructor-like function for Icm30630 device
 *
 *  See inv_device_icm30xxx_init() for arguments description.
 */
static inline void inv_device_icm30630_init(inv_device_icm30xxx_t * self,
		const inv_host_serif_t * serif, const inv_sensor_listener_t * listener,
		uint8_t * buffer, uint32_t buffer_size, inv_bool_t jtag_en)
{
	inv_device_icm30xxx_init(self, serif, listener, buffer, buffer_size, jtag_en,
			INV_ICM30XXX_VARIANT_ICM30630);
}

/** @brief constructor-like function for Icm30670 device
 *
 *  See inv_device_icm30xxx_init() for arguments description.
 */
static inline void inv_device_icm30670_init(inv_device_icm30xxx_t * self,
		const inv_host_serif_t * serif, const inv_sensor_listener_t * listener,
		uint8_t * buffer, uint32_t buffer_size, inv_bool_t jtag_en)
{
	inv_device_icm30xxx_init(self, serif, listener, buffer, buffer_size, jtag_en,
			INV_ICM30XXX_VARIANT_ICM30670);
}

/** @brief Helper function to get handle to base object
 */
static inline inv_device_t * inv_device_icm30xxx_get_base(inv_device_icm30xxx_t * self)
{
	if(self)
		return &self->base;

	return 0;
}

/** @brief  Helper function to register an interrupt interrupt callback
 *
 *  This method should be call after inv_device_icm30630_init()
 *
 *  Will call register_interrupt_callback() method of underlying Serial
 *  Interface object.
 *  This allows to call the device poll() function upon interrupt
 *  (instead of polling) depending on underlying Serial Interface object
 *  capabilities and implementation.
 *
 *  @warning registered callback will be called in the same context than
 *  Host Serif callbak.
 *  @param[in] self          handle to device icm30xxx object
 *  @param[in] interrupt_cb  callback called upon interrupt (throug host serif object)
 *  @param[in] context       some pointer passed to the callback
 *  @return					 0 on success
 *							 INV_ERROR_NIMPL   if adapter doesn't support IRQ
 *                           INV_ERROR_BAD_ARG if inv_device_icm30xxx_t* is set to NULL
 */
int inv_device_icm30xxx_register_interrupt_callback(inv_device_icm30xxx_t * self,
	   void (*interrupt_cb)(void * context, int int_num), void * context);

/*
 * Functions below are described in Device.h
 */

int inv_device_icm30xxx_whoami(void * context, uint8_t * whoami);

int inv_device_icm30xxx_reset(void * context);

int inv_device_icm30xxx_setup(void * context);

int inv_device_icm30xxx_cleanup(void * context);

int inv_device_icm30xxx_load(void * context, int what,
		const uint8_t * image, uint32_t size, inv_bool_t verify, inv_bool_t force);

int inv_device_icm30xxx_get_fw_info(void * context,
		struct inv_fw_version * version);

int inv_device_icm30xxx_set_running_state(void * context, inv_bool_t state);

int inv_device_icm30xxx_ping_sensor(void * context, int sensor);

int inv_device_icm30xxx_start_sensor(void * context, int sensor);

int inv_device_icm30xxx_stop_sensor(void * context, int sensor);

int inv_device_icm30xxx_set_sensor_period(void * context,
		int sensor, uint32_t period);

int inv_device_icm30xxx_set_sensor_timeout(void * context,
		int sensor, uint32_t timeout);

int inv_device_icm30xxx_flush_sensor(void * context, int sensor);

int inv_device_icm30xxx_set_sensor_bias(void * context, int sensor,
		const float bias[3]);

int inv_device_icm30xxx_get_sensor_bias(void * context, int sensor,
		float bias[3]);

int inv_device_icm30xxx_set_sensor_mounting_matrix(void * context,
		int sensor, const float matrix[9]);

int inv_device_icm30xxx_get_sensor_data(void * context, int sensor,
		inv_sensor_event_t * event);

int inv_device_icm30xxx_poll(void * context);

int inv_device_icm30xxx_self_test(void * context, int sensor);

#ifdef __cplusplus
}
#endif

#endif /* _INV_DEVICE_ICM30XXX_H_ */


/** @} */
