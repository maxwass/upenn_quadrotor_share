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

/** @defgroup Device Device
 *	@brief    Abstract device interface definition
 *
 *            All functions declared in this file are virtual.
 *            They aim to provide a unified way of accessing InvenSense devices.
 *            All functions shall return a int for which 0 indicates success and
 *            a negative value indicates an error as described by enum inv_error
 *
 *            If a particular device implementation does not support any of the
 *            method declared here, it shall return INV_ERROR_NIMPL.
 *
 *            Implementation is not expecteded to be thread-safe.
 *
 *            Refer to concrete device implementation for additionnal and specific
 *            information about API usage related to a particular device.
 *
 *	@{
 */

#ifndef _INV_DEVICE_H_
#define _INV_DEVICE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "Invn/InvBool.h"
#include "Invn/InvError.h"
#include "Invn/Devices/SensorTypes.h"
#include "Invn/Devices/HostSerif.h"

/** @brief FW version structure definition
 */
typedef struct inv_fw_version
{
	uint8_t  major, minor, patch; /**< major, minor, patch version number */
	char     suffix[16];          /**< version suffix string (always terminated by '\0') */
	uint32_t crc;                 /**< FW checksum */
} inv_fw_version_t;

/** @brief Device virtual table definition
 */
typedef struct inv_device_vt {
	int (*whoami)(void * self, uint8_t * whoami);
	int (*reset)(void * self);
	int (*setup)(void * self);
	int (*cleanup)(void * self);
	int (*load)(void * self, int type, const uint8_t * image, uint32_t size,
			inv_bool_t verify, inv_bool_t force);
	int (*poll)(void * self);
	int (*self_test)(void * self, int sensor);
	int (*get_fw_info)(void * self, struct inv_fw_version * version);
	int (*ping_sensor)(void * self, int sensor);
	int (*set_running_state)(void * self, inv_bool_t state);
	int (*start_sensor)(void * self, int sensor);
	int (*stop_sensor)(void * self, int sensor);
	int (*set_sensor_period)(void * self, int sensor, uint32_t period);
	int (*set_sensor_timeout)(void * self, int sensor, uint32_t timeout);
	int (*flush_sensor)(void * self, int sensor);
	int (*set_sensor_bias)(void * self, int sensor, const float bias[3]);
	int (*get_sensor_bias)(void * self, int sensor, float bias[3]);
	int (*set_sensor_mounting_matrix)(void * self, int sensor, const float matrix[9]);
	int (*get_sensor_data)(void * self, int sensor, inv_sensor_event_t * event);
} inv_device_vt_t;

/** @brief Abtract device object definition
 */
typedef struct inv_device
{
	void *                        instance; /**< pointer to object instance */
	const struct inv_device_vt *  vt;       /**< pointer to object virtual table */
	const inv_host_serif_t *      serif;    /**< pointer serial interface instance */
	const inv_sensor_listener_t * listener; /**< pointer to listener instance */
} inv_device_t;

/** @brief Gets WHO AM I value
 *
 *  Can be called before performing device setup
 *
 *  @param[in]  dev     pointer to device object instance
 *  @param[out] whoami  WHO AM I value
 *  @return             0 on sucess
 *                      INV_ERROR_TRANSPORT in case of low level serial error
 */
static inline int inv_device_whoami(const inv_device_t * dev, uint8_t * whoami)
{
	assert(dev && dev->vt);

	if(dev->vt->whoami)
		return dev->vt->whoami(dev->instance, whoami);

	return INV_ERROR_NIMPL;
}

/** @brief Resets the device to a known state
 *
 *  Will perform an HW and SW reset of device, and reset internal driver states
 *  To know value.
 *  Should be called before setup or when device state is unknown.
 *
 *  @param[in] dev      pointer to device object instance
 *  @return             0 on sucess
 *                      INV_ERROR_TIMEOUT if reset does not complete in time
 *                      INV_ERROR_TRANSPORT in case of low level serial error
 */
static inline int inv_device_reset(const inv_device_t * dev)
{
	assert(dev && dev->vt);

	if(dev->vt->reset)
		return dev->vt->reset(dev->instance);

	return INV_ERROR_NIMPL;
}

/** @brief Performs basic device initialization
 *
 *  Except if device's flash memory is outdated, devuie should be able to handle
 *  request after setup() is complete.
 *  If devices's flash memory need to be updated, load_begin()/load_continue()/
 *  load_end() methods must be called first with suitable argument.
 *  
 *  @param[in] dev      pointer to device object instance
 *  @return             0 on sucess
 *                      INV_ERROR_TIMEOUT if setup does not complete in time
 *                      INV_ERROR_TRANSPORT in case of low level serial error
 */
static inline int inv_device_setup(const inv_device_t * dev)
{
	assert(dev && dev->vt);

	if(dev->vt->setup)
		return dev->vt->setup(dev->instance);

	return INV_ERROR_NIMPL;
}

/** @brief Shutdowns the device and clean-up internal states
 *
 *  @param[in] dev      pointer to device object instance
 *  @return             0 on sucess
 *                      INV_ERROR_TIMEOUT if clean-up does not complete in time
 *                      INV_ERROR_TRANSPORT in case of low level serial error
 */
static inline int inv_device_cleanup(const inv_device_t * dev)
{
	assert(dev && dev->vt);

	if(dev->vt->cleanup)
		return dev->vt->cleanup(dev->instance);

	return INV_ERROR_NIMPL;
}

/** @brief Polls the device for data
 *
 *  Will read device interrupt registers and data registers or FIFOs.
 *  Will parse data and called sensor events handler provided at init time. 
 *  Handler will be called in the same context of this function.
 * 
 *  @warning Care should be taken regarding concurency. If this function is
 *           called in a dedicated thread, suitable protection must be used to
 *           avoid concurent calls to poll() or any other device methods.
 *
 *  @param[in] dev      pointer to device object instance
 *  @return             0 on sucess
 *                      INV_ERROR_TRANSPORT in case of low level serial error
 *                      INV_ERROR_UNEXPECTED in case of bad formated or 
 *                                           un-handled data frame
 */
static inline int inv_device_poll(const inv_device_t * dev)
{
	assert(dev && dev->vt);

	if(dev->vt->poll)
		return dev->vt->poll(dev->instance);

	return INV_ERROR_NIMPL;
}

/** @brief Begins loading procedure for device's image(s)
 *
 *  Will start the process of loading an image to device's memory.
 *  Type of images to load will depend on the device type and/or FW version.
 *
 *  @param[in] dev      pointer to device object instance
 *  @param[in] type     type of image to load. Can vary from one implementation
 *                      to another. Refer to specific implementation for details.
 *  @param[in] image    pointer to image (or image chunk) data
 *  @param[in] size     size of image (or size of image chunk)
 *  @return             0 on sucess
 *                      INV_ERROR_TRANSPORT in case of low level serial error
 *                      INV_ERROR_TIMEOUT if device does not respond in time
 *                      INV_ERROR_SIZE if image size does not fit in device memory
 */
static inline int inv_device_load(const inv_device_t * dev, int type,
		const uint8_t * image, uint32_t size, inv_bool_t verify, inv_bool_t force)
{
	assert(dev && dev->vt);

	if(dev->vt->load)
		return dev->vt->load(dev->instance, type, image, size, verify, force);

	return INV_ERROR_NIMPL;
}

/** @brief Gets device FW version
 *
 *  @param[in]  dev     pointer to device object instance
 *  @param[out] version version structure placeholder
 *  @return             0 on sucess
 *                      INV_ERROR_TRANSPORT in case of low level serial error
 *                      INV_ERROR_TIMEOUT if device does not respond in time
 */
static inline int inv_device_get_fw_info(const inv_device_t * dev,
		struct inv_fw_version * version)
{
	assert(dev && dev->vt);

	if(dev->vt->get_fw_info)
		return dev->vt->get_fw_info(dev->instance, version);

	return INV_ERROR_NIMPL;
}

/** @brief Indicates to device current RUN/SUSPEND state of the host
 *
 *  If SUSPEND state (false) is set, device should not notify any sensor events
 *  (besides event comming from a wake-up source). If RUNNING state (true) is
 *  set, all sensor events will be notify to host.
 *  Device will consider host to be in RUNNING state after a reset/setup.
 * 
 *  @param[in] dev      pointer to device object instance
 *  @param[in] state    RUNNING (true) or SUSPEND (false) state
 *  @return             0 on sucess
 *                      INV_ERROR_TRANSPORT in case of low level serial error
 */
static inline int inv_device_set_running_state(const inv_device_t * dev, inv_bool_t state)
{
	if(dev->vt->set_running_state)
		return dev->vt->set_running_state(dev->instance, state);

	return INV_ERROR_NIMPL;
}

/** @brief Checks if a sensor is supported by the device
 *
 *  @param[in] dev      pointer to device object instance
 *  @param[in] sensor   sensor type (as defined by @sa inv_sensor_type_t)
 *  @return             0 on sucess
 *                      INV_ERROR_TRANSPORT in case of low level serial error
 *                      INV_ERROR_TIMEOUT if device does not respond in time
 *                      INV_ERROR if sensor is not supported by the device
 *                      INV_ERROR_BAD_ARG if sensor is no unsupported by the implementation
 */
static inline int inv_device_ping_sensor(const inv_device_t * dev, int sensor)
{
	assert(dev && dev->vt);
	
	if(dev->vt->ping_sensor)
		return dev->vt->ping_sensor(dev->instance, sensor);

	return INV_ERROR_NIMPL;
}

/** @brief Starts a sensor
 * 
 *  Send a command to start a sensor. Device will start sending events if sensor
 *  is supported (ie: ping() returns 0 for this sensor type).
 *
 *  @param[in] dev      pointer to device object instance
 *  @param[in] sensor   sensor type (as defined by @sa inv_sensor_type_t)
 *  @return             0 on sucess
 *                      INV_ERROR_TRANSPORT in case of low level serial error
 *                      INV_ERROR_BAD_ARG if sensor is no unsupported by the implementation
 */
static inline int inv_device_start_sensor(const inv_device_t * dev, int sensor)
{
	assert(dev && dev->vt);

	if(dev->vt->start_sensor)
		return dev->vt->start_sensor(dev->instance, sensor);

	return INV_ERROR_NIMPL;
}

/** @brief Stops a sensor
 * 
 *  Send a command to stop a sensor. Device will stop sending events if sensor
 *  was previously started.
 *
 *  @param[in] dev      pointer to device object instance
 *  @param[in] sensor   sensor type (as defined by @sa inv_sensor_type_t)
 *  @return             0 on sucess
 *                      INV_ERROR_TRANSPORT in case of low level serial error
 *                      INV_ERROR_BAD_ARG if sensor is no unsupported by the implementation
 */
static inline int inv_device_stop_sensor(const inv_device_t * dev, int sensor)
{
	assert(dev && dev->vt);

	if(dev->vt->stop_sensor)
		return dev->vt->stop_sensor(dev->instance, sensor);

	return INV_ERROR_NIMPL;
}

/** @brief Configure sensor output data period.
 * 
 *  Send a command to set sensor output data period. Period is a hint only.
 *  Depending on sensor type or device capability, the effective output data
 *  might be different. User shall refer to sensor events timestamp to determine
 *  effective output data period.
 *
 *  @param[in] dev      pointer to device object instance
 *  @param[in] sensor   sensor type (as defined by @sa inv_sensor_type_t)
 *  @param[in] period   requested data period in ms
 *  @return             0 on sucess
 *                      INV_ERROR_TRANSPORT in case of low level serial error
 *                      INV_ERROR_BAD_ARG if sensor is no unsupported by the implementation
 */
static inline int inv_device_set_sensor_period(const inv_device_t * dev,
		int sensor, uint32_t period)
{
	assert(dev && dev->vt);

	if(dev->vt->set_sensor_period)
		return dev->vt->set_sensor_period(dev->instance, sensor, period);

	return INV_ERROR_NIMPL;
}

/** @brief Configure sensor output timeout.
 * 
 *  Send a command to set sensor maximum report latency (or batch timeout).
 *  This allows to enable batch mode. Provided timeout is a hint only and sensor
 *  events may be notified at a faster rate depending on sensor type or device
 *  capability or other active sensors.
 *
 *  @param[in] dev      pointer to device object instance
 *  @param[in] sensor   sensor type (as defined by @sa inv_sensor_type_t)
 *  @param[in] timeout  allowed timeout in ms
 *  @return             0 on sucess
 *                      INV_ERROR_TRANSPORT in case of low level serial error
 *                      INV_ERROR_BAD_ARG if sensor is no unsupported by the implementation
 */
static inline int inv_device_set_sensor_timeout(const inv_device_t * dev,
		int sensor, uint32_t timeout)
{
	assert(dev && dev->vt);

	if(dev->vt->set_sensor_timeout)
		return dev->vt->set_sensor_timeout(dev->instance, sensor, timeout);

	return INV_ERROR_NIMPL;
}

/** @brief Forces flush of devices's internal buffers
 * 
 *  Send a command a flush command to device. 
 *  Device will imediatly send all sensor events that may be store in its
 *  internal buffers.
 *
 *  @param[in] dev      pointer to device object instance
 *  @param[in] sensor   sensor type (as defined by @sa inv_sensor_type_t)
 *  @return             0 on sucess
 *                      INV_ERROR_TRANSPORT in case of low level serial error
 *                      INV_ERROR_BAD_ARG if sensor is no unsupported by the implementation
 */
static inline int inv_device_flush_sensor(const inv_device_t * dev,
		int sensor)
{
	assert(dev && dev->vt);

	if(dev->vt->flush_sensor)
		return dev->vt->flush_sensor(dev->instance, sensor);

	return INV_ERROR_NIMPL;
}

/** @brief Configure bias value for a sensor
 * 
 *  Bias configuration makes sense only for few sensor types: 
 *   - INV_SENSOR_TYPE_ACCELEROMETER
 *   - INV_SENSOR_TYPE_MAGNETOMETER
 *   - INV_SENSOR_TYPE_GYROMETER
 *  Bias unit is the same as the corresponding sensor unit.
 *  @sa inv_sensor_event_t for details.
 *
 *  If this feature is supported by the implementation but not by the device,
 *  behavior is undefined (but will most probably have no effect).
 *
 *  @param[in] dev      pointer to device object instance
 *  @param[in] sensor   sensor type (as defined by @sa inv_sensor_type_t)
 *  @param[in] bias     bias to set
 *  @return             0 on sucess
 *                      INV_ERROR_TRANSPORT in case of low level serial error
 *                      INV_ERROR_BAD_ARG if sensor is no unsupported by the implementation
 */
static inline int inv_device_set_sensor_bias(const inv_device_t * dev,
		int sensor, const float bias[3])
{
	assert(dev && dev->vt);

	if(dev->vt->set_sensor_bias)
		return dev->vt->set_sensor_bias(dev->instance, sensor, bias);

	return INV_ERROR_NIMPL;
}

/** @brief Gets bias value for a sensor
 * 
 *  Bias configuration makes sense only for few sensor types: 
 *   - INV_SENSOR_TYPE_ACCELEROMETER
 *   - INV_SENSOR_TYPE_MAGNETOMETER
 *   - INV_SENSOR_TYPE_GYROMETER
 *  Bias unit is the same as the corresponding sensor unit.
 *  @sa inv_sensor_event_t for details.
 *
 *  @param[in]  dev     pointer to device object instance
 *  @param[in]  sensor  sensor type (as defined by @sa inv_sensor_type_t)
 *  @param[out] bias    returned bias
 *  @return             0 on sucess
 *                      INV_ERROR_TRANSPORT in case of low level serial error
 *                      INV_ERROR_TIMEOUT if device does not respond in time
 *                      INV_ERROR_BAD_ARG if sensor is no unsupported by the implementation
 */
static inline int inv_device_get_sensor_bias(const inv_device_t * dev,
		int sensor, float bias[3])
{
	assert(dev && dev->vt);

	if(dev->vt->get_sensor_bias)
		return dev->vt->get_sensor_bias(dev->instance, sensor, bias);

	return INV_ERROR_NIMPL;
}

/** @brief Sets the mounting matrix information for a multi-axis sensor
 * 
 *  Allow to specify the mounting matrix for multi-axis sensor in order to
 *  align axis of several sensors in the same reference frame.
 *  Sensor types allowed:
 *   - INV_SENSOR_TYPE_ACCELEROMETER
 *   - INV_SENSOR_TYPE_MAGNETOMETER
 *   - INV_SENSOR_TYPE_GYROMETER
 *  Depending on device capability, called to this function may have no effect.
 *
 *  @param[in]  dev     pointer to device object instance
 *  @param[in]  sensor  sensor type (as defined by @sa inv_sensor_type_t)
 *  @param[in]  matrix  mounting matrix to apply
 *  @return             0 on sucess
 *                      INV_ERROR_TRANSPORT in case of low level serial error
 *                      INV_ERROR_TIMEOUT if device does not respond in time
 *                      INV_ERROR_BAD_ARG if sensor is no unsupported by the implementation
 */
static inline int inv_device_set_sensor_mounting_matrix(const inv_device_t * dev,
		int sensor, const float matrix[9])
{
	assert(dev && dev->vt);

	if(dev->vt->set_sensor_mounting_matrix)
		return dev->vt->set_sensor_mounting_matrix(dev->instance, sensor, matrix);

	return INV_ERROR_NIMPL;
}

/** @brief Retrieve last known sensor event for a sensor
 * 
 *  Depending on device capability, a call to this function may have no effect or
 *  return an error.
 *
 *  @param[in]  dev     pointer to device object instance
 *  @param[in]  sensor  sensor type (as defined by @sa inv_sensor_type_t)
 *  @param[out] event   last known event data
 *  @return             0 on sucess
 *                      INV_ERROR_TRANSPORT in case of low level serial error
 *                      INV_ERROR_TIMEOUT if device does not respond in time
 *                      INV_ERROR_BAD_ARG if sensor is no unsupported by the implementation
 *                      INV_ERROR if and event was received but unmanaged by the implementation
 */
static inline int inv_device_get_sensor_data(const inv_device_t * dev, int sensor,
		inv_sensor_event_t * event)
{
	assert(dev && dev->vt);

	if(dev->vt->get_sensor_data)
		return dev->vt->get_sensor_data(dev->instance, sensor, event);

	return INV_ERROR_NIMPL;
}

/** @brief Perform self-test procedure for MEMS component of the device
 * 
 *  Available MEMS vary depend on the device.
 *  Use following sensor type for the various MEMS:
 *   - INV_SENSOR_TYPE_ACCELEROMETER: for HW accelerometer sensor
 *   - INV_SENSOR_TYPE_MAGNETOMETER : for HW magnetometer sensor
 *   - INV_SENSOR_TYPE_GYROMETER    : for HW gyrometer sensor
 *   - INV_SENSOR_TYPE_PRESSURE     : for HW pressure sensor
 *
 *  @param[in]  dev     pointer to device object instance
 *  @param[in]  sensor  sensor type (as defined by @sa inv_sensor_type_t)
 *  @return             0 on sucess
 *                      INV_ERROR_TRANSPORT in case of low level serial error
 *                      INV_ERROR_TIMEOUT if device does not respond in time
 *                      INV_ERROR if self test has failed
 */
static inline int inv_device_self_test(const inv_device_t * dev, int sensor)
{
	assert(dev && dev->vt);

	if(dev->vt->self_test)
		return dev->vt->self_test(dev->instance, sensor);

	return INV_ERROR_NIMPL;
}

#ifdef __cplusplus
}
#endif

#endif /* _INV_DEVICE_H_ */

/** @} */
