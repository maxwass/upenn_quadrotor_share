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

#include "DeviceIcm30xxx.h"

#include "Invn/Utils/Message.h"

#include "Invn/FifoProtocol/FifoProtocolDefs.h"
#include "Invn/FifoProtocol/FifoProtocolSensorId.h"
#include "Invn/FifoProtocol/FifoProtocolHelper.h"

static const inv_device_vt_t device_icm30xxx_vt = {
	inv_device_icm30xxx_whoami,
	inv_device_icm30xxx_reset,
	inv_device_icm30xxx_setup,
	inv_device_icm30xxx_cleanup,
	inv_device_icm30xxx_load,
	inv_device_icm30xxx_poll,
	inv_device_icm30xxx_self_test,
	inv_device_icm30xxx_get_fw_info,
	inv_device_icm30xxx_ping_sensor,
	inv_device_icm30xxx_set_running_state,
	inv_device_icm30xxx_start_sensor,
	inv_device_icm30xxx_stop_sensor,
	inv_device_icm30xxx_set_sensor_period,
	inv_device_icm30xxx_set_sensor_timeout,
	inv_device_icm30xxx_flush_sensor,
	inv_device_icm30xxx_set_sensor_bias,
	inv_device_icm30xxx_get_sensor_bias,
	inv_device_icm30xxx_set_sensor_mounting_matrix,
	inv_device_icm30xxx_get_sensor_data,
};

static int sensor_type_2_fifo_protocol_id(int type);
static int fifo_protocol_id_2_sensor_type(uint8_t sensorid);
static inv_bool_t build_sensor_event(void * context, uint8_t sensorid, uint32_t timestamp,
		uint8_t status, uint8_t accuracy, const uint8_t * data, uint16_t len,
		inv_sensor_event_t * event);
static int serif_read_reg(void * context, uint8_t reg,
		uint8_t * buf, uint32_t len);
static int serif_write_reg(void * context, uint8_t reg,
		const uint8_t * buf, uint32_t len);
static void data_handler_wrapper(void * context, uint8_t sensorid,
		uint32_t timestamp, uint8_t status, uint8_t accuracy,
		const uint8_t * data, uint16_t size);

/*
 * Using a wrapper will allow to filter event later
 * WARNING : This function is called under the same context than the host serif
 * interrupt callback (possibly IRQ)
 */
static void interrupt_cb_wrapper(void * context, int int_num)
{
  	inv_device_icm30xxx_t * self = (inv_device_icm30xxx_t *) context;

	if(self->irq_callback)
		self->irq_callback(context, int_num);
}

int inv_device_icm30xxx_register_interrupt_callback(inv_device_icm30xxx_t * self,
	   void (*interrupt_cb)(void * context, int int_num), void * context)
{
	int rc = INV_ERROR_BAD_ARG;

	if(self)
	{
		self->irq_callback         = interrupt_cb;
		self->irq_callback_context = context;

		rc = inv_host_serif_register_interrupt_callback(self->base.serif, interrupt_cb_wrapper, self);
	}

	return(rc);
}

void inv_device_icm30xxx_init(inv_device_icm30xxx_t * self,
		const inv_host_serif_t * serif, const inv_sensor_listener_t * listener,
		uint8_t * buffer, uint32_t buffer_size, inv_bool_t debug, int chip_variant)
{
	struct inv_icm30xxx_serif icm30xxx_serif;

	assert(self && listener && serif && buffer);

	/* build base */
	self->base.instance = self;
	self->base.vt       = &device_icm30xxx_vt;
	self->base.serif    = serif;
	self->base.listener = listener;

	/* initialize icm30xxx serif structure */
	icm30xxx_serif.context   = self;
	icm30xxx_serif.read_reg  = serif_read_reg;
	icm30xxx_serif.write_reg = serif_write_reg;
	icm30xxx_serif.max_read  = inv_host_serif_get_max_transaction_size(serif);
	icm30xxx_serif.max_write = inv_host_serif_get_max_transaction_size(serif);
	icm30xxx_serif.is_spi    = inv_host_serif_is_spi(serif);

	/* reset icm30xxx driver states */
	inv_icm30xxx_reset_states(&self->icm30xxx_states, chip_variant,
		&icm30xxx_serif, buffer, buffer_size, data_handler_wrapper, self);

	self->clock_rate = 1;
	self->debug_en   = debug;

	/* init cb and context */
	self->irq_callback         = 0;
	self->irq_callback_context = 0;
}

int inv_device_icm30xxx_poll(void * context)
{
	inv_device_icm30xxx_t * self = (inv_device_icm30xxx_t *)context;

	return inv_icm30xxx_ctrl_poll(&self->icm30xxx_states);
}

int inv_device_icm30xxx_whoami(void * context, uint8_t * whoami)
{
	inv_device_icm30xxx_t * self = (inv_device_icm30xxx_t *)context;

	assert(whoami);

	return inv_icm30xxx_get_whoami(&self->icm30xxx_states, whoami);
}

int inv_device_icm30xxx_reset(void * context)
{
	inv_device_icm30xxx_t * self = (inv_device_icm30xxx_t *)context;
	int rc;

	INV_MSG(INV_MSG_LEVEL_VERBOSE, "Reseting device...");

	if((rc = inv_icm30xxx_soft_reset(&self->icm30xxx_states)) != 0) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "Icm30xxx soft reset returned %d", rc);
		return rc;
	}

	return 0;
}

int inv_device_icm30xxx_setup(void * context)
{
	inv_device_icm30xxx_t * self = (inv_device_icm30xxx_t *)context;
	int rc;
	uint8_t whoami, ping_value;

	INV_MSG(INV_MSG_LEVEL_INFO, "Booting up Icm30xxx...");

	/* reset if not already done */
	if((rc = inv_device_icm30xxx_reset(self)) != 0) {
		return rc;
	}

	/* Check WHOAMI */
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "Reading WHOAMI...");
	if((rc = inv_device_icm30xxx_whoami(self, &whoami)) != 0) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "Error %d when reading WHOAMI value", rc);
		return rc;
	}

	if(whoami == 0 || whoami == 0xff) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "Unexpected WHOAMI value 0x%x. Aborting setup.", whoami);
		return INV_ERROR;
	} else {
		INV_MSG(INV_MSG_LEVEL_INFO, "WHOAMI value: 0x%x", whoami);
	}

	/* set default power mode */
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "Putting Icm30xxx in sleep mode...");
	if((rc = inv_icm30xxx_set_sleep(&self->icm30xxx_states)) != 0)
		goto error;

	/* start M0 */
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "Starting M0 (debug=%s)", (self->debug_en) ? "on" : "off");
	if((rc = inv_icm30xxx_start_m0(&self->icm30xxx_states, self->debug_en)) != 0)
		goto error;

	/* caching FIFOs config */
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "Retrieving FIFOs configuration...");
	if((rc = inv_icm30xxx_ctrl_retrieve_fifo_config(&self->icm30xxx_states)) != 0)
		goto error;

	/* ping system to see if it responding */
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "Ping system to see if it is responding...");
	if((rc = inv_icm30xxx_cmd_ping_sensor(&self->icm30xxx_states, 0, &ping_value)) != 0)
		goto error;

	if(!ping_value) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "System not responding. Something went wrong.");
		rc = INV_ERROR;
		goto error;
	} else {
		INV_MSG(INV_MSG_LEVEL_INFO, "System responded OK to ping");
	}

	/* retrieve clock rate to convert received data timestamp to standard unit */
	// FIXME Will fail if we do this before loading DMP image because of FW bug
	INV_MSG(INV_MSG_LEVEL_INFO, "Getting clock rate...");
	if((rc = inv_icm30xxx_cmd_get_clock_rate(&self->icm30xxx_states, &self->clock_rate)) != 0)
		goto error;

	INV_MSG(INV_MSG_LEVEL_VERBOSE, "System clock rate is %lu ns/tick", self->clock_rate);

	// FIXME need to set proper power mode

	/* we should be good to go ! */
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "We're good to go !");

	return 0;
error:
	INV_MSG(INV_MSG_LEVEL_ERROR, "Error %d while setting-up device.", rc);

	return rc;
}

int inv_device_icm30xxx_cleanup(void * context)
{
	inv_device_icm30xxx_t * self = (inv_device_icm30xxx_t *)context;

	return inv_device_icm30xxx_reset(self);
}

int inv_device_icm30xxx_load(void * context, int what,
		const uint8_t * image, uint32_t size, inv_bool_t verify, inv_bool_t force)
{
	inv_device_icm30xxx_t * self = (inv_device_icm30xxx_t *)context;

	switch(what) {
		case INV_DEVICE_ICM30XXX_IMAGE_TYPE_M0_FLASH:
			INV_MSG(INV_MSG_LEVEL_INFO,
					"Attempting to load %u bytes into FLASH...", size);
			return inv_icm30xxx_load_flash(&self->icm30xxx_states, image, size, verify, force);

		case INV_DEVICE_ICM30XXX_IMAGE_TYPE_M0_SRAM:
			INV_MSG(INV_MSG_LEVEL_INFO,
					"Attempting to load %u bytes into SRAM...", size);
			return INV_ERROR_NIMPL;

		case INV_DEVICE_ICM30XXX_IMAGE_TYPE_DMP3:
			INV_MSG(INV_MSG_LEVEL_INFO,
					"Attempting to load %u bytes into DMP3...", size);
			return inv_icm30xxx_load_other(&self->icm30xxx_states,
					FIFOPROTOCOL_LOAD_WHO_DMP3_FW, image, size);

		case INV_DEVICE_ICM30XXX_IMAGE_TYPE_DMP4:
			INV_MSG(INV_MSG_LEVEL_INFO,
					"Attempting to load %u bytes into DMP4...", size);
			return inv_icm30xxx_load_other(&self->icm30xxx_states,
					FIFOPROTOCOL_LOAD_WHO_DMP4_FW, image, size);

		default:
			return INV_ERROR_BAD_ARG;
	}
}

int inv_device_icm30xxx_get_fw_info(void * context,
		struct inv_fw_version * version)
{
	inv_device_icm30xxx_t * self = (inv_device_icm30xxx_t *)context;
	int rc;
	struct inv_icm30xxx_fw_version icm30xxx_version;

	assert(version);

	if((rc = inv_icm30xxx_cmd_get_fw_info(&self->icm30xxx_states,
			&icm30xxx_version)) != 0)
		return rc;

	version->major = icm30xxx_version.major;
	version->minor = icm30xxx_version.minor;
	version->patch = icm30xxx_version.patch;
	version->crc   = icm30xxx_version.crc;
	memset(version->suffix, '\0', sizeof(version->suffix));
	strncpy(version->suffix, icm30xxx_version.hash, sizeof(icm30xxx_version.hash));

	return 0;
}

int inv_device_icm30xxx_set_running_state(void * context, inv_bool_t state)
{
	inv_device_icm30xxx_t * self = (inv_device_icm30xxx_t *)context;

	return inv_icm30xxx_cmd_set_running_state(&self->icm30xxx_states, state);
}

int inv_device_icm30xxx_ping_sensor(void * context, int sensor)
{
	inv_device_icm30xxx_t * self = (inv_device_icm30xxx_t *)context;
	int rc;
	const int id = sensor_type_2_fifo_protocol_id(sensor);
	const uint8_t sid = (uint8_t)id;
	uint8_t ping_value;

	if(id == -1)
		return INV_ERROR_BAD_ARG;

	if((rc = inv_icm30xxx_cmd_ping_sensor(&self->icm30xxx_states, sid,
			&ping_value)) != 0)
		return rc;

	if(ping_value == 0)
		return INV_ERROR;

	return 0;
}

int inv_device_icm30xxx_start_sensor(void * context, int sensor)
{
	inv_device_icm30xxx_t * self = (inv_device_icm30xxx_t *)context;
	const int id = sensor_type_2_fifo_protocol_id(sensor);
	const uint8_t sid = (uint8_t)id;

	if(id == -1)
		return INV_ERROR_BAD_ARG;

	return inv_icm30xxx_cmd_enable_sensor(&self->icm30xxx_states, sid, true);
}

int inv_device_icm30xxx_stop_sensor(void * context, int sensor)
{
	inv_device_icm30xxx_t * self = (inv_device_icm30xxx_t *)context;
	const int id = sensor_type_2_fifo_protocol_id(sensor);
	const uint8_t sid = (uint8_t)id;

	if(id == -1)
		return INV_ERROR_BAD_ARG;

	return inv_icm30xxx_cmd_enable_sensor(&self->icm30xxx_states, sid, false);
}

int inv_device_icm30xxx_set_sensor_period(void * context,
		int sensor, uint32_t period)
{
	inv_device_icm30xxx_t * self = (inv_device_icm30xxx_t *)context;
	const int id = sensor_type_2_fifo_protocol_id(sensor);
	const uint8_t sid = (uint8_t)id;

	if(id == -1)
		return INV_ERROR_BAD_ARG;

	return inv_icm30xxx_cmd_set_sensor_period(&self->icm30xxx_states, sid, period);
}

int inv_device_icm30xxx_set_sensor_timeout(void * context,
		int sensor, uint32_t timeout)
{
	inv_device_icm30xxx_t * self = (inv_device_icm30xxx_t *)context;
	const int id = sensor_type_2_fifo_protocol_id(sensor);
	const uint8_t sid = (uint8_t)id;

	if(id == -1)
		return INV_ERROR_BAD_ARG;

	return inv_icm30xxx_cmd_set_sensor_timeout(&self->icm30xxx_states, sid, timeout);
}

int inv_device_icm30xxx_flush_sensor(void * context, int sensor)
{
	inv_device_icm30xxx_t * self = (inv_device_icm30xxx_t *)context;
	const int id = sensor_type_2_fifo_protocol_id(sensor);
	const uint8_t sid = (uint8_t)id;

	if(id == -1)
		return INV_ERROR_BAD_ARG;

	return inv_icm30xxx_cmd_flush_sensor(&self->icm30xxx_states, sid);
}

int inv_device_icm30xxx_set_sensor_bias(void * context, int sensor,
		const float bias[3])
{
	inv_device_icm30xxx_t * self = (inv_device_icm30xxx_t *)context;
	const int id = sensor_type_2_fifo_protocol_id(sensor);
	const uint8_t sid = (uint8_t)id;

	if(id == -1)
		return INV_ERROR_BAD_ARG;

	return inv_icm30xxx_cmd_set_sensor_bias(&self->icm30xxx_states, sid, bias);
}

int inv_device_icm30xxx_get_sensor_bias(void * context, int sensor,
		float bias[3])
{
	inv_device_icm30xxx_t * self = (inv_device_icm30xxx_t *)context;
	const int id = sensor_type_2_fifo_protocol_id(sensor);
	const uint8_t sid = (uint8_t)id;

	if(id == -1)
		return INV_ERROR_BAD_ARG;

	return inv_icm30xxx_cmd_get_sensor_bias(&self->icm30xxx_states, sid, bias);
}

int inv_device_icm30xxx_set_sensor_mounting_matrix(void * context,
		int sensor, const float matrix[9])
{
	inv_device_icm30xxx_t * self = (inv_device_icm30xxx_t *)context;
	const int id = sensor_type_2_fifo_protocol_id(sensor);
	const uint8_t sid = (uint8_t)id;

	if(id == -1)
		return INV_ERROR_BAD_ARG;

	return inv_icm30xxx_cmd_set_sensor_mounting_matrix(&self->icm30xxx_states, sid,
			matrix);
}

int inv_device_icm30xxx_get_sensor_data(void * context, int sensor,
		inv_sensor_event_t * event)
{
	inv_device_icm30xxx_t * self = (inv_device_icm30xxx_t *)context;
	int rc;
	const int id = sensor_type_2_fifo_protocol_id(sensor);
	const uint8_t sid = (uint8_t)id;
	uint8_t data[64];
	uint16_t size;
	uint32_t timestamp;

	if(id == -1)
		return INV_ERROR_BAD_ARG;

	if((rc = inv_icm30xxx_cmd_get_last_sensor_data(&self->icm30xxx_states, sid,
			&timestamp, data, &size)) != 0)
		return rc;

	if(!build_sensor_event(context, sid, timestamp, FIFOPROTOCOL_STATUS_POLL, 0,
			data, size, event))
		return INV_ERROR;

	return 0;
}

int inv_device_icm30xxx_self_test(void * context, int sensor)
{
	(void)context, (void)sensor;

	return INV_ERROR_NIMPL;
}


/******************************************************************************/

static int serif_read_reg(void * context, uint8_t reg,
		uint8_t * buf, uint32_t len)
{
	inv_device_icm30xxx_t * self = (inv_device_icm30xxx_t *)context;

	return inv_host_serif_read_reg(self->base.serif, reg, buf, len);
}

static int serif_write_reg(void * context, uint8_t reg,
		const uint8_t * buf, uint32_t len)
{
	inv_device_icm30xxx_t * self = (inv_device_icm30xxx_t *)context;

	return inv_host_serif_write_reg(self->base.serif, reg, buf, len);
}

static void data_handler_wrapper(void * context, uint8_t sensorid,
		uint32_t timestamp, uint8_t status, uint8_t accuracy,
		const uint8_t * data, uint16_t size)
{
	inv_device_icm30xxx_t * self = (inv_device_icm30xxx_t *)context;
	inv_sensor_event_t event;

	if(build_sensor_event(context, sensorid, timestamp, status, accuracy,
			data, size, &event)) {
		inv_sensor_listener_notify(self->base.listener, &event);
	}
}

static int sensor_type_2_fifo_protocol_id(int type)
{
	int sensorid;
	static int map[INV_SENSOR_TYPE_MAX] = {
		/* TYPE_META_DATA              */ FIFOPROTOCOL_SENSOR_ID_META_DATA,
		/* TYPE_ACCELEROMETER          */ FIFOPROTOCOL_SENSOR_ID_ACCELEROMETER,
		/* TYPE_MAGNETOMETER           */ FIFOPROTOCOL_SENSOR_ID_MAGNETOMETER,
		/* TYPE_ORIENTATION            */ FIFOPROTOCOL_SENSOR_ID_ORIENTATION,
		/* TYPE_GYROMETER              */ FIFOPROTOCOL_SENSOR_ID_GYROMETER,
		/* TYPE_LIGHT                  */ FIFOPROTOCOL_SENSOR_ID_LIGHT,
		/* TYPE_PRESSURE               */ FIFOPROTOCOL_SENSOR_ID_PRESSURE,
		/* TYPE_TEMPERATURE            */ FIFOPROTOCOL_SENSOR_ID_TEMPERATURE,
		/* TYPE_PROXIMITY              */ FIFOPROTOCOL_SENSOR_ID_PROXIMITY,
		/* TYPE_GRAVITY                */ FIFOPROTOCOL_SENSOR_ID_GRAVITY,
		/* TYPE_LINEAR_ACCELERATION    */ FIFOPROTOCOL_SENSOR_ID_LINEAR_ACCELERATION,
		/* TYPE_ROTATION_VECTOR        */ FIFOPROTOCOL_SENSOR_ID_ROTATION_VECTOR,
		/* TYPE_HUMIDITY               */ FIFOPROTOCOL_SENSOR_ID_HUMIDITY,
		/* TYPE_AMBIENT_TEMPERATURE    */ FIFOPROTOCOL_SENSOR_ID_AMBIENT_TEMPERATURE,
		/* TYPE_UNCAL_MAGNETOMETER     */ FIFOPROTOCOL_SENSOR_ID_UNCAL_MAGNETOMETER,
		/* TYPE_GAME_ROTATION_VECTOR   */ FIFOPROTOCOL_SENSOR_ID_GAME_ROTATION_VECTOR,
		/* TYPE_UNCAL_GYROMETER        */ FIFOPROTOCOL_SENSOR_ID_UNCAL_GYROMETER,
		/* TYPE_SMD                    */ FIFOPROTOCOL_SENSOR_ID_SMD,
		/* TYPE_STEP_DETECTOR          */ FIFOPROTOCOL_SENSOR_ID_STEP_DETECTOR,
		/* TYPE_STEP_COUNTER           */ FIFOPROTOCOL_SENSOR_ID_STEP_COUNTER,
		/* TYPE_GEOMAG_ROTATION_VECTOR */ FIFOPROTOCOL_SENSOR_ID_GEOMAG_ROTATION_VECTOR,
		/* TYPE_HEART_RATE             */ FIFOPROTOCOL_SENSOR_ID_HEART_RATE,
		/* TYPE_TILT_DETECTOR          */ FIFOPROTOCOL_SENSOR_ID_TILT_DETECTOR,
		/* TYPE_WAKE_GESTURE           */ FIFOPROTOCOL_SENSOR_ID_WAKE_GESTURE,
		/* TYPE_GLANCE_GESTURE         */ FIFOPROTOCOL_SENSOR_ID_GLANCE_GESTURE,
		/* TYPE_PICK_UP_GESTURE        */ FIFOPROTOCOL_SENSOR_ID_PICK_UP_GESTURE,
		/* TYPE_BAC                    */ FIFOPROTOCOL_SENSOR_ID_ACTIVITY_CLASSIFIER,
		/* TYPE_PDR                    */ -1, // FIFOPROTOCOL_SENSOR_ID_EXT_0,
		/* TYPE_B2S                    */ -1, // FIFOPROTOCOL_SENSOR_ID_EXT_7,
		/* TYPE_3AXIS                  */ -1,
		/* TYPE_CUSTOM0                */ FIFOPROTOCOL_SENSOR_ID_EXT_0,
		/* TYPE_CUSTOM1                */ FIFOPROTOCOL_SENSOR_ID_EXT_1,
		/* TYPE_CUSTOM2                */ FIFOPROTOCOL_SENSOR_ID_EXT_2,
		/* TYPE_CUSTOM3                */ FIFOPROTOCOL_SENSOR_ID_EXT_3,
		/* TYPE_CUSTOM4                */ FIFOPROTOCOL_SENSOR_ID_EXT_4,
		/* TYPE_CUSTOM5                */ FIFOPROTOCOL_SENSOR_ID_EXT_5,
		/* TYPE_CUSTOM6                */ FIFOPROTOCOL_SENSOR_ID_EXT_6,
		/* TYPE_CUSTOM7                */ FIFOPROTOCOL_SENSOR_ID_EXT_7,
	};

	if(!INV_SENSOR_IS_VALID(type))
		return -1;

	sensorid = map[type & ~FIFOPROTOCOL_SENSOR_ID_WAKEUP_FLAG];

	if(sensorid != -1 && INV_SENSOR_IS_WU(type)) {
		sensorid |= FIFOPROTOCOL_SENSOR_ID_WAKEUP_FLAG;
	}

	return sensorid;
}

static int fifo_protocol_id_2_sensor_type(uint8_t sensorid)
{
	int type;
	static int map[FIFOPROTOCOL_SENSOR_ID_EXT_MAX] = {
		/* ID_META_DATA              */ INV_SENSOR_TYPE_META_DATA,
		/* ID_ACCELEROMETER          */ INV_SENSOR_TYPE_ACCELEROMETER,
		/* ID_ORIENTATION            */ INV_SENSOR_TYPE_MAGNETOMETER,
		/* ID_GYROMETER              */ INV_SENSOR_TYPE_ORIENTATION,
		/* ID_GYROMETER              */ INV_SENSOR_TYPE_GYROMETER,
		/* ID_LIGHT                  */ INV_SENSOR_TYPE_LIGHT,
		/* ID_PRESSURE               */ INV_SENSOR_TYPE_PRESSURE,
		/* ID_TEMPERATURE            */ INV_SENSOR_TYPE_TEMPERATURE,
		/* ID_PROXIMITY              */ INV_SENSOR_TYPE_PROXIMITY,
		/* ID_GRAVITY                */ INV_SENSOR_TYPE_GRAVITY,
		/* ID_LINEAR_ACCELERATION    */ INV_SENSOR_TYPE_LINEAR_ACCELERATION,
		/* ID_ROTATION_VECTOR        */ INV_SENSOR_TYPE_ROTATION_VECTOR,
		/* ID_HUMIDITY               */ INV_SENSOR_TYPE_HUMIDITY,
		/* ID_AMBIENT_TEMPERATURE    */ INV_SENSOR_TYPE_AMBIENT_TEMPERATURE,
		/* ID_UNCAL_MAGNETOMETER     */ INV_SENSOR_TYPE_UNCAL_MAGNETOMETER,
		/* ID_GAME_ROTATION_VECTOR   */ INV_SENSOR_TYPE_GAME_ROTATION_VECTOR,
		/* ID_UNCAL_GYROMETER        */ INV_SENSOR_TYPE_UNCAL_GYROMETER,
		/* ID_SMD                    */ INV_SENSOR_TYPE_SMD,
		/* ID_STEP_DETECTOR          */ INV_SENSOR_TYPE_STEP_DETECTOR,
		/* ID_STEP_COUNTER           */ INV_SENSOR_TYPE_STEP_COUNTER,
		/* ID_GEOMAG_ROTATION_VECTOR */ INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR,
		/* ID_HEART_RATE             */ INV_SENSOR_TYPE_HEART_RATE,
		/* ID_TILT_DETECTOR          */ INV_SENSOR_TYPE_TILT_DETECTOR,
		/* ID_WAKE_GESTURE           */ INV_SENSOR_TYPE_WAKE_GESTURE,
		/* ID_GLANCE_GESTURE         */ INV_SENSOR_TYPE_GLANCE_GESTURE,
		/* ID_PICK_UP_GESTURE        */ INV_SENSOR_TYPE_PICK_UP_GESTURE,
		/* ID_RESERVED_26            */ -1,
		/* ID_RESERVED_27            */ -1,
		/* ID_RESERVED_28            */ -1,
		/* ID_RESERVED_29            */ -1,
		/* ID_RESERVED_30            */ -1,
		/* ACTIVITY_CLASSIFIERID_BAC */ INV_SENSOR_TYPE_BAC,
		/* ID_EXT_0                  */ INV_SENSOR_TYPE_CUSTOM0, //INV_SENSOR_TYPE_PDR,
		/* ID_EXT_1                  */ INV_SENSOR_TYPE_CUSTOM1,
		/* ID_EXT_2                  */ INV_SENSOR_TYPE_CUSTOM2,
		/* ID_EXT_3                  */ INV_SENSOR_TYPE_CUSTOM3,
		/* ID_EXT_4                  */ INV_SENSOR_TYPE_CUSTOM4,
		/* ID_EXT_5                  */ INV_SENSOR_TYPE_CUSTOM5,
		/* ID_EXT_6                  */ INV_SENSOR_TYPE_CUSTOM6,
		/* ID_EXT_7                  */ INV_SENSOR_TYPE_CUSTOM7, //INV_SENSOR_TYPE_B2S,
	};

	if((sensorid & ~FIFOPROTOCOL_SENSOR_ID_WAKEUP_FLAG)
			>= FIFOPROTOCOL_SENSOR_ID_EXT_MAX)
		return -1;

	type = map[(sensorid & ~FIFOPROTOCOL_SENSOR_ID_WAKEUP_FLAG)];

	if(type != -1 && !!(sensorid & FIFOPROTOCOL_SENSOR_ID_WAKEUP_FLAG)) {
		type  |= INV_SENSOR_TYPE_WU_FLAG;
	}

	return type;
}

static inv_bool_t build_sensor_event(void * context, uint8_t sensorid, uint32_t timestamp,
		uint8_t status, uint8_t accuracy, const uint8_t * data, uint16_t len,
		inv_sensor_event_t * event)
{
	inv_device_icm30xxx_t * self = (inv_device_icm30xxx_t *)context;

	inv_bool_t wakeup = !!(sensorid & FIFOPROTOCOL_SENSOR_ID_WAKEUP_FLAG);

	sensorid &= ~FIFOPROTOCOL_SENSOR_ID_WAKEUP_FLAG;

	assert(event);

	memset(event, 0, sizeof(*event));

	if(status == FIFOPROTOCOL_STATUS_DATAUPDATED || status == FIFOPROTOCOL_STATUS_POLL) {
		switch(sensorid) {
		case FIFOPROTOCOL_SENSOR_ID_META_DATA:
			break;

		case FIFOPROTOCOL_SENSOR_ID_ACCELEROMETER:
		case FIFOPROTOCOL_SENSOR_ID_GRAVITY:
		case FIFOPROTOCOL_SENSOR_ID_LINEAR_ACCELERATION:
			event->data.acc.accuracy_flag = accuracy;
			inv_fifo_protocol_helper_decode_acc_data(data, len, &event->data.acc.vect[0],
					&event->data.acc.vect[1], &event->data.acc.vect[2]);
			break;

		case FIFOPROTOCOL_SENSOR_ID_MAGNETOMETER:
			event->data.mag.accuracy_flag = accuracy;
			event->data.mag.bias[0] = 0;
			event->data.mag.bias[1] = 0;
			event->data.mag.bias[2] = 0;
			inv_fifo_protocol_helper_decode_mag_data(data, len, &event->data.mag.vect[0],
					&event->data.mag.vect[1], &event->data.mag.vect[2]);
			break;

		case FIFOPROTOCOL_SENSOR_ID_UNCAL_MAGNETOMETER:
			event->data.mag.accuracy_flag = accuracy;
			inv_fifo_protocol_helper_decode_mag_data(data, len, &event->data.mag.vect[0],
					&event->data.mag.vect[1], &event->data.mag.vect[2]);
			inv_fifo_protocol_helper_decode_mag_bias_data(data, len, &event->data.mag.bias[0],
					&event->data.mag.bias[1], &event->data.mag.bias[2]);
			break;

		case FIFOPROTOCOL_SENSOR_ID_GYROMETER:
			event->data.gyr.accuracy_flag = accuracy;
			event->data.gyr.bias[0] = 0;
			event->data.gyr.bias[1] = 0;
			event->data.gyr.bias[2] = 0;
			inv_fifo_protocol_helper_decode_gyr_data(data, len, &event->data.gyr.vect[0],
					&event->data.gyr.vect[1], &event->data.gyr.vect[2]);
			break;

		case FIFOPROTOCOL_SENSOR_ID_UNCAL_GYROMETER:
			event->data.gyr.accuracy_flag = accuracy;
			inv_fifo_protocol_helper_decode_gyr_data(data, len, &event->data.gyr.vect[0],
					&event->data.gyr.vect[1], &event->data.gyr.vect[2]);
			inv_fifo_protocol_helper_decode_gyr_bias_data(data, len, &event->data.gyr.bias[0],
					&event->data.gyr.bias[1], &event->data.mag.bias[2]);
			break;

		case FIFOPROTOCOL_SENSOR_ID_ORIENTATION:
			event->data.orientation.accuracy_flag = accuracy;
			inv_fifo_protocol_helper_decode_orientation_data(data, len,
					&event->data.orientation.x, &event->data.orientation.y,
					&event->data.orientation.z);
			break;

		case FIFOPROTOCOL_SENSOR_ID_ROTATION_VECTOR:
		case FIFOPROTOCOL_SENSOR_ID_GEOMAG_ROTATION_VECTOR:
		case FIFOPROTOCOL_SENSOR_ID_GAME_ROTATION_VECTOR:
			event->data.quaternion.accuracy = 0;
			inv_fifo_protocol_helper_decode_q30_quat(data, len, &event->data.quaternion.quat[1],
				&event->data.quaternion.quat[2], &event->data.quaternion.quat[3],
				&event->data.quaternion.quat[0], &event->data.quaternion.accuracy);
			break;

		case FIFOPROTOCOL_SENSOR_ID_STEP_COUNTER:
			event->data.step.count  = (uint32_t)(data[0]);
			event->data.step.count |= (uint32_t)(data[1] << 8U);
			event->data.step.count |= (uint32_t)(data[2] << 16U);
			event->data.step.count |= (uint32_t)(data[3] << 24U);
			break;

		case FIFOPROTOCOL_SENSOR_ID_HEART_RATE:
			inv_fifo_protocol_helper_decode_heart_rate_data(data, len,
				&event->data.hrm.bpm);
			break;

		case FIFOPROTOCOL_SENSOR_ID_LIGHT:
				inv_fifo_protocol_helper_decode_light_data(data, len,
					&event->data.light.level);
			break;

		case FIFOPROTOCOL_SENSOR_ID_PRESSURE:
				inv_fifo_protocol_helper_decode_pressure_data(data, len,
				&event->data.pressure.pressure);
			break;

		case FIFOPROTOCOL_SENSOR_ID_TEMPERATURE:
		case FIFOPROTOCOL_SENSOR_ID_AMBIENT_TEMPERATURE:
			inv_fifo_protocol_helper_decode_ambient_temperature_data(data, len,
				&event->data.temperature.tmp);
			break;

		case FIFOPROTOCOL_SENSOR_ID_PROXIMITY:
			event->data.proximity.distance  = data[0];
			event->data.proximity.distance |= data[1] << 8;
			break;

		case FIFOPROTOCOL_SENSOR_ID_SMD:
		case FIFOPROTOCOL_SENSOR_ID_STEP_DETECTOR:
		case FIFOPROTOCOL_SENSOR_ID_TILT_DETECTOR:
		case FIFOPROTOCOL_SENSOR_ID_WAKE_GESTURE:
		case FIFOPROTOCOL_SENSOR_ID_GLANCE_GESTURE:
		case FIFOPROTOCOL_SENSOR_ID_PICK_UP_GESTURE:
			event->data.event = true;
			break;

		case FIFOPROTOCOL_SENSOR_ID_ACTIVITY_CLASSIFIER:
			event->data.bac.event = (data[0] & 0x7f);
			if(data[0] & 0x80)
				event->data.bac.event = -event->data.bac.event;
			break;

		case FIFOPROTOCOL_SENSOR_ID_HUMIDITY:
			assert(0); // TODO
			break;

		// case FIFOPROTOCOL_SENSOR_ID_EXT_0: /* PDR */
		// 	assert(len <= sizeof(event->data.pdr.fxdata));
		// 	memcpy(event->data.pdr.fxdata, data, len);
		// 	break;

		case FIFOPROTOCOL_SENSOR_ID_EXT_0:
		case FIFOPROTOCOL_SENSOR_ID_EXT_1:
		case FIFOPROTOCOL_SENSOR_ID_EXT_2:
		case FIFOPROTOCOL_SENSOR_ID_EXT_3:
		case FIFOPROTOCOL_SENSOR_ID_EXT_4:
		case FIFOPROTOCOL_SENSOR_ID_EXT_5:
		case FIFOPROTOCOL_SENSOR_ID_EXT_6:
		case FIFOPROTOCOL_SENSOR_ID_EXT_7:
			memcpy(event->data.reserved, data, (len > sizeof(event->data.reserved))
					? sizeof(event->data.reserved) : len);
			break;

		default:
			/* unexpected sensor id */
			INV_MSG(INV_MSG_LEVEL_WARNING,
					"Unexpected data frame received for sensor id %d. Ignored.",
					sensorid);
			return false;
		}
	}

	/* finish up building event */
	event->sensor    = fifo_protocol_id_2_sensor_type(sensorid);
	
	event->timestamp = timestamp;
	event->timestamp *= self->clock_rate; /* to ns */
	event->timestamp /= 1000;             /* to us */

	event->status    = status;
	if(wakeup)
		event->sensor |= INV_SENSOR_TYPE_WU_FLAG;

	return true;
}
