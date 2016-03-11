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

#include "DeviceDummy.h"

#include "Invn/Utils/Message.h"

#include <string.h>

static const inv_device_vt_t device_dummy_vt = {
	inv_device_dummy_whoami,
	inv_device_dummy_reset,
	inv_device_dummy_setup,
	inv_device_dummy_cleanup,
	inv_device_dummy_load,
	inv_device_dummy_poll,
	inv_device_dummy_self_test,
	inv_device_dummy_get_fw_info,
	inv_device_dummy_ping_sensor,
	inv_device_dummy_set_running_state,
	inv_device_dummy_start_sensor,
	inv_device_dummy_stop_sensor,
	inv_device_dummy_set_sensor_period,
	inv_device_dummy_set_sensor_timeout,
	inv_device_dummy_flush_sensor,
	inv_device_dummy_set_sensor_bias,
	inv_device_dummy_get_sensor_bias,
	inv_device_dummy_set_sensor_mounting_matrix,
	inv_device_dummy_get_sensor_data,
};

static inline int get_sensor_idx(int sensor)
{
	assert(INV_SENSOR_IS_VALID(sensor));

	return INV_SENSOR_ID_TO_TYPE(sensor)
			+ INV_SENSOR_IS_WU(sensor)*INV_SENSOR_TYPE_MAX;
}

static void update_event(inv_sensor_event_t * event)
{
	int sensor = INV_SENSOR_ID_TO_TYPE(event->sensor);
	unsigned i;

	if(event->status != INV_SENSOR_STATUS_DATA_UPDATED) {
		return;
	}

	switch(sensor) {
	case INV_SENSOR_TYPE_ACCELEROMETER:
	case INV_SENSOR_TYPE_GRAVITY:
	case INV_SENSOR_TYPE_LINEAR_ACCELERATION:
		event->data.acc.vect[0]       += 0.1f;
		event->data.acc.vect[1]       += 0.2f;
		event->data.acc.vect[2]       += 0.3f;
		event->data.acc.accuracy_flag += 1;
		for(i = 0; i < 3; ++i) {
			if(event->data.acc.vect[i] > 8.0f)
				event->data.acc.vect[i] = -8.0f;
		}
		break;

	case INV_SENSOR_TYPE_MAGNETOMETER:
	case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER:
		event->data.mag.vect[0]       += 0.5f;
		event->data.mag.bias[0]       += 0.5f;
		event->data.mag.vect[1]       += 1.0f;
		event->data.mag.bias[1]       += 1.0f;
		event->data.mag.vect[2]       += 1.5f;
		event->data.mag.bias[2]       += 1.5f;
		event->data.mag.accuracy_flag += 1;
		for(i = 0; i < 3; ++i) {
			if(event->data.mag.vect[i] > 60.0f)
				event->data.mag.vect[i] = -60.0f;
			if(event->data.mag.bias[i] > 60.0f)
				event->data.mag.bias[i] = -60.0f;
		}
		break;

	case INV_SENSOR_TYPE_GYROMETER:
	case INV_SENSOR_TYPE_UNCAL_GYROMETER:
		event->data.gyr.vect[0]       += 0.5f;
		event->data.gyr.bias[0]       += 0.5f;
		event->data.gyr.vect[1]       += 1.0f;
		event->data.gyr.bias[1]       += 1.0f;
		event->data.gyr.vect[2]       += 1.5f;
		event->data.gyr.bias[2]       += 1.5f;
		event->data.gyr.accuracy_flag += 1;
		for(i = 0; i < 3; ++i) {
			if(event->data.gyr.vect[i] > 2000.0f)
				event->data.gyr.vect[i] = -2000.0f;
			if(event->data.gyr.bias[i] > 2000.0f)
				event->data.gyr.bias[i] = -2000.0f;
		}
		break;

	case INV_SENSOR_TYPE_ORIENTATION:
		event->data.orientation.x       += 0.5f;
		event->data.orientation.y       += 1.0f;
		event->data.orientation.z       += 1.5f;
		event->data.orientation.accuracy_flag += 1;
		if(event->data.orientation.x > 360.0f)
			event->data.orientation.x = -360.0f;
		if(event->data.orientation.y > 360.0f)
			event->data.orientation.y = -360.0f;
		if(event->data.orientation.z > 360.0f)
			event->data.orientation.z = -360.0f;
		break;

	case INV_SENSOR_TYPE_ROTATION_VECTOR:
	case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
	case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
		event->data.quaternion.accuracy = 0;
		event->data.quaternion.quat[0] = 0.01f;
		event->data.quaternion.quat[1] = 0.02f;
		event->data.quaternion.quat[2] = 0.03f;
		event->data.quaternion.quat[3] = 0.04f;

		for(i = 0; i < 4; ++i) {
			if(event->data.quaternion.quat[i] > 1.0f)
				event->data.quaternion.quat[i] = -1.0f;
		}
		break;

	case INV_SENSOR_TYPE_STEP_COUNTER:
		event->data.step.count += 1;
		break;

	case INV_SENSOR_TYPE_HEART_RATE:
		event->data.hrm.bpm += 1;
		break;

	case INV_SENSOR_TYPE_SMD:
	case INV_SENSOR_TYPE_STEP_DETECTOR:
	case INV_SENSOR_TYPE_TILT_DETECTOR:
	case INV_SENSOR_TYPE_WAKE_GESTURE:
	case INV_SENSOR_TYPE_GLANCE_GESTURE:
	case INV_SENSOR_TYPE_PICK_UP_GESTURE:
		event->data.event = true;
		break;

	case INV_SENSOR_TYPE_CUSTOM0:
	case INV_SENSOR_TYPE_CUSTOM1:
	case INV_SENSOR_TYPE_CUSTOM2:
	case INV_SENSOR_TYPE_CUSTOM3:
	case INV_SENSOR_TYPE_CUSTOM4:
	case INV_SENSOR_TYPE_CUSTOM5:
	case INV_SENSOR_TYPE_CUSTOM6:
	case INV_SENSOR_TYPE_CUSTOM7:
	default:
		/* just increment the value to make create a visible change */
		for(i = 0; i < sizeof(event->data.reserved); ++i) {
			++event->data.reserved[i];
		}
		break;
	}

}

/* Simulate sensor polling
 * Generate fake data according to configured RI/Batch timeout
 */
static inv_bool_t poll_sensor(inv_device_dummy_t * self, int sensor, uint64_t now)
{
	struct inv_device_dummy_sensor_state * states =
			&self->sensor_states[get_sensor_idx(sensor)];

	assert(states->active);

	/* First poll since sensor was started */
	if(states->next_time == (uint64_t)-1LL) {
		memset(&states->event, 0, sizeof(states->event));
		states->event.sensor    = sensor;
		states->event.status    = INV_SENSOR_STATUS_STATE_CHANGED;
		states->event.timestamp = now;

		states->next_time = now;
		if(states->timeout == 0) {
			states->next_time += states->period * 1000;
		} else {
			states->next_time += states->timeout * 1000;
		}

		return true;

	/* Sensor already started */
	} else if(states->next_time <= now) {
		states->event.sensor     = sensor;
		states->event.status     = INV_SENSOR_STATUS_DATA_UPDATED;
		states->event.timestamp += states->period*1000;

		update_event(&states->event);

		if(states->event.timestamp >= now) {
			states->next_time = now;
			if(states->timeout == 0) {
				states->next_time += states->period * 1000;
			} else {
				states->next_time += states->timeout * 1000;
			}
		}

		return true;
	}

	return false;
}

void inv_device_dummy_init(inv_device_dummy_t * self,
		const inv_host_serif_t * serif, const inv_sensor_listener_t * listener,
		uint64_t (*fget_us_time)(void))
{
	assert(self && fget_us_time);

	memset(self, 0, sizeof(*self));
	self->base.instance = self;
	self->base.vt       = &device_dummy_vt;
	self->base.serif    = serif;
	self->base.listener = listener;
	self->fget_us_time  = fget_us_time;
}

int inv_device_dummy_poll(void * context)
{
	inv_device_dummy_t * self = (inv_device_dummy_t *)context;
	uint64_t now = self->fget_us_time();
	int sensor;

	for(sensor = 0; sensor < INV_SENSOR_TYPE_MAX; ++sensor) {
		if(self->sensor_states[sensor].active) {
			while(poll_sensor(self, sensor, now))
				inv_sensor_listener_notify(self->base.listener,
						&self->sensor_states[sensor].event);
		}
		if(self->sensor_states[sensor+INV_SENSOR_TYPE_MAX].active) {
			while(poll_sensor(self, sensor | INV_SENSOR_TYPE_WU_FLAG, now))
				inv_sensor_listener_notify(self->base.listener, 
						&self->sensor_states[sensor+INV_SENSOR_TYPE_MAX].event);
		}
	}

	return 0;
}

int inv_device_dummy_whoami(void * context, uint8_t * whoami)
{
	(void)context;

	*whoami = 'I';

	return 0;
}

int inv_device_dummy_reset(void * context)
{
	(void)context;

	INV_MSG(INV_MSG_LEVEL_DEBUG, "device_dummy::reset()");

	return 0;
}

int inv_device_dummy_setup(void * context)
{
	(void)context;

	INV_MSG(INV_MSG_LEVEL_DEBUG, "device_dummy::setup()");

	return 0;
}

int inv_device_dummy_cleanup(void * context)
{
	(void)context;

	INV_MSG(INV_MSG_LEVEL_DEBUG, "device_dummy::cleanup()");

	return 0;
}

int inv_device_dummy_load(void * context, int what,
		const uint8_t * image, uint32_t size, inv_bool_t verify, inv_bool_t force)
{
	(void)context, (void)what, (void)image, (void)size, (void)verify, (void)force;

	INV_MSG(INV_MSG_LEVEL_DEBUG, "device_dummy::load(what=%d,image=%p"
		",size=%u,verify=%d,force=%d)", what, image, size, verify, force);

	return 0;
}

int inv_device_dummy_get_fw_info(void * context,
		struct inv_fw_version * version)
{
	(void)context;

	version->major = 6;
	version->minor = 6;
	version->patch = 6;
	version->suffix[0] = '\0';

	INV_MSG(INV_MSG_LEVEL_DEBUG, "device_dummy::get_fw_version()=%d.%d.%d",
		version->major, version->minor, version->patch);

	return 0;
}

int inv_device_dummy_set_running_state(void * context, inv_bool_t state)
{
	(void)context, (void)state;

	INV_MSG(INV_MSG_LEVEL_DEBUG, "device_dummy::set_running_state(%d)", state);

	return 0;
}

int inv_device_dummy_ping_sensor(void * context, int sensor)
{
	(void)context;

	INV_MSG(INV_MSG_LEVEL_DEBUG, "device_dummy::ping_sensor(%s)",
			inv_sensor_2str(sensor));

	return (INV_SENSOR_IS_VALID(sensor)) ? 0 : INV_ERROR;
}

int inv_device_dummy_start_sensor(void * context, int sensor)
{
	inv_device_dummy_t * self = (inv_device_dummy_t *)context;

	INV_MSG(INV_MSG_LEVEL_DEBUG, "device_dummy::start_sensor(%s)",
			inv_sensor_2str(sensor));

	if(!INV_SENSOR_IS_VALID(sensor))
		return INV_ERROR;

	self->sensor_states[get_sensor_idx(sensor)].active    = true;
	self->sensor_states[get_sensor_idx(sensor)].next_time = (uint64_t)-1LL;
	if(self->sensor_states[get_sensor_idx(sensor)].period == 0)
		self->sensor_states[get_sensor_idx(sensor)].period = 200;

	return 0;
}

int inv_device_dummy_stop_sensor(void * context, int sensor)
{
	inv_device_dummy_t * self = (inv_device_dummy_t *)context;

	INV_MSG(INV_MSG_LEVEL_DEBUG, "device_dummy::stop_sensor(%s)",
			inv_sensor_2str(sensor));

	if(!INV_SENSOR_IS_VALID(sensor))
		return INV_ERROR;

	self->sensor_states[get_sensor_idx(sensor)].active    = false;

	return 0;
}

int inv_device_dummy_set_sensor_period(void * context,
		int sensor, uint32_t period)
{
	inv_device_dummy_t * self = (inv_device_dummy_t *)context;

	INV_MSG(INV_MSG_LEVEL_DEBUG, "device_dummy::set_sensor_period(%s,%u)",
			inv_sensor_2str(sensor), period);

	if(!INV_SENSOR_IS_VALID(sensor))
		return INV_ERROR;

	if(period == 0)
		period = 200;

	self->sensor_states[get_sensor_idx(sensor)].period = period;

	return 0;
}

int inv_device_dummy_set_sensor_timeout(void * context,
		int sensor, uint32_t timeout)
{
	inv_device_dummy_t * self = (inv_device_dummy_t *)context;

	INV_MSG(INV_MSG_LEVEL_DEBUG, "device_dummy::set_sensor_timeout(%s,%u)",
			inv_sensor_2str(sensor), timeout);

	if(!INV_SENSOR_IS_VALID(sensor))
		return INV_ERROR;

	// if(timeout == 0) {
	// 	timeout = self->sensor_states[get_sensor_idx(sensor)].period;
	// }
	self->sensor_states[get_sensor_idx(sensor)].timeout = timeout;

	return 0;
}

int inv_device_dummy_flush_sensor(void * context, int sensor)
{
	inv_device_dummy_t * self = (inv_device_dummy_t *)context;
	uint64_t now = self->fget_us_time();
	struct inv_device_dummy_sensor_state * states;

	INV_MSG(INV_MSG_LEVEL_DEBUG, "device_dummy::flush_sensor(%s)",
			inv_sensor_2str(sensor));

	if(!INV_SENSOR_IS_VALID(sensor))
		return INV_ERROR;

	states = &self->sensor_states[get_sensor_idx(sensor)];

	if(states->active) {
		states->next_time = now;
		while(poll_sensor(self, sensor, now))
			inv_sensor_listener_notify(self->base.listener, &states->event);
	}

	states->event.status = INV_SENSOR_STATUS_FLUSH_COMPLETE;
	inv_sensor_listener_notify(self->base.listener, &states->event);

	return 0;
}

int inv_device_dummy_set_sensor_bias(void * context, int sensor,
		const float bias[3])
{
	inv_device_dummy_t * self = (inv_device_dummy_t *)context;
	int idx;

	INV_MSG(INV_MSG_LEVEL_DEBUG, "device_dummy::set_sensor_bias(%s,[%f %f %f])",
			inv_sensor_2str(sensor), bias[0], bias[1], bias[2]);

	sensor &= ~INV_SENSOR_TYPE_WU_FLAG;

	if(sensor == INV_SENSOR_TYPE_ACCELEROMETER) {
		idx = 0;
	} else if(sensor == INV_SENSOR_TYPE_MAGNETOMETER) {
		idx = 1;
	} else if(sensor == INV_SENSOR_TYPE_GYROMETER) {
		idx = 2;
	} else {
		return INV_ERROR;
	}

	memcpy(self->sensor_cfg[idx].bias, bias, 3*sizeof(float));

	return 0;
}

int inv_device_dummy_get_sensor_bias(void * context, int sensor,
		float bias[3])
{
	inv_device_dummy_t * self = (inv_device_dummy_t *)context;
	int idx;

	sensor &= ~INV_SENSOR_TYPE_WU_FLAG;

	if(sensor == INV_SENSOR_TYPE_ACCELEROMETER) {
		idx = 0;
	} else if(sensor == INV_SENSOR_TYPE_MAGNETOMETER) {
		idx = 1;
	} else if(sensor == INV_SENSOR_TYPE_GYROMETER) {
		idx = 2;
	} else {
		return INV_ERROR;
	}

	memcpy(bias, self->sensor_cfg[idx].bias, 3*sizeof(float));

	INV_MSG(INV_MSG_LEVEL_DEBUG, "device_dummy::get_sensor_bias(%s)=[%f %f %f]",
			inv_sensor_2str(sensor), bias[0], bias[1], bias[2]);

	return 0;
}

int inv_device_dummy_set_sensor_mounting_matrix(void * context,
		int sensor, const float matrix[9])
{
	(void)context, (void)sensor, (void)matrix;

	INV_MSG(INV_MSG_LEVEL_DEBUG, "device_dummy::set_mounting_matrix(%s,"
			"[%f %f %f ; %f %f %f ; %f %f %f])", inv_sensor_2str(sensor),
			matrix[0], matrix[1], matrix[2],
			matrix[3], matrix[4], matrix[5],
			matrix[6], matrix[7], matrix[8]);

	return 0;
}

int inv_device_dummy_get_sensor_data(void * context, int sensor,
		inv_sensor_event_t * event)
{
	inv_device_dummy_t * self = (inv_device_dummy_t *)context;

	INV_MSG(INV_MSG_LEVEL_DEBUG, "device_dummy::get_sensor_data(%s)",
			inv_sensor_2str(sensor));

	if(!INV_SENSOR_IS_VALID(sensor))
		return INV_ERROR;

	memcpy(event,
			&self->sensor_states[get_sensor_idx(sensor)].event,
			sizeof(inv_sensor_event_t));
	event->sensor = sensor;
	event->status = INV_SENSOR_STATUS_POLLED_DATA;

	return 0;
}

int inv_device_dummy_self_test(void * context, int sensor)
{
	(void)context, (void)sensor;

	INV_MSG(INV_MSG_LEVEL_DEBUG, "device_dummy::self_test(%s)",
			inv_sensor_2str(sensor));

	return INV_ERROR_NIMPL;
}
