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

#include "Icm30xxxCmd.h"
#include "Icm30xxx.h"

#include "Invn/FifoProtocol/FifoProtocol.h"
#include "Invn/FifoProtocol/FifoProtocolHelper.h"

#include "Invn/Utils/Message.h"

#include <assert.h>

static int send_cmd(struct inv_icm30xxx * s, uint8_t sensor, uint8_t cmd,
		const uint8_t * payload, uint16_t len, inv_bool_t wait_response)
{
	int rc = 0;

	if(wait_response) {
		s->response.event = false;
	}

	if((rc = inv_icm30xxx_ctrl_send_command(s, sensor, cmd, payload, len)) != 0) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "Error %d while sending command '%d'"
				" (payload: %d bytes)", rc, cmd, len);
		return rc;
	}

	if(wait_response) {
		int timeout = 1000; /* 1s */

		/* poll FIFO until we received the answer */
		do {
			if((rc = inv_icm30xxx_ctrl_poll(s)) != 0)
				return rc;

			inv_icm30xxx_sleep(10);
			timeout -= 10;

			if(timeout <= 0)
				return INV_ERROR_TIMEOUT;

		} while(!s->response.event);

		if(s->response.sensorid != sensor || s->response.command != cmd) {
			INV_MSG(INV_MSG_LEVEL_ERROR, "Unexpected response for command");
			INV_MSG(INV_MSG_LEVEL_DEBUG, "sensor %d != %d", s->response.sensorid,
					sensor);
			INV_MSG(INV_MSG_LEVEL_DEBUG, "command %d != %d", s->response.command,
					cmd);

			return INV_ERROR_UNEXPECTED;
		}
	}

	return rc;
}


int inv_icm30xxx_cmd_load(struct inv_icm30xxx * s, uint8_t who, uint8_t what,
		uint32_t arg)
{
	int rc;
	uint8_t payload[6];
	const uint16_t len = inv_fifoprotocol_helper_encode_load(who, what,
			arg, payload);

	if((rc = send_cmd(s, FIFOPROTOCOL_SENSOR_ID_META_DATA,
			FIFOPROTOCOL_CMD_LOAD, payload, len, true)) != 0)
		return rc;

	if(s->response.payload.load.who != who ||
				s->response.payload.load.what != what) {

		INV_MSG(INV_MSG_LEVEL_ERROR, "Unexpected response received.");
		INV_MSG(INV_MSG_LEVEL_DEBUG, "who:  received %d vs %d expected",
				s->response.payload.load.who, who);
		INV_MSG(INV_MSG_LEVEL_DEBUG, "what: received %d vs %d expected",
				s->response.payload.load.what, what);

		return INV_ERROR_UNEXPECTED;
	}

	return 0;
}

int inv_icm30xxx_cmd_get_fw_info(struct inv_icm30xxx * s, struct inv_icm30xxx_fw_version * fw)
{
	int rc;

	if((rc = send_cmd(s, FIFOPROTOCOL_SENSOR_ID_META_DATA,
			FIFOPROTOCOL_CMD_GETFIRMWAREINFO, 0, 0, true)) != 0)
		return rc;

	*fw = s->response.payload.fwversion;

	return 0;
}

int inv_icm30xxx_cmd_enable_sensor(struct inv_icm30xxx * s, uint8_t sensorid,
		inv_bool_t state)
{
	const int cmd = (state) ? FIFOPROTOCOL_CMD_SENSORON : FIFOPROTOCOL_CMD_SENSOROFF;

	return send_cmd(s, sensorid, cmd, 0, 0, false);
}

int inv_icm30xxx_cmd_set_running_state(struct inv_icm30xxx * s, inv_bool_t state)
{
	uint8_t payload[4];
	FifoProtocolApPowerMode power = (state) ?
			FIFOPROTOCOL_APPOWERMODE_ON : FIFOPROTOCOL_APPOWERMODE_SUSPEND;
	const uint16_t len = inv_fifo_protocol_helper_encode_power(power, payload);

	return send_cmd(s, FIFOPROTOCOL_SENSOR_ID_META_DATA,
			FIFOPROTOCOL_CMD_SETPOWER, payload, len, false);
}

int inv_icm30xxx_cmd_set_sensor_period(struct inv_icm30xxx * s, uint8_t sensorid,
		uint32_t ms_period)
{
	uint8_t payload[4];
	const uint16_t len = inv_fifo_protocol_helper_encode_delay(ms_period, payload);

	return send_cmd(s, sensorid, FIFOPROTOCOL_CMD_SETDELAY, payload, len, false);
}

int inv_icm30xxx_cmd_set_sensor_timeout(struct inv_icm30xxx * s, uint8_t sensorid,
		uint32_t ms_timeout)
{
	uint8_t payload[4];
	const uint16_t len = inv_fifo_protocol_helper_encode_timeout(ms_timeout, payload);

	return send_cmd(s, sensorid, FIFOPROTOCOL_CMD_BATCHON, payload, len, false);
}

int inv_icm30xxx_cmd_flush_sensor(struct inv_icm30xxx * s, uint8_t sensorid)
{
	return send_cmd(s, sensorid, FIFOPROTOCOL_CMD_FLUSH, 0, 0, false);
}

int inv_icm30xxx_cmd_set_sensor_mounting_matrix(struct inv_icm30xxx * s, uint8_t sensorid,
		const float matrix[9])
{
	uint8_t payload[9*4];
	// cast because of const attribute missing in prototy of below function
	const uint16_t len = inv_fifo_protocol_helper_encode_referenceframe(
			(float *)matrix, payload);

	return send_cmd(s, sensorid, FIFOPROTOCOL_CMD_SETREFERENCEFRAME,
			payload, len, false);
}

int inv_icm30xxx_cmd_set_sensor_bias(struct inv_icm30xxx * s, uint8_t sensorid,
		const float bias[3])
{
	uint8_t payload[3*4];
	// cast because of const attribute missing in prototy of below function
	const uint16_t len = inv_fifo_protocol_helper_encode_calibrationoffsets(
			(float *)bias, 3*sizeof(float), payload);

	return send_cmd(s, sensorid, FIFOPROTOCOL_CMD_SETCALIBRATIONOFFSETS,
			payload, len, false);
}

int inv_icm30xxx_cmd_get_sensor_bias(struct inv_icm30xxx * s, uint8_t sensorid,
		float bias[3])
{
	int rc;

	if((rc = send_cmd(s, sensorid, FIFOPROTOCOL_CMD_GETCALIBRATIONOFFSETS,
			0, 0, true)) != 0)
		return rc;

	memcpy(bias, s->response.payload.bias, 3*sizeof(float));

	return 0;
}

int inv_icm30xxx_cmd_set_sensor_gain(struct inv_icm30xxx * s, uint8_t sensorid,
		const float gain[9])
{
	uint8_t payload[9*4];
	// cast because of const attribute missing in prototy of below function
	const uint16_t len = inv_fifo_protocol_helper_encode_calibrationgains(
			(float *)gain, 3*sizeof(float), payload);

	return send_cmd(s, sensorid, FIFOPROTOCOL_CMD_SETCALIBRATIONGAINS,
			payload, len, false);
}

int inv_icm30xxx_cmd_get_sensor_gain(struct inv_icm30xxx * s, uint8_t sensorid,
		float gain[9])
{
	int rc;

	if((rc = send_cmd(s, sensorid, FIFOPROTOCOL_CMD_GETCALIBRATIONGAINS,
			0, 0, true)) != 0)
		return rc;

	memcpy(gain, s->response.payload.gain, 9*sizeof(float));

	return 0;
}

int inv_icm30xxx_cmd_get_clock_rate(struct inv_icm30xxx * s, uint32_t * clock_rate)
{
	int rc;

	if((rc = send_cmd(s, FIFOPROTOCOL_SENSOR_ID_META_DATA,
			FIFOPROTOCOL_CMD_GETCLOCKRATE, 0, 0, true)) != 0)
		return rc;

	*clock_rate = s->response.payload.clock_rate;

	return 0;
}

int inv_icm30xxx_cmd_get_last_sensor_data(struct inv_icm30xxx * s, uint8_t sensorid,
		 uint32_t * timestamp, uint8_t * data, uint16_t * len)
{
	int rc;

	if((rc = send_cmd(s, sensorid, FIFOPROTOCOL_CMD_GETDATA,
			0, 0, true)) != 0)
		return rc;

	*len = s->response.payload.sensor_data.len;
	memcpy(data, s->response.payload.sensor_data.data, *len);

	*timestamp = s->response.payload.sensor_data.timestamp;

	return 0;
}

int inv_icm30xxx_cmd_ping_sensor(struct inv_icm30xxx * s, uint8_t sensorid,
		uint8_t * ping_value)
{
	int rc;

	if((rc = send_cmd(s, sensorid, FIFOPROTOCOL_CMD_PING, 0, 0, true)) != 0)
		return rc;

	*ping_value = s->response.payload.ping;

	return 0;
}
