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

/** @defgroup DriverIcm30xxxCmd Icm30xxx driver command
 *  @brief    Low-level driver functions to send specific commands to ICM30xxx device
 *  @ingroup  DriverIcm30xxx
 *  @{
 */

#ifndef _INV_ICM30XXX_CMD_H_
#define _INV_ICM30XXX_CMD_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "Invn/InvBool.h"

/* forward declaration */
struct inv_icm30xxx;
struct inv_icm30xxx_fw_version;

int inv_icm30xxx_cmd_load(struct inv_icm30xxx * s, uint8_t who, uint8_t what,
		uint32_t arg);

int inv_icm30xxx_cmd_get_fw_info(struct inv_icm30xxx * s, struct inv_icm30xxx_fw_version * fw);

int inv_icm30xxx_cmd_enable_sensor(struct inv_icm30xxx * s, uint8_t sensorid,
		inv_bool_t state);

int inv_icm30xxx_cmd_set_running_state(struct inv_icm30xxx * s, inv_bool_t state);

int inv_icm30xxx_cmd_set_sensor_period(struct inv_icm30xxx * s, uint8_t sensorid,
		uint32_t ms_period);

int inv_icm30xxx_cmd_set_sensor_timeout(struct inv_icm30xxx * s, uint8_t sensorid,
		uint32_t ms_timeout);

int inv_icm30xxx_cmd_flush_sensor(struct inv_icm30xxx * s, uint8_t sensorid);

int inv_icm30xxx_cmd_set_sensor_mounting_matrix(struct inv_icm30xxx * s, uint8_t sensorid,
		const float matrix[9]);

int inv_icm30xxx_cmd_set_sensor_bias(struct inv_icm30xxx * s, uint8_t sensorid,
		const float bias[3]);

int inv_icm30xxx_cmd_get_sensor_bias(struct inv_icm30xxx * s, uint8_t sensorid,
		float bias[3]);

int inv_icm30xxx_cmd_set_sensor_gain(struct inv_icm30xxx * s, uint8_t sensorid,
		const float gain[3]);

int inv_icm30xxx_cmd_get_sensor_gain(struct inv_icm30xxx * s, uint8_t sensorid,
		float gain[3]);

int inv_icm30xxx_cmd_get_clock_rate(struct inv_icm30xxx * s, uint32_t * clock_rate);

int inv_icm30xxx_cmd_get_last_sensor_data(struct inv_icm30xxx * s, uint8_t sensorid,
		uint32_t * timestamp, uint8_t * data, uint16_t * len);

int inv_icm30xxx_cmd_ping_sensor(struct inv_icm30xxx * s, uint8_t sensorid,
		uint8_t * ping_value);

#ifdef __cplusplus
}
#endif

#endif /* _INV_ICM30XXX_CMD_H_ */

/** @} */
