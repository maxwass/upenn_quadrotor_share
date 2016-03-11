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

/** @defgroup DriverIcm30xxxUartFfio Icm30xxx driver uart emulation
 *  @brief UART emulation with HW FIFO for Icm30xxx
 *  @ingroup  DriverIcm30xxx
 *  @{
 */

#ifndef _INV_ICM30XXX_UARTFIFO_H_
#define _INV_ICM30XXX_UARTFIFO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>

#include "Invn/InvBool.h"

/* enum for android sensor*/
enum ANDROID_SENSORS {
	ANDROID_SENSOR_META_DATA = 0,
	ANDROID_SENSOR_ACCELEROMETER,
	ANDROID_SENSOR_GEOMAGNETIC_FIELD,
	ANDROID_SENSOR_ORIENTATION,
	ANDROID_SENSOR_GYROSCOPE,
	ANDROID_SENSOR_LIGHT,
	ANDROID_SENSOR_PRESSURE,
	ANDROID_SENSOR_TEMPERATURE,
	ANDROID_SENSOR_WAKEUP_PROXIMITY,
	ANDROID_SENSOR_GRAVITY,
	ANDROID_SENSOR_LINEAR_ACCELERATION,
	ANDROID_SENSOR_ROTATION_VECTOR,
	ANDROID_SENSOR_HUMIDITY,
	ANDROID_SENSOR_AMBIENT_TEMPERATURE,
	ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED,
	ANDROID_SENSOR_GAME_ROTATION_VECTOR,
	ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED,
	ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION,
	ANDROID_SENSOR_STEP_DETECTOR,
	ANDROID_SENSOR_STEP_COUNTER,
	ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR,
	ANDROID_SENSOR_HEART_RATE,
	ANDROID_SENSOR_PROXIMITY,

	ANDROID_SENSOR_WAKEUP_ACCELEROMETER,
	ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD,
	ANDROID_SENSOR_WAKEUP_ORIENTATION,
	ANDROID_SENSOR_WAKEUP_GYROSCOPE,
	ANDROID_SENSOR_WAKEUP_LIGHT,
	ANDROID_SENSOR_WAKEUP_PRESSURE,
	ANDROID_SENSOR_WAKEUP_GRAVITY,
	ANDROID_SENSOR_WAKEUP_LINEAR_ACCELERATION,
	ANDROID_SENSOR_WAKEUP_ROTATION_VECTOR,
	ANDROID_SENSOR_WAKEUP_RELATIVE_HUMIDITY,
	ANDROID_SENSOR_WAKEUP_AMBIENT_TEMPERATURE,
	ANDROID_SENSOR_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED,
	ANDROID_SENSOR_WAKEUP_GAME_ROTATION_VECTOR,
	ANDROID_SENSOR_WAKEUP_GYROSCOPE_UNCALIBRATED,
	ANDROID_SENSOR_WAKEUP_STEP_DETECTOR,
	ANDROID_SENSOR_WAKEUP_STEP_COUNTER,
	ANDROID_SENSOR_WAKEUP_GEOMAGNETIC_ROTATION_VECTOR,
	ANDROID_SENSOR_WAKEUP_HEART_RATE,
	ANDROID_SENSOR_WAKEUP_TILT_DETECTOR,
	ANDROID_SENSOR_NUM_MAX,

	ANDROID_SENSOR_B2S,
	ANDROID_SENSOR_FLIP_PICKUP,
	ANDROID_SENSOR_ACTIVITY_CLASSIFICATON,
	ANDROID_SENSOR_SCREEN_ROTATION,
	SELF_TEST,
	SETUP,
	GENERAL_SENSORS_MAX
};

enum inv_uartfifo_cmd_type {
	UARTFIFOPROTOCOL_CMD_GETINFO,
	UARTFIFOPROTOCOL_CMD_ENABLE,
	UARTFIFOPROTOCOL_CMD_DISABLE,
	UARTFIFOPROTOCOL_CMD_SETODR,
	UARTFIFOPROTOCOL_CMD_BATCH,
	UARTFIFOPROTOCOL_CMD_SELFTEST
};

/* forward declaration */
struct inv_icm30xxx;
struct inv_icm30xxx_fw_version;

int inv_icm30xxx_uartfifo_retrieve_fifo_config(struct inv_icm30xxx * s);
int inv_icm30xxx_uartfifo_poll(struct inv_icm30xxx * s);       
int inv_icm30xxx_uartfifo_get_fw_info(struct inv_icm30xxx * s, struct inv_icm30xxx_fw_version * fw);
int inv_icm30xxx_uartfifo_enable_sensor(struct inv_icm30xxx * s, char sensorid,	inv_bool_t state);
int inv_icm30xxx_uartfifo_set_sensor_period(struct inv_icm30xxx * s, char sensorid,	uint32_t ms_period);
int inv_icm30xxx_uartfifo_set_sensor_timeout(struct inv_icm30xxx * s, char sensorid, uint32_t ms_timeout);
int inv_icm30xxx_uartfifo_get_self_test(struct inv_icm30xxx * s);

#ifdef __cplusplus
}
#endif

#endif /* _INV_ICM30XXX_UARTFIFO_H_ */

/** @} */
