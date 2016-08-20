/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2015-2016 InvenSense Inc. All rights reserved.
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

/** @defgroup SensorConfig Sensor Configuration
 *  @brief    General sensor configuration types definitions
 *  @{
 */

#ifndef _INV_SENSOR_CONFIG_H_
#define _INV_SENSOR_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "Invn/InvBool.h"

/** @brief Sensor type identifier definition
 */
enum inv_sensor_config {
	INV_SENSOR_CONFIG_RESERVED = 0,     /**< Reserved config ID: do not use */
	INV_SENSOR_CONFIG_MOUNTING_MATRIX,  /**< 3x3 mounting matrix */
	INV_SENSOR_CONFIG_GAIN,             /**< 3x3 gain matrix (to correct for cross-axis defect)*/
	INV_SENSOR_CONFIG_OFFSET,           /**< 3d offset vector  */
	INV_SENSOR_CONFIG_CONTEXT,          /**< arbitrary context buffer */
	INV_SENSOR_CONFIG_FSR,              /**< Full scale range */
	INV_SENSOR_CONFIG_CUSTOM   = 128 ,  /**< Configuration ID above this value are device specific */
	INV_SENSOR_CONFIG_MAX      = 255,   /**< Absolute maximum value for sensor config */
};

/** @brief Define mounting matrix value for 3-axis sensors
 *         (associated with INV_SENSOR_CONFIG_MOUNTING_MATRIX config ID)
 *         Mounting matrix value can be set (is supported by device implementation) to convert from
 *         sensor reference to system reference.
 *         Value is expetcted to be a rotation matrix.
 */
typedef struct inv_sensor_config_mounting_mtx {
	float matrix[3*3];
} inv_sensor_config_mounting_mtx_t;

/** @brief Define gain matrix value for 3-axis sensors
 *         (associated with INV_SENSOR_CONFIG_GAIN config ID)
 *         Gain matrix value can be set (is supported by device implementation) to correct for
 *         cross-axis defect.
 */
typedef struct inv_sensor_config_gain {
	float gain[3*3];
} inv_sensor_config_gain_t;

/** @brief Define offset vector value for 3-axis sensors
 *         (associated with INV_SENSOR_CONFIG_OFFSET config ID)
 *         Offset value can be set (is supported by device implementation) to correct for bias defect.
 *         If applied to RAW sensor, value is expected to be in lsb.
 *         If applied to other sensor, value is expected to be in sensor unit (g, uT or dps).
 */
typedef struct inv_sensor_config_offset {
	float offset[3];
} inv_sensor_config_offset_t;

/** @brief Define configuration context value
 *         (associated with INV_SENSOR_CONFIG_CONTEXT config ID)
 *         Context is an arbitrary buffer specific to the sensor and device implemetation
 */
typedef struct inv_sensor_config_context {
	uint8_t context[64];
} inv_sensor_config_context_t;

/** @brief Define full-scale range value for accelero, gyro or mangetometer based sensor
 *         (associated with INV_SENSOR_CONFIG_FSR config ID)
 *         Value is expetcted to be expressed in mg, dps and uT for accelero, gyro or mangetometer
 *         eg: +/-2g = 2000
 *             +/-250 dps = 250
 *             +/-2000 uT = 2000
 */
typedef struct inv_sensor_config_fsr {
	uint32_t fsr;
} inv_sensor_config_fsr_t;

#ifdef __cplusplus
}
#endif

#endif /* _INV_SENSOR_CONFIG_H_ */

/** @} */
