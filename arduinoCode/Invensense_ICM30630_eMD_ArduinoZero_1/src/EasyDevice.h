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

/**
 * @file
 * @brief Group functions call of InvenSense Device Driver for simplification of usage
 */

 
#ifndef EASY_DEVICE_H
#define EASY_DEVICE_H

// Include device driver
#include "Invn/Devices/DeviceIcm30xxx.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * Handle easy device settings
 */
typedef struct 
{
	void                   (*interrupt_cb)(void * context, int int_num);  /*!< Sensor interrupt callback */
	void                   * context;                                     /*!< Context of Sensor interrupt callback */
	inv_device_t           * device;                                      /*!< device object definition */
	const inv_host_serif_t * pserif;                                      /*!< Serial Interface interface definition */
	
	uint8_t                * buffer;                                      /*!< Some memory to be used by the icm30xxx driver (must be >= 4k) */
	uint32_t               buffer_size;                                   /*!< Size of the buffer used by the icm30xxx driver */
	
	inv_device_icm30xxx_t    icm30xxx;                                        /*!< Instance of the icm30xxx device */
	inv_sensor_listener_t  sensor_listener;                               /*!< Sensor event listener definition */
	
	const uint8_t          * fw_image_buffer;                             /*!< M0 flash buffer */
	uint32_t               fw_image_buffer_size;                          /*!< M0 flash buffer size */
	
	const uint8_t          * dmp3_image_buffer;                           /*!< DMP3 image buffer  */
	uint32_t               dmp3_image_buffer_size;                        /*!< DMP3 image buffer size */
	
	const uint8_t          * dmp4_image_buffer;                           /*!< DMP4 image buffer */
	uint32_t               dmp4_image_buffer_size;                        /*!< DMP4 image buffer size */
	
	const float            acc_gyr_mounting_matrix[9];                    /*!< mounting matrix for accelerometer and gyrometer */
	const float            mag_mounting_matrix[9];                        /*!< mounting matrix for magnetometer */
} inv_easy_device_settings_t;


 /** @brief Init the device driver
 *  @param[in]  psettings    settings for libIDD
 *  @param[out] pwhoami      pointer on the who am i read in the chip
 *  @param[out] pfw_version  pointer on the M0 firmware version
 *  @return     0 on success, else a negative value in enum inv_error
 */
int inv_easy_device_init(inv_easy_device_settings_t * psettings, uint8_t *pwhoami, inv_fw_version_t * pfw_version);


/** @brief Start a sensor
 *  @param[in]  psettings    settings for libIDD
 *  @param[in]  sensor       sensor type (as defined by @sa inv_sensor_type_t)
 *  @param[in]  period       requested data period in ms
 *  @param[in]  timeout      timeout in millisecond for buffering of sensor samples, set to 0 to disable the buffering
 *  @return     0 on success, else a negative value in enum inv_error
 */
int inv_easy_device_start_sensor(inv_easy_device_settings_t *psettings, int sensor, uint32_t period, uint32_t timeout);


/** @brief Stop a sensor
 *  @param[in]  psettings    settings for libIDD
 *  @param[in]  sensor       sensor type (as defined by @sa inv_sensor_type_t)
 *  @return     0 on success, else a negative value in enum inv_error
 */
int inv_easy_device_stop_sensor(inv_easy_device_settings_t *psettings, int sensor);


/** @brief Close the device driver
 *  @param[in]  psettings    settings for libIDD
 *  @return     0 on success, else a negative value in enum inv_error
 */
int inv_easy_device_close(inv_easy_device_settings_t * psettings);


#ifdef __cplusplus
}
#endif

#endif /* EASY_DEVICE_H */ 
