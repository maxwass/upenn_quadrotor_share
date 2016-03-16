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

// We can not processs warning as error on whole code since Arduino libraries
#pragma GCC diagnostic push
#pragma GCC diagnostic warning "-Werror"


/* Includes ------------------------------------------------------------------*/

#include "EasyDevice.h"
#include "Invn/Utils/Message.h"
#include "ArduinoEmdAdapter.h"

/* Private define ------------------------------------------------------------*/

/* Private typedef ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Public function prototypes ------------------------------------------------*/

/* Private function code -----------------------------------------------------*/

/* Public function code ------------------------------------------------------*/


int inv_easy_device_init(inv_easy_device_settings_t *psettings, uint8_t *pwhoami, inv_fw_version_t * pfw_version)
{
	int rc = 0;
	
	/*
	 * Open serial (SPI or I2C) interface before using the device
	 */
	INV_MSG(INV_MSG_LEVEL_INFO, "Easy Device : Open serial interface");
	psettings->pserif = inv_arduino_emd_adapter_get_instance();
	rc = inv_host_serif_open(psettings->pserif);
	
	if(rc)
	{
		INV_MSG(INV_MSG_LEVEL_ERROR, "Easy Device : Unexpected error on serif open. Aborting");
		return(rc);
	}
	
	/*
	 * Create Icm30630 Device 
	 * Pass to the driver :
	 * - reference to serial interface object,
	 * - reference to listener that will catch sensor events,
	 * - a static buffer for the driver to use as a temporary buffer
	 * - various driver option
	 */
	INV_MSG(INV_MSG_LEVEL_INFO, "Easy Device : Init device");
	
	inv_device_icm30630_init(&psettings->icm30xxx, psettings->pserif, &psettings->sensor_listener,
	     psettings->buffer, psettings->buffer_size, true /* enable device debug interface (SWD and JTAG) */);
	
	/*
	 * Register interrupt callback
	 */
	INV_MSG(INV_MSG_LEVEL_INFO, "Easy Device : Register interrupt callback");
	rc = inv_device_icm30xxx_register_interrupt_callback(&psettings->icm30xxx, psettings->interrupt_cb, psettings->context);
		 
	if(rc)
	{
		INV_MSG(INV_MSG_LEVEL_ERROR, "Easy Device : Unexpected error on register interrupt callback");
		return(rc);
	}
	
	/*
	 * Simply get generic device handle from Icm30xxx Device
	 */
	psettings->device = inv_device_icm30xxx_get_base(&psettings->icm30xxx);
	
	/*
	 * Get the whoami
	 */
	INV_MSG(INV_MSG_LEVEL_INFO, "Easy Device : Read who am i");
	
	rc = inv_device_whoami(psettings->device, pwhoami);
	
	if(rc)
	{
		INV_MSG(INV_MSG_LEVEL_ERROR, "Easy Device : Unexpected error on read whoami. Aborting");
		return(rc);
	}
	
	/*
	 * Flash device FW (FW must be flashed before calling setup())
	 * This step is not mandatory if device was already flashed
	 */
	if(psettings->fw_image_buffer != NULL)
	{
		INV_MSG(INV_MSG_LEVEL_INFO, "Easy Device : Load M0 image");
		
		/*
		 * "verify" and "force" call parameters are set for programming the M0 image only if the "fw_image_buffer"
		 * is different from the M0 already flashed in the device
		 */
		rc = inv_device_load(psettings->device, INV_DEVICE_ICM30XXX_IMAGE_TYPE_M0_FLASH, psettings->fw_image_buffer,
				psettings->fw_image_buffer_size, true  /* verify */, false /* force */);
		
		if(rc)
		{
			INV_MSG(INV_MSG_LEVEL_ERROR, "Easy Device : Unexpected error on Load M0 image. Aborting");
			return(rc);
		}
	}
	else
	{
		INV_MSG(INV_MSG_LEVEL_INFO, "Easy Device : Skip Load M0 image");
	}
	
	/*
	 * Now that FW is loaded, configure and initialize the icm30xxx device
	 */
	INV_MSG(INV_MSG_LEVEL_INFO, "Easy Device : Device setup");
	
	rc = inv_device_setup(psettings->device);
	
	if(rc)
	{
		INV_MSG(INV_MSG_LEVEL_ERROR, "Easy Device : Unexpected error on Setup. Aborting");
		return(rc);
	}
	
	/*
	 * Now that device was inialized, we can proceed with DMP image loading
	 * This step is mandatory as DMP image are not store in non volatile memory
	 */
	INV_MSG(INV_MSG_LEVEL_INFO, "Easy Device : Load DMP3 image");
	
	rc = inv_device_load(psettings->device, INV_DEVICE_ICM30XXX_IMAGE_TYPE_DMP3, psettings->dmp3_image_buffer,
			psettings->dmp3_image_buffer_size, true, false);
	
	if(rc)
	{
		INV_MSG(INV_MSG_LEVEL_ERROR, "Easy Device : Unexpected error on DMP3 image load. Aborting");
		return(rc);
	}
	
	INV_MSG(INV_MSG_LEVEL_INFO, "Easy Device : Load DMP4 image");
	
	rc = inv_device_load(psettings->device, INV_DEVICE_ICM30XXX_IMAGE_TYPE_DMP4, psettings->dmp4_image_buffer,
			psettings->dmp4_image_buffer_size, true, false);
	
	if(rc)
	{
		INV_MSG(INV_MSG_LEVEL_ERROR, "Easy Device : Unexpected error on DMP4 image load. Aborting");
		return(rc);
	}
	
	/*
	 * Get device FW the version
	 */
	INV_MSG(INV_MSG_LEVEL_INFO, "Easy Device : Get Fw Version");
	rc = inv_device_get_fw_info(psettings->device, pfw_version);
	
	if(rc)
	{
		INV_MSG(INV_MSG_LEVEL_ERROR, "Easy Device : Unexpected error on get Fw version load. Aborting");
		return(rc);
	}
	
	// Display M0 Fw version
	INV_MSG(INV_MSG_LEVEL_INFO, "Easy Device : Fw Version %d.%d.%d %s", pfw_version->major, pfw_version->minor, pfw_version->patch, pfw_version->suffix);
	
	// configure mounting matrix for accelerometer and gyrometer
	// setting mounting matrix for one sensor will be also apply to the other because they are mechanically linked
	rc = inv_device_set_sensor_mounting_matrix(psettings->device, INV_SENSOR_TYPE_ACCELEROMETER, psettings->acc_gyr_mounting_matrix);
	if(rc)
	{
		INV_MSG(INV_MSG_LEVEL_ERROR, "Easy Device : fail to set mounting matrix for acc/gyr");
		return(rc);
	}
	
	// configure mouting matrix for magnetometer to be aligned with the accelerometer/gyrometer from the ICM-30630
	// this is mandatory to run fusion algorithms based on both magnetometer and gyrometer/accelerometer (INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR or INV_SENSOR_TYPE_ROTATION_VECTOR)
	rc = inv_device_set_sensor_mounting_matrix(psettings->device, INV_SENSOR_TYPE_MAGNETOMETER, psettings->mag_mounting_matrix);
	if(rc)
	{
		INV_MSG(INV_MSG_LEVEL_ERROR, "Easy Device : fail to set mounting matrix for mag");
		return(rc);
	}
	
	return(rc);
}


int inv_easy_device_start_sensor(inv_easy_device_settings_t *psettings, int sensor, uint32_t period, uint32_t timeout)
{
	int rc = 0;
	
	// Set sensor period
	INV_MSG(INV_MSG_LEVEL_INFO, "Easy Device : Set sensor period");
	rc = inv_device_set_sensor_period(psettings->device, sensor, period);
	
	if(rc)
	{
		INV_MSG(INV_MSG_LEVEL_ERROR, "Easy Device : Unexpected error on set sensor period. Aborting");
		return(rc);
	}
	
	// Set sensor buffering timeout
	INV_MSG(INV_MSG_LEVEL_INFO, "Easy Device : Set sensor timeout");
	rc = inv_device_set_sensor_timeout(psettings->device, sensor, timeout);
	
	if(rc)
	{
		INV_MSG(INV_MSG_LEVEL_ERROR, "Easy Device : Unexpected error on set sensor timeout. Aborting");
		return(rc);
	}
	
	// Start sensor
	INV_MSG(INV_MSG_LEVEL_INFO, "Easy Device : Start sensor");
	rc = inv_device_start_sensor(psettings->device, sensor);
	
	if(rc)
	{
		INV_MSG(INV_MSG_LEVEL_ERROR, "Easy Device : Unexpected error on start sensor. Aborting");
		return(rc);
	}
	
	return(rc);
}


int inv_easy_device_stop_sensor(inv_easy_device_settings_t *psettings, int sensor)
{
	int rc = 0;
	
	// Stop sensor
	INV_MSG(INV_MSG_LEVEL_INFO, "Easy Device : Stop sensor");
	rc = inv_device_stop_sensor(psettings->device, sensor);
	
	if(rc)
	{
		INV_MSG(INV_MSG_LEVEL_ERROR, "Easy Device : Unexpected error on stop sensor. Aborting");
		return(rc);
	}
	
	return(rc);
}


int inv_easy_device_close(inv_easy_device_settings_t * psettings)
{
	int rc;
	
	/*
	 * Shutdown everything.
	 */
	INV_MSG(INV_MSG_LEVEL_INFO, "Easy Device : Device Cleanup");
	rc = inv_device_cleanup(psettings->device);
	
	if(rc)
	{
		INV_MSG(INV_MSG_LEVEL_ERROR, "Easy Device : Unexpected error cleanup. Aborting");
		return(rc);
	}
	
	/*
	 * Close serial interface link
	 */
	INV_MSG(INV_MSG_LEVEL_INFO, "Easy Device : Serif close");
	rc = inv_host_serif_close(psettings->pserif);
	
	
	if(rc)
	{
		INV_MSG(INV_MSG_LEVEL_ERROR, "Easy Device : Unexpected error on serif close. Aborting");
		return(rc);
	}
	
	return(rc);
}
