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

/** @defgroup SensorTypes Sensor types
 *  @brief    Sensor related types definitions
 *  @{
 */

#ifndef _INV_SENSOR_TYPES_H_
#define _INV_SENSOR_TYPES_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "Invn/InvBool.h"

/** @brief Sensor type identifier definition
 */
enum inv_sensor_type {
	INV_SENSOR_TYPE_META_DATA                    = 0 ,  /**< Meta-data */
	INV_SENSOR_TYPE_ACCELEROMETER                = 1 ,  /**< Accelerometer */
	INV_SENSOR_TYPE_MAGNETOMETER                 = 2 ,  /**< Magnetic field */
	INV_SENSOR_TYPE_ORIENTATION                  = 3 ,  /**< Deprecated orientation */
	INV_SENSOR_TYPE_GYROMETER                    = 4 ,  /**< Gyroscope */
	INV_SENSOR_TYPE_LIGHT                        = 5 ,  /**< Ambient light sensor */
	INV_SENSOR_TYPE_PRESSURE                     = 6 ,  /**< Barometer */
	INV_SENSOR_TYPE_TEMPERATURE                  = 7 ,  /**< Temperature */
	INV_SENSOR_TYPE_PROXIMITY                    = 8 ,  /**< Proximity */
	INV_SENSOR_TYPE_GRAVITY                      = 9 ,  /**< Gravity */
	INV_SENSOR_TYPE_LINEAR_ACCELERATION          = 10,  /**< Linear acceleration */
	INV_SENSOR_TYPE_ROTATION_VECTOR              = 11,  /**< Rotation vector */
	INV_SENSOR_TYPE_HUMIDITY                     = 12,  /**< Relative humidity */
	INV_SENSOR_TYPE_AMBIENT_TEMPERATURE          = 13,  /**< Ambient temperature */
	INV_SENSOR_TYPE_UNCAL_MAGNETOMETER           = 14,  /**< Uncalibrated magnetic field */
	INV_SENSOR_TYPE_GAME_ROTATION_VECTOR         = 15,  /**< Game rotation vector */
	INV_SENSOR_TYPE_UNCAL_GYROMETER              = 16,  /**< Uncalibrated gyroscope */
	INV_SENSOR_TYPE_SMD                          = 17,  /**< Significant motion detection */
	INV_SENSOR_TYPE_STEP_DETECTOR                = 18,  /**< Step detector */
	INV_SENSOR_TYPE_STEP_COUNTER                 = 19,  /**< Step counter */
	INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR       = 20,  /**< Geomagnetic rotation vector */
	INV_SENSOR_TYPE_HEART_RATE                   = 21,  /**< Heart rate */
	INV_SENSOR_TYPE_TILT_DETECTOR                = 22,  /**< Tilt detector */
	INV_SENSOR_TYPE_WAKE_GESTURE                 = 23,  /**< Wake-up gesture  */
	INV_SENSOR_TYPE_GLANCE_GESTURE               = 24,  /**< Glance gesture  */
	INV_SENSOR_TYPE_PICK_UP_GESTURE              = 25,  /**< Pick-up gesture */
	INV_SENSOR_TYPE_BAC                          = 26,  /**< Basic Activity Classifier */
	INV_SENSOR_TYPE_PDR                          = 27,  /**< Pedestrian Dead Reckoning */
	INV_SENSOR_TYPE_B2S                          = 28,  /**< Bring to see */
	INV_SENSOR_TYPE_3AXIS                        = 29,  /**< 3 Axis sensor */

	INV_SENSOR_TYPE_CUSTOM0,                            /**< Custom sensor ID 0 */
	INV_SENSOR_TYPE_CUSTOM1,                            /**< Custom sensor ID 1 */
	INV_SENSOR_TYPE_CUSTOM2,                            /**< Custom sensor ID 2 */
	INV_SENSOR_TYPE_CUSTOM3,                            /**< Custom sensor ID 3 */
	INV_SENSOR_TYPE_CUSTOM4,                            /**< Custom sensor ID 4 */
	INV_SENSOR_TYPE_CUSTOM5,                            /**< Custom sensor ID 5 */
	INV_SENSOR_TYPE_CUSTOM6,                            /**< Custom sensor ID 6 */
	INV_SENSOR_TYPE_CUSTOM7,                            /**< Custom sensor ID 7 */

	INV_SENSOR_TYPE_MAX                                 /**< sentinel value for sensor type */
};

#define INV_SENSOR_TYPE_CUSTOM_BASE    INV_SENSOR_TYPE_CUSTOM0
#define INV_SENSOR_TYPE_CUSTOM_END     (INV_SENSOR_TYPE_CUSTOM7+1)

/** @brief Helper flag to indicate if sensor is a Wale-Up sensor
 */
#define INV_SENSOR_TYPE_WU_FLAG        (unsigned int)(0x80000000)

/** @brief Sensor status definition
 */
enum inv_sensor_status
{
	INV_SENSOR_STATUS_DATA_UPDATED      = 0,    /**< new sensor data */
	INV_SENSOR_STATUS_STATE_CHANGED     = 1,	/**< dummy sensor data indicating
                                                     to a change in sensor state */
	INV_SENSOR_STATUS_FLUSH_COMPLETE    = 2,    /**< dummy sensor data indicating
                                                     a end of batch after a manual flush */
	INV_SENSOR_STATUS_POLLED_DATA       = 3,    /**< sensor data value after manual request */
};

/** @brief Event definition for BAC sensor
 */
enum inv_sensor_bac_event {
	INV_SENSOR_BAC_EVENT_ACT_UNKNOWN             =  0,
	INV_SENSOR_BAC_EVENT_ACT_IN_VEHICLE_BEGIN    =  1,
	INV_SENSOR_BAC_EVENT_ACT_IN_VEHICLE_END      = -1,
	INV_SENSOR_BAC_EVENT_ACT_WALKING_BEGIN 	     =  2,
	INV_SENSOR_BAC_EVENT_ACT_WALKING_END         = -2,
	INV_SENSOR_BAC_EVENT_ACT_RUNNING_BEGIN       =  3,
	INV_SENSOR_BAC_EVENT_ACT_RUNNING_END         = -3,
	INV_SENSOR_BAC_EVENT_ACT_ON_BICYCLE_BEGIN    =  4,
	INV_SENSOR_BAC_EVENT_ACT_ON_BICYCLE_END      = -4,
	INV_SENSOR_BAC_EVENT_ACT_TILT_BEGIN          =  5,
	INV_SENSOR_BAC_EVENT_ACT_TILT_END            = -5,
	INV_SENSOR_BAC_EVENT_ACT_STILL_BEGIN         =  6,
	INV_SENSOR_BAC_EVENT_ACT_STILL_END           = -6,
};

/** @brief Maximum size of an event data
 */
#define IVN_SENSOR_EVENT_DATA_SIZE      64

/** @brief Sensor event definition
 */
typedef struct inv_sensor_event
{
	unsigned int         sensor;           /**< sensor type */
	int                  status;           /**< sensor data status as of
	                                            enum inv_sensor_status */
	uint64_t             timestamp;        /**< sensor data timestamp in us */
	union {
		struct {
			float        vect[3];          /**< x,y,z vector data */
			uint8_t      accuracy_flag;    /**< accuracy flag */
		} acc;                             /**< 3d accelerometer data in g */
		struct {
			float        vect[3];          /**< x,y,z vector data */
			float        bias[3];          /**< x,y,z bias vector data (for uncal sensor variant) */
			uint8_t      accuracy_flag;    /**< accuracy flag */
		} mag;                             /**< 3d magnetometer data in uT */
		struct {
			float        vect[3];          /**< x,y,z vector data */
			float        bias[3];          /**< x,y,z bias vector data (for uncal sensor variant) */
			uint8_t      accuracy_flag;    /**< accuracy flag */
		} gyr;                             /**< 3d gyrometer data in deg/s */
		struct {
			float        quat[4];          /**< w,x,y,z quaternion data */
			float        accuracy;         /**< heading accuracy in deg */
		} quaternion;                      /**< quaternion data */
		struct {
			float        x,y,z;            /**< x,y,z angles in deg as defined by Google Orientation sensor */
			uint8_t      accuracy_flag;    /**< heading accuracy in deg */
		} orientation;                     /**< oentation data */
		struct {
			float        bpm;              /**< beat per minute */
		} hrm;                             /**< heart rate monitor data */			
		struct {
			float        tmp;              /**< temperature in deg celcius */
		} temperature;                     /**< temperature data */
		struct {
			float        percent;          /**< relative humidity in % */
		} humidity;                        /**< humidity data */
		struct {
			uint64_t     count;             /**< number of steps */
		} step;                            /**< step-counter data */
		struct {
			uint32_t     level;            /**< light level in lux */
		} light;                           /**< light data */
		struct {
			uint32_t     distance;          /**< distance in mm */
		} proximity;                        /**< proximity data */
		struct {
			uint32_t     pressure;         /**< pressure in Pa */
		} pressure;                        /**< pressure data */
		struct {
			int          event;            /**< BAC data begin/end event as of 
			                                    enum inv_sensor_bac_event */
		} bac;                             /**< BAC data */
		struct {
			uint32_t     fxdata[12];       /**< PDR data in fixpoint*/
		} pdr;                             /**< PDR data */
		inv_bool_t       event;            /**< event state for gesture-like sensor (SMD, step-detector, ...) */
		uint8_t          reserved[IVN_SENSOR_EVENT_DATA_SIZE];     /**< reserved sensor data for future sensor */
	} data;                                /**< sensor data */
} inv_sensor_event_t;


/** @brief Sensor listener event callback definition
 *  @param[in] event     reference to sensor event
 *  @param[in] context   listener context
 *  @return    none
 */
typedef void (*inv_sensor_listener_event_cb_t)(const inv_sensor_event_t * event,
		void * context);

/** @brief Sensor event listener definition
 */
typedef struct inv_sensor_listener {
	inv_sensor_listener_event_cb_t event_cb; /**< sensor event callback */
	void *                         context;  /**< listener context */
} inv_sensor_listener_t;

/** @brief Helper to initialize a listener object
 */
static inline void inv_sensor_listener_init(inv_sensor_listener_t * listener,
	inv_sensor_listener_event_cb_t event_cb, void * context)
{
	listener->event_cb = event_cb;
	listener->context  = context;
}

/** @brief Helper to notify a listener of a new sensor event
 */
static inline void inv_sensor_listener_notify(const inv_sensor_listener_t * listener,
		const inv_sensor_event_t * event)
{
	if(listener) {
		listener->event_cb(event, listener->context);
	}
}

/** @brief Helper macro to retrieve sensor type (without wake-up flag) from a sensor id.
 */
#define INV_SENSOR_ID_TO_TYPE(sensor) \
	((unsigned int)(sensor) & ~INV_SENSOR_TYPE_WU_FLAG)

/** @brief Helper macro that check if given sensor is of known type
 */
#define INV_SENSOR_IS_VALID(sensor) \
	(INV_SENSOR_ID_TO_TYPE(sensor) < INV_SENSOR_TYPE_MAX)

/** @brief Helper macro that check if given sensor is a wake-up sensor
 */
#define INV_SENSOR_IS_WU(sensor) \
 	(((int)(sensor) & INV_SENSOR_TYPE_WU_FLAG) != 0)

/** @brief Utility function that returns a string from a sensor id
 *  Empty string is returned if sensor is invalid
 */
const char * inv_sensor_str(int sensor);

/** @brief Alias for inv_sensor_str
 */
#define inv_sensor_2str 	inv_sensor_str

#ifdef __cplusplus
}
#endif

#endif /* _INV_SENSOR_TYPES_H_ */

/** @} */
