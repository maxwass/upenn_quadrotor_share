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
 * @brief This sketch print on the serial interface statistics on accelerometer and gryometer 
 *        data reporting at 1kHz read from device until a char is received on the serial interface.
 *        You can restart the sensor by sending a new char on the serial interface
 *        Only available for ICM30670 device!
 */

// Be sure that you choose Arduino Zero board in Tools => Board
#ifndef ARDUINO_SAMD_ZERO
#  error "You must choose Arduino Zero in IDE Tools => Board"
#endif

// We can not processs warning as error on whole code since Arduino libraries
// has warnings in -Wall
#pragma GCC diagnostic push
#pragma GCC diagnostic warning "-Werror"

// Required for spi use in library
#include <SPI.h>
#include "ArduinoEmdAdapter.h"
#include "DeviceIoHal.h"
#include "Invn/Devices/DeviceIcm30xxx.h"
#include "Invn/Utils/Message.h"
#include <stdarg.h>
#include "EasyDevice.h"

/* Private define ------------------------------------------------------------*/

/**
 * Define msg level
 */
#define MSG_LEVEL INV_MSG_LEVEL_WARNING

/**
 * Comment this line to avoid ICM-M0 fw programming
 */
#define ENABLE_FW_M0_PROG

/**
 * Define where traces are transmitted
 */
#define SERIAL_TRACES Serial

/**
 * Max string size for trace display
 */
#define MAX_TRACES_STRING_SIZE   (200)

/**
 * Macro for testing return code and return from function with a FAIL message
 */
#define TEST_RC(rc) if(rc) \
			{ \
				printTraces("FAIL"); \
				return rc; \
			}

#define DELTA_TIMESTAMP_MAX   1000

/* Public function prototypes ------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/**
 * Some memory to be used by the Icm30xxx driver (must be >= 4k)
 */
static uint8_t device_buffer[1024*4]; 

static sensor_event_stats_t  sRawGyrEvent;
static sensor_event_stats_t  sRawAccEvent;
static sensor_event_stats_t  sGrvEvent;
/**
 * Flag = true if a gpio interrupt was asserted
 */
static bool data_int_occured = false;

/**
 * Flag = true if a timer overflow interrupt was asserted
 */
static bool watchdog_int_occured = false;

static int32_t event_count = 0;
 
/*
 * The device requires a FW and DMP images to be loaded on init
 * Images will be provided as a byte array
 */

/**
 * M0 image
 */
#ifdef ENABLE_FW_M0_PROG
static const uint8_t flash_image[] = {
		#include "Images/icm30670_img.flash.m0.h"
		};
#endif

/**
 * DMP3 image
 */
static const uint8_t dmp3_image[] = { 0 };

/**
 * DMP4 image
 */
static const uint8_t dmp4_image[] = {
		#include "Images/icm30670_img.sram.dmp4.h"
		};


/* Private function code ------------------------------------------------------*/

/** @brief Display a message (through means of printer function)
 *  Called from INV_MSG macro defined in Utils/Message.h
 *  @param[in]  level  level for the message
 *  @param[in]  str    message string
 *  @param[in]  ap     optional arguments
 *  @return none
 */
 void inv_msg_printer_arduino(int level, const char * str, va_list ap)
 {
	char String[MAX_TRACES_STRING_SIZE];

	const char * s[INV_MSG_LEVEL_MAX] = {
			"",    // INV_MSG_LEVEL_OFF
			" [E] ", // INV_MSG_LEVEL_ERROR
			" [W] ", // INV_MSG_LEVEL_WARNING
			" [I] ", // INV_MSG_LEVEL_INFO
			" [V] ", // INV_MSG_LEVEL_VERBOSE
			" [D] ", // INV_MSG_LEVEL_DEBUG
			};

	// Print the traces
	SERIAL_TRACES.print(s[level]);

	// Print format in a string
	vsnprintf(String, MAX_TRACES_STRING_SIZE, str, ap);
	String[MAX_TRACES_STRING_SIZE-1] = '\0'; // Avoid overflow if string is >= MAX_TRACES_STRING_SIZE char

	// print string
	SERIAL_TRACES.println(String);
 }


/** @brief Display a trace from sketch, manage warning as a gcc printf
 *  Data are printed on the Serial defined by SERIAL_TRACES
 *  @param[in]  str   message string
 *  @param[in]  ...   optional arguments
 *  @return none
 */
void printTraces(const char * str, ...) __attribute__ ((format (printf, 1, 2)));
void printTraces(const char * str, ...)
{
	char String[MAX_TRACES_STRING_SIZE];

	// Print a sketch
	SERIAL_TRACES.print("[SKETCH] ");

	// Print the traces
	va_list ap;
	va_start(ap, str);
	vsnprintf(String, MAX_TRACES_STRING_SIZE, str, ap);
	va_end(ap);
	String[MAX_TRACES_STRING_SIZE-1] = '\0'; // Avoid overflow if string is >= MAX_TRACES_STRING_SIZE char

	// Print string
	SERIAL_TRACES.println(String);
}

/** 
 * systick get implementation
 */
uint32_t inv_systick_ms_get(void)
{
	return( millis());
}

/** @brief Convert timestamp 64 bit in us to day, hour, min, sec, ms, us
 *  @param[in]  timestamp timestamp 64 bit in us
 *  @param[out] day       day
 *  @param[out] hr        hour
 *  @param[out] min       minute
 *  @param[out] sec       second
 *  @param[out] ms        milli-second
 *  @param[out] us        micro-second
 *  @return none
 */
void conv_timestamp_to_time(uint64_t timestamp, uint32_t *day, uint8_t *hr, uint8_t *min, uint8_t *sec, uint16_t *ms,  uint16_t *us)
{
  // uint64_t max value is +18,446,744,073,709,551,615
  // convert timestamp
  *us = (timestamp % 1000);
  timestamp /= 1000;
  
  *ms = (timestamp % 1000);
  timestamp /= 1000;
  
  *sec = (timestamp % 60);
  timestamp /= 60;
  
  *min = (timestamp % 60);
  timestamp /= 60;
  
  *hr = (timestamp % 60);
  timestamp /= 60;
  
  *day = timestamp;
}

void compute_stats(const inv_sensor_event_t * event, sensor_event_stats_t * sensor)
{
  uint32_t day;
  uint8_t  hr, min, sec;
  uint16_t ms, us;
  char     string[MAX_TRACES_STRING_SIZE];
  
	if(event->status == INV_SENSOR_STATUS_DATA_UPDATED) {
		if(sensor->count != 0) {
			sensor->dt = event->timestamp - sensor->last_timestamp;
			sensor->sum += sensor->dt;

			if (sensor->dt < sensor->min_dt || sensor->count == 1)
				sensor->min_dt = sensor->dt;

			if (sensor->dt > sensor->max_dt || sensor->count == 1)
				sensor->max_dt = sensor->dt;
/*
        // Increase event counter
        event_count++;
        
        conv_timestamp_to_time(event->timestamp, &day, &hr, &min, &sec, &ms, &us);
        
        //
        // Display sensor info. printf in Arduino doesn't support %f so we can not use trace function
        //
        SERIAL_TRACES.print("[SKETCH] ");
        snprintf(string, MAX_TRACES_STRING_SIZE, "%lud %02dh%02dm%02ds.%03hums.%03huus : evt cnt : %10u" , day, hr, min, sec, ms, us, event_count);
        string[MAX_TRACES_STRING_SIZE-1] = '\0';
        SERIAL_TRACES.print(string);
        
        SERIAL_TRACES.print(" : W = ");
        SERIAL_TRACES.print(event->data.quaternion.quat[0], 6);
        SERIAL_TRACES.print(" : X = ");
        SERIAL_TRACES.print(event->data.quaternion.quat[1], 6);
        SERIAL_TRACES.print(" : Y = ");
        SERIAL_TRACES.print(event->data.quaternion.quat[2], 6);
        SERIAL_TRACES.print(" : Z = ");
        SERIAL_TRACES.print(event->data.quaternion.quat[3], 6);
        SERIAL_TRACES.println("");
        */
		}

		sensor->last_timestamp = event->timestamp;
		sensor->count++;
	}
}


/** @brief Sensor listener event callback definition
 *  @param[in] event     reference to sensor event
 *  @param[in] arg       listener context
 *  @return    none
 */
void sensor_event_cb(const inv_sensor_event_t * event, void * arg)
{
	switch(event->sensor) 
	{

	case INV_SENSOR_TYPE_RAW_GYROSCOPE:
		compute_stats(event, &sRawGyrEvent);
		break;

  case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
    compute_stats(event, &sRawAccEvent);
    break;

  case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
    compute_stats(event, &sGrvEvent);
    break;

	default:
		printTraces("UNEXPECTED SENSOR EVENT %d", event->sensor);
		break;
	}

	// Avoid a warning
	(void) arg; // We don't need this arg
}


/** @brief Callback Registered to the adapter
 *  WARNING : CALLED UNDER IRQ ! CODE INSIDE MUST BE THREAD SAFE !
 * @param[in] context         context passed to callback
 * @param[in] int_num         interrupt number
 * @return none
 */
void device_interrupt_cb(void * context, int int_num)
{
	switch(int_num)
	{
	case INV_DEVICE_IO_INTERRUPT:
		// Set data interrupt flag
		data_int_occured = true;
		break;
	case INV_DEVICE_TIMER_0_INTERRUPT:
		// Set watchdog interrupt flag
		watchdog_int_occured = true;
		break;
	default:
		break;
	}

	// Avoid warning. We don't need this parameter for our function
	(void) context;
}


// We don't diagnostic "missing-field-initializer" for "device_settings" struct
// There are too much fields to initialize by lib and it's a nonsense to
// initialize all values in variable declaration
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"

/**
 * Handle settings for easy device
 */
inv_easy_device_settings_t device_settings = 
{
	.interrupt_cb = device_interrupt_cb,
	.context      = NULL,
	.device       = NULL,
	.pserif       = NULL,
	.buffer       = device_buffer,
	.buffer_size  = sizeof(device_buffer),
	.icm30xxx       = {0},
	.sensor_listener = 
	{
	sensor_event_cb, /* callback that will receive sensor events */
	(void *)0xDEAD   /* some pointer passed to the callback */
	},
#ifdef ENABLE_FW_M0_PROG
	.fw_image_buffer        = flash_image,
	.fw_image_buffer_size   = sizeof(flash_image),
#else 
	.fw_image_buffer        = NULL,
	.fw_image_buffer_size   = 0,
#endif
	.dmp3_image_buffer      = dmp3_image,
	.dmp3_image_buffer_size = sizeof(dmp3_image),
	.dmp4_image_buffer      = dmp4_image,        
	.dmp4_image_buffer_size = sizeof(dmp4_image),

	.acc_gyr_mounting_matrix = {1.0, 0.0, 0.0,
	                          0.0, 1.0, 0.0,
	                          0.0, 0.0, 1.0},
	// Align mag axis with accel and gyro
	// Update this matrix if you mount a magnetometer with a different axis referential
	.mag_mounting_matrix     = {0.0, -1.0, 0.0,
	                          1.0, 0.0, 0.0,
	                          0.0, 0.0, 1.0},
};

#pragma GCC diagnostic pop // Now "missing-field-initializer" is tested

/**
 * Utils to reset sensor event structure
 */
void sensor_event_reset(sensor_event_stats_t * sensor)
{
	sensor->count = 0;
	sensor->last_timestamp = 0;
	sensor->dt = 0;
	sensor->sum = 0;
	sensor->min_dt = 0;
	sensor->max_dt = 0;
}


void sensor_event_print_stats(sensor_event_stats_t * eventStats, const char* sensorName)
{
	printTraces("%s : %ld event received | dt (Average [Min/Max]) = %ld usec [%ld usec / %ld usec]", 
			sensorName,
			(uint32_t) eventStats->count,
			(uint32_t) eventStats->sum/eventStats->count,
			(uint32_t) eventStats->min_dt,
			(uint32_t) eventStats->max_dt
			);

	if (eventStats->max_dt > 1000 * 1.1)
		printTraces("Error detected: Potential loss of event happened");
}

/**
 * Utils to compute sensor event statistics
 */
int sensor_event_stats()
{
	sensor_event_print_stats(&sRawGyrEvent, "Raw Gyr");
	sensor_event_print_stats(&sRawAccEvent, "Raw Acc");
  sensor_event_print_stats(&sGrvEvent, "GRV");

	return 0;
}

/**
 * Sanity test
 */
int high_ODR_rate(void)
{
	uint32_t timeout;
	int rc;

	// Reset sensors structure
	sensor_event_reset(&sRawGyrEvent);
	sensor_event_reset(&sRawAccEvent);

	printTraces("Starting sensors at 1kHz");

	rc = inv_easy_device_start_sensor(&device_settings, INV_SENSOR_TYPE_RAW_GYROSCOPE, 1, 0);
  rc = inv_easy_device_start_sensor(&device_settings, INV_SENSOR_TYPE_RAW_ACCELEROMETER, 1, 0);
  rc = inv_easy_device_start_sensor(&device_settings, INV_SENSOR_TYPE_GAME_ROTATION_VECTOR, 1, 0);
  
	TEST_RC(rc);

	do {
		// Compute time out => return timestamp in mS
		timeout = inv_systick_ms_get() + 5000;
			do {
				// Now, test if interrupt occured
				if(data_int_occured != false) {
					// Reset interrupt flag
					data_int_occured = false;

					// Interrupt occurs poll device
					rc = inv_device_poll(device_settings.device);

					if(rc != 0)
						printTraces("Error in polling : %d", rc);
				}
    
				// Test if watchdog needs to be polled
				if(watchdog_int_occured != false) {
					// Reset interrupt flag
					watchdog_int_occured = false;

					// Check that device firmware is running properly
					if(inv_device_icm30670_watchdog_poll(&device_settings.icm30xxx) == INV_ERROR_WATCHDOG) {
						INV_MSG(INV_MSG_LEVEL_ERROR, "Watchdog error, device is in reset state");
						// Trap watchdog error
						while(1);
					}
				}
			} while (inv_systick_ms_get() < timeout);

			rc = sensor_event_stats();

			// Reset sensors structure
			sensor_event_reset(&sRawGyrEvent);
			sensor_event_reset(&sRawAccEvent);
      sensor_event_reset(&sGrvEvent);
		} while (rc == 0);

	rc = stop_sensor();
	TEST_RC(rc);

	return -1;
}

/** @brief Stop sensor
 *  @return return code of driver function
 */
int stop_sensor(void)
{
	int rc;
	uint8_t i;

	// Stop gyro
	rc = inv_easy_device_stop_sensor(&device_settings, INV_SENSOR_TYPE_RAW_GYROSCOPE);
  rc = inv_easy_device_stop_sensor(&device_settings, INV_SENSOR_TYPE_RAW_ACCELEROMETER);
  rc = inv_easy_device_stop_sensor(&device_settings, INV_SENSOR_TYPE_GAME_ROTATION_VECTOR);
	TEST_RC(rc);


	printTraces("Sensor Gyroscope STOPPED !");
	// wait 50 ms for all data to be outputed
	for (i = 0; i < 10; i++) {
		rc = inv_device_poll(device_settings.device);
		inv_icm30xxx_sleep(5);
	}

	return rc;
}


/** @brief Init sensor and start Gyroscope
 *  @return Last return code of last driver function called
 */
int initSketch(void)
{
	int rc;
	uint8_t whoami;
	inv_fw_version_t fw_version;

	// Setup driver messages if you want to see device driver traces
	printTraces("Setup msg level as warning");
	inv_msg_setup(MSG_LEVEL, inv_msg_printer_arduino);

	// Device easy init
	printTraces("Easy device init");
	rc = inv_easy_device_init(&device_settings, &whoami, &fw_version);
	TEST_RC(rc);

	// Test whoami
	if((whoami != 0xC1) && (whoami != 0xC2)) {
		// who am i incorrect
		rc = INV_ERROR_UNEXPECTED;
		printTraces("FAIL : Unexpected device who am i %x", whoami);
		return rc;
	} 
	
	do {
		rc = high_ODR_rate();
		TEST_RC(rc);
	} while (rc == 0);

	return(rc);
}


/* Public function code ------------------------------------------------------*/


/** @brief Sleep implementation for Icm30xxx.
 *  @param[in]  ms  millisecond  of sleep
 *  @return     none
 */
void inv_icm30xxx_sleep(int ms)
{
	// You must provide a sleep function that blocks the current programm
	// execution for the specified amount of ms
	delay(ms);
}

/** @brief Arduino sketch setup function
 *  Called at init by Arduino Sketch
 *  @return     none
 */
void setup() 
{
	// Open native serial port through USB programming port (for debug purpose)
	SERIAL_TRACES.begin(250000); // Max speed for arduino monitor

	// Wait before starting example
	while(! SERIAL_TRACES); // Wait only on open of the native port

	// Init sketch
	printTraces("Sketch start");
	initSketch();
}

/** @brief Arduino sketch loop function
 *  called in loop by Arduino sketch
 *  @return     never
 */
void loop()
{
	// Test if there is any char in buffer
	if(SERIAL_TRACES.available() != 0) {
		// Flush serial buffer
		while(SERIAL_TRACES.available() != 0) {
			SERIAL_TRACES.read();
		}

		// Display restart info
		printTraces("Send a char for restarting sensor");
		while(SERIAL_TRACES.available() == 0);

		// Wait all incoming char
		delay(100); 

		// Flush serial buffer
		while(SERIAL_TRACES.available() != 0) {
			SERIAL_TRACES.read();
		}

		high_ODR_rate();
	}
}
