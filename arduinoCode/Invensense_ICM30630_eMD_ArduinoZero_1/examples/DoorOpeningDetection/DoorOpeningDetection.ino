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

/******************************************************************************

 ******************************************************************************/

/**
 * @file
 * @brief This sketch tones a buzzer and active LED_L for 5s when an event from door opening
 *        custom sensor
 *
 *        WARNING 1 : you must flash ICM-30630 with DoorOpeningDetection code sample included
 *                    in studio
 *
 *        WARNING 2 : you must use the calibration sketch before using this one in 
 *                    order to have a correct orientation. The bias computed are displayed in
 *                    the Serial monitor window. Copy/paste the value in this sketch
 *                    See bias table below in this sketch
 */

// Be sure that you choose Arduino Zero board in Tools => Board
#ifndef ARDUINO_SAMD_ZERO
#	error "You must choose Arduino Zero in IDE Tools => Board"
#endif

// We can not processs warning as error on whole code since Arduino libraries
// has warnings in -Wall
#pragma GCC diagnostic push
#pragma GCC diagnostic warning "-Werror"

// Required for spi use in library
#include <SPI.h>
#include "ArduinoEmdAdapter.h"
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

/**
 * Door opening detection custom sensor ID
 */ 
#define DOOR_OPENING_CUSTOM_SENSOR_ID             (INV_SENSOR_TYPE_CUSTOM0)

/**
 * Detection Led pin 
 */ 
#define DETECTION_LED_PIN                         (13)

/* Public function prototypes ------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/******************************************************************************
 * WARNING : you must use the calibration sketch before using this one in 
 * order to have a correct orientation. The bias computed are displayed in
 * the Serial monitor window. Copy/paste the value in this sketch
 ******************************************************************************/

// Comment this warning after you've updated the sensor bias
#warning "Use sketch AccAndGyrCalibration to get your sensor bias values"

/**
 * Accel bias
 * You must change this table from data computed from the calibration sketch
 */
 static float acc_bias[3] = { 0.0, 0.0, 0.0 };

/**
 * Gyro bias
 * You must change this table from data computed from the calibration sketch 
 */
static float gyr_bias[3] = { 0.0, 0.0, 0.0 };

/**
 * Some memory to be used by the icm30xxx driver (must be >= 4k)
 */
static uint8_t device_buffer[1024*4]; 

/**
 * Flag = true if an hardware interrupt was asserted
 */
static bool interrupt_occured = false;

/**
 * Flag = true if door opened is detected
 */
static bool door_open_detected;

/**
 * Device requires a FW and additionnal DMP images to be loaded on init
 * Provides such images by mean of a byte array
 */
 
/**
 * M0 image
 */
// No M0 image for this sketch, you must flash Tungsten with Studio flow sample
// "Door opening detection"

/**
 * DMP3 image
 */
static const uint8_t dmp3_image[] = {
	#include "Images/icm30630_img.sram.dmp3.h"
};

/**
 * DMP4 image
 */
static const uint8_t dmp4_image[] = {
	#include "Images/icm30630_img.sram.dmp4.h"
};


/* Private function code ------------------------------------------------------*/

/** @brief Display a message (through means of printer function)
 *  Called from INV_MSG macro defined in Utils/Message.h
 *  @param[in]  level  level for the message
 *  @param[in]  str    message string
 *  @param[in]  ap     optional arguments
 *  @return     none
 */
 static void inv_msg_printer_arduino(int level, const char * str, va_list ap)
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
 *  @return     none
 */
static void printTraces(const char * str, ...) __attribute__ ((format (printf, 1, 2)));
static void printTraces(const char * str, ...)
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


/** @brief Sensor listener event callback definition
 *  @param[in]  event     reference to sensor event
 *  @param[in]  arg       listener context
 *  @return     none
 */
static void sensor_event_cb(const inv_sensor_event_t * event, void * arg)
{
	switch(event->sensor) 
	{
		case DOOR_OPENING_CUSTOM_SENSOR_ID:
			if(event->status == INV_SENSOR_STATUS_DATA_UPDATED)
			{
				if (event->data.event != 0) 
				{
					if (door_open_detected == false)
					{
						// Add a traces
						printTraces("Door open detected");
						
						// Tone buzzer and active led for 5s
						tone(DETECTION_LED_PIN, 1000, 5000);
						
						door_open_detected = true;
					}
				}
				else
				{
					if (door_open_detected == true)
					{
						printTraces("Door close detected");
						
						door_open_detected = false;
					}
				}
			}
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
 * @param[in]  context         context passed to callback
 * @param[in]  int_num         interrupt number
 * @return     none
 */
static void device_interrupt_cb(void * context, int int_num)
{
	// Set interrupt flag
	interrupt_occured = true;
	
	// Avoid warning. We don't need any of this parameters for our function
	(void) context;
	(void) int_num;
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

	// M0 image must be flashed with studio
	.fw_image_buffer        = NULL,
	.fw_image_buffer_size   = 0,

	.dmp3_image_buffer      = dmp3_image,
	.dmp3_image_buffer_size = sizeof(dmp3_image),
	.dmp4_image_buffer      = dmp4_image,        
	.dmp4_image_buffer_size = sizeof(dmp4_image),
	
	.acc_gyr_mounting_matrix = {1.0, 0.0, 0.0,
	                            0.0, 1.0, 0.0,
	                            0.0, 0.0, 1.0},
	// Align mag axis with accel and gyro
	// If you mount a magnetometer with a different axis referential from this daughter board, please, change the matrix
	.mag_mounting_matrix     = {0.0, -1.0, 0.0,
	                            1.0, 0.0, 0.0,
	                            0.0, 0.0, 1.0},
};

#pragma GCC diagnostic pop // Now "missing-field-initializer" is tested


/** @brief Start sensor
 *  @return return code of driver function
 */
static int start_sensor(void)
{
	int rc;
	int pingRc;
	
	// Ping and start sensor
	printTraces("Ping custom sensor");
	pingRc = inv_device_ping_sensor(device_settings.device, DOOR_OPENING_CUSTOM_SENSOR_ID);
	
	if(pingRc != INV_ERROR_SUCCESS)
	{	
		// Sensor not pinged => Display an error 
		printTraces("ERROR : Custom Sensor not detected");
		printTraces("ERROR : You must flash the ICM with Studio Door opening detection ");
		// Wait here
		while(1);
	}
	else
	{
		// Start sensor at 10Hz and data are not bufferized
		printTraces("Start custom sensor");
		rc = inv_easy_device_start_sensor(&device_settings, DOOR_OPENING_CUSTOM_SENSOR_ID, 100, 0);
		TEST_RC(rc);
	}
	
	return rc;
}


/** @brief Stop sensor
 *  @return return code of driver function
 */
static int stop_sensor(void)
{
	int rc;
	uint8_t i;
	
	// Stop sensor
	rc = inv_easy_device_stop_sensor(&device_settings, DOOR_OPENING_CUSTOM_SENSOR_ID);
	TEST_RC(rc);
	
	// wait 500 ms for all data to be outputed
	for (i = 0; i < 100; i++)
	{
		rc = inv_device_poll(device_settings.device);
		inv_icm30xxx_sleep(5);
	}
	
	return rc;
}


/** @brief Reset sketch variables
 *  @return None
 */
static void resetSketchVariables(void)
{
	// Reset flag
	door_open_detected = false;
}


/** @brief Init sensor
 *  @return Last return code of last driver function called
 */
static int initSketch(void)
{
	int              rc;
	uint8_t          whoami;
	inv_fw_version_t fw_version;
	
	// Reset variables
	resetSketchVariables();
	
	// Setup driver messages if you want to see device driver traces
	printTraces("Setup msg level as warning");
	inv_msg_setup(MSG_LEVEL, inv_msg_printer_arduino);
	
	// Device easy init
	printTraces("Easy device init");
	rc = inv_easy_device_init(&device_settings, &whoami, &fw_version);
	TEST_RC(rc);
	
	// Test who am i
	if(whoami != 0xC0)
	{
		// who am i incorrect
		rc = INV_ERROR_UNEXPECTED;
		printTraces("FAIL : Device who am i must be 0xC0");
		return rc;
	}
	
	// Set accel and gyro bias
	rc = inv_device_set_sensor_bias(device_settings.device, INV_SENSOR_TYPE_ACCELEROMETER, acc_bias);
	TEST_RC(rc);
	
	rc = inv_device_set_sensor_bias(device_settings.device, INV_SENSOR_TYPE_GYROMETER, gyr_bias);
	TEST_RC(rc);
	
	// Start sensor at 10 Hz and data are not bufferized
	rc = start_sensor();
	TEST_RC(rc);
	
	// Reset interrupt flag
	interrupt_occured = false;
	
	return(rc);
}


/* Public function code ------------------------------------------------------*/


/** @brief Sleep implementation for Icm30xxx.
 *  @param[in]  ms  millisecond  of sleep
 *  @return     none
 */
void inv_icm30xxx_sleep(int ms)
{
	/*
	 * You must provide a sleep function that blocks the current programm
	 * execution for the specified amount of ms
	 */
	delay(ms);
}

/** @brief Arduino sketch setup function
 *  Called at init by Arduino Sketch
 *  @return     none
 */
void setup() 
{
	// Configure led pin in output
	pinMode(DETECTION_LED_PIN, OUTPUT);
	
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
	if(SERIAL_TRACES.available() == 0)
	{
		// No, test if interrupt occured
		if(interrupt_occured != false)
		{
			// Reset interrupt flag
			interrupt_occured = false;
			
			// Interrupt occur poll device
			inv_device_poll(device_settings.device);
		}
	}
	else
	{
		// Char in buffer Stop Sensor
		stop_sensor();
		printTraces("Sensor STOPPED !");
		
		// Flush serial buffer
		while(SERIAL_TRACES.available() != 0)
		{
			SERIAL_TRACES.read();
		}
		
		// Display restart info
		printTraces("Send a char for restarting sensor");
		while(SERIAL_TRACES.available() == 0);
		
		// Wait all incoming char
		delay(100); 
		
		// Flush serial buffer
		while(SERIAL_TRACES.available() != 0)
		{
			SERIAL_TRACES.read();
		}
		
		resetSketchVariables();
		start_sensor();
	}
}
