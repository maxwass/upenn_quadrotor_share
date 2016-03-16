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
 * @brief This sketch will start accelerometer and gyroscope at 200 Hz and wait for accuracy value to reach 'high' state (3). 
 *        It will then displays bias in a C-style float table. You can keep this values for other sketch which need
 *        accurate bias.
 *        The gesture required to be done is explained in the software user guide.
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

//int32_t send_data;
int32_t gyro_data[3];
float send_data;
int aa = 0;
float att_x, att_y, att_z;
float att_data[3];
uint8_t send_byte[25];
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
 
/* Public function prototypes ------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/**
 * Some memory to be used by the Icm30xxx driver (must be >= 4k)
 */
static uint8_t device_buffer[1024*4]; 

/**
 * Flag = true if an hardware interrupt was asserted
 */
static bool interrupt_occured = false;

/**
 * Accel accuracy flag
 */
static uint8_t accel_accuracy_flag;

/**
 * Gyro accuracy flag
 */
static uint8_t gyro_accuracy_flag;

/*
 * The device requires a FW and DMP images to be loaded on init
 * Images will be provided as a byte array
 */
 
/**
 * M0 image
 */
#ifdef ENABLE_FW_M0_PROG
static const uint8_t flash_image[] = {
	#include "Images/icm30630_img.flash.m0.h"
};
#endif

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
		"",      // INV_MSG_LEVEL_OFF
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
	//SERIAL_TRACES.print("[SKETCH] ");
	
	// Print the traces
	va_list ap;
	va_start(ap, str);
	vsnprintf(String, MAX_TRACES_STRING_SIZE, str, ap);
	va_end(ap);
	String[MAX_TRACES_STRING_SIZE-1] = '\0'; // Avoid overflow if string is >= MAX_TRACES_STRING_SIZE char
	
	// Print string
	SERIAL_TRACES.println(String);
}


/** @brief Convert timestamp 64 bit in us to day, hour, min, sec, ms, us
 *  @param[in]  timestamp timestamp 64 bit in us
 *  @param[out] day       day
 *  @param[out] hr        hour
 *  @param[out] min       minute
 *  @param[out] sec       second
 *  @param[out] ms        milli-second
 *  @param[out] us        micro-second
 *  @return     none
 */
static void conv_timestamp_to_time(uint64_t timestamp, uint32_t *day, uint8_t *hr, uint8_t *min, uint8_t *sec, uint16_t *ms,  uint16_t *us)
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


/** @brief Sensor listener event callback definition
 *  @param[in] event     reference to sensor event
 *  @param[in] arg       listener context
 *  @return    none
 */
static void sensor_event_cb(const inv_sensor_event_t * event, void * arg)
{
	uint32_t day;
	uint8_t hr, min, sec;
	uint16_t ms, us;
	
	switch(event->sensor) 
	{
		case INV_SENSOR_TYPE_ACCELEROMETER:
			if(event->status == INV_SENSOR_STATUS_DATA_UPDATED)
			{   

			}
			break;

      case INV_SENSOR_TYPE_ORIENTATION:
      if(event->status == INV_SENSOR_STATUS_DATA_UPDATED)
      {
       
        att_data[0] = event->data.orientation.x;
        att_data[1] = event->data.orientation.y;
        att_data[2] = event->data.orientation.z;

      }
      break;
			
		case INV_SENSOR_TYPE_GYROMETER:
			if(event->status == INV_SENSOR_STATUS_DATA_UPDATED)
			{

      
      gyro_data[0] = (int32_t) (event->data.gyr.vect[0]*1000);
      gyro_data[1] = (int32_t) (event->data.gyr.vect[1]*1000);
      gyro_data[2] = (int32_t) (event->data.gyr.vect[2]*1000);
     
     // Serial.write(send_byte,13);
      
     /*
				// Display values
				conv_timestamp_to_time(event->timestamp, &day, &hr, &min, &sec, &ms, &us);

        //send_data = (int32_t) (event->data.acc.vect[0]*1000);// <------------------
        send_data = 120.134;
        uint8_t send_byte[5];
        int32_t data_cp;
        memcpy(&data_cp, &send_data, 4);


        send_byte[0] = 0xbd;
        send_byte[1] = (data_cp);
        send_byte[2] = (data_cp >> 8);
        send_byte[3] = (data_cp >> 16);
        send_byte[4] = (data_cp >> 24);
        //printTraces(" %d, %d, %d, %d\n", send_byte[0],send_byte[1],send_byte[2],send_byte[3]);
      
          Serial.write(send_byte, 5);
*/
      /*      printTraces("                                                 GYR : X mdps = %+5ld : Y mdps = %+5ld : Z  = %+5ld ", 
            (int32_t) (event->data.gyr.vect[0]*1000), (int32_t) (event->data.gyr.vect[1]*1000), (int32_t) (event->data.gyr.vect[2]*1000));
						SERIAL_TRACES.print(" : Xxxx= ");
        SERIAL_TRACES.print(att_x, 6);
        SERIAL_TRACES.print(" : Yxxx = ");
        SERIAL_TRACES.print(att_y, 6);
        SERIAL_TRACES.print(" : Zxxx= ");
        SERIAL_TRACES.print(att_z, 6);
        SERIAL_TRACES.println("");
				*/
				// Save accuracy flag
				//gyro_accuracy_flag = event->data.gyr.accuracy_flag;
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
 *  @param[in]  context         context passed to callback
 *  @param[in]  int_num         interrupt number
 *  @return     none
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


/** @brief Init sensor
 *  @return     Last return code of last driver function called
 */
static int initSketch(void)
{
	int              rc;
	uint8_t          whoami;
	inv_fw_version_t fw_version;

static float acc_bias[3] = {  -0.008972, -0.001648, -0.025635 };
static float gyr_bias[3] = { 0.305176, 0.244141, 1.037598  };
	
	// Reset flag
	accel_accuracy_flag = 0;
	gyro_accuracy_flag = 0;
	
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
	  rc = inv_device_set_sensor_bias(device_settings.device, INV_SENSOR_TYPE_ACCELEROMETER, acc_bias);
  TEST_RC(rc);
  
  rc = inv_device_set_sensor_bias(device_settings.device, INV_SENSOR_TYPE_GYROMETER, gyr_bias);
  TEST_RC(rc);
	// Start accel and gyro sensor at 200Hz

	rc = inv_easy_device_start_sensor(&device_settings, INV_SENSOR_TYPE_GYROMETER, 5, 1);
	TEST_RC(rc);

  rc = inv_easy_device_start_sensor(&device_settings, INV_SENSOR_TYPE_ORIENTATION, 10, 10);
  TEST_RC(rc);

  
	
	// Reset interrupt flag  
	interrupt_occured = false;
	
	return(rc);
}


/** @brief Stop and close sensor
 *  @return Last return code of last driver function called
 */
static int stopSketch(void)
{
	int   rc;
	float acc_bias[3];
	float gyr_bias[3];
	uint8_t i;
	
	// Stop accel
	rc = inv_easy_device_stop_sensor(&device_settings, INV_SENSOR_TYPE_ACCELEROMETER);
	TEST_RC(rc);
	
	// Stop Gyro
	rc = inv_easy_device_stop_sensor(&device_settings, INV_SENSOR_TYPE_GYROMETER);
	TEST_RC(rc);
	
	// wait 50 ms for all data to be outputed
	for (i = 0; i < 10; i++)
	{
		rc = inv_device_poll(device_settings.device);
		inv_icm30xxx_sleep(5);
	}
	
	// Read bias
	rc = inv_device_get_sensor_bias(device_settings.device, INV_SENSOR_TYPE_ACCELEROMETER, acc_bias);
	TEST_RC(rc);
	
	rc = inv_device_get_sensor_bias(device_settings.device, INV_SENSOR_TYPE_GYROMETER, gyr_bias);
	TEST_RC(rc);
	
	// Print bias
	printTraces("BIAS COMPUTED : Copy/Paste this value in sketch which need this values");
	
	// Print bias
	SERIAL_TRACES.print("static float acc_bias[3] = { ");
	SERIAL_TRACES.print(acc_bias[0], 6);
	SERIAL_TRACES.print(", ");
	SERIAL_TRACES.print(acc_bias[1], 6);
	SERIAL_TRACES.print(", ");
	SERIAL_TRACES.print(acc_bias[2], 6);
	SERIAL_TRACES.println(" };");
	
	SERIAL_TRACES.print("static float gyr_bias[3] = { ");
	SERIAL_TRACES.print(gyr_bias[0], 6);
	SERIAL_TRACES.print(", ");
	SERIAL_TRACES.print(gyr_bias[1], 6);
	SERIAL_TRACES.print(", ");
	SERIAL_TRACES.print(gyr_bias[2], 6);
	SERIAL_TRACES.println(" };");
	
	// Close device
	rc = inv_easy_device_close(&device_settings);
	TEST_RC(rc);
	
	return(rc);
}


/* Public function code ------------------------------------------------------*/


/** @brief Sleep implementation for Icm30xxx.
 *  @param[in]  ms  millisecond of sleep
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
	// Open native serial port through USB programming port (for debug purpose)
	SERIAL_TRACES.begin(230400); // Max speed for arduino monitor
	
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
	// Now, test if interrupt occured
	if(interrupt_occured != false)
	{
		// Reset interrupt flag
		interrupt_occured = false;
		
		// Interrupt occur poll device
		inv_device_poll(device_settings.device);
	}

        send_byte[0] = 0xbd;
        send_byte[25] = 0xff;
        
        memcpy(&send_byte[1], att_data, 12);
        memcpy(&send_byte[13],gyro_data,12);
        //printTraces("%d, %d, %d\n", gyro_data[0], gyro_data[1], gyro_data[2]);
        Serial.write(send_byte, 26);
        delay(5);

}


