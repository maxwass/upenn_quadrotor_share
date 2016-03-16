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
  * @brief This sketch made a USB <-> SPI bridge to allow InvenSense Studio
  *        to use the sensors on shield 
  */

// Be sure that you choose Arduino Zero board in Tools => Board
#ifndef ARDUINO_SAMD_ZERO
#	error "You must choose Arduino Zero in IDE Tools => Board"
#endif

// We can not processs warning as error on whole code since Arduino libraries
// has warnings in -Wall
#pragma GCC diagnostic push
#pragma GCC diagnostic warning "-Werror"

#include <ArduinoEmdAdapter.h>
#include <DeviceIoHal.h>
#include <Bridge/InvnBridge.h>

#include <SPI.h>

/* Private define ------------------------------------------------------------*/

/**
 * Led L pin 
 */ 
#define LED_L_PIN    (13)

/* Public function prototypes ------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/**
 * Bridge USB <-> SPI object
 */
InvnBridge invn_bridge;

/* Public function code ------------------------------------------------------*/

/** @brief Arduino sketch setup function
 *  Called at init by Arduino Sketch
 *  @return     none
 */
void setup() 
{
	// Init bridge
	invn_bridge.init();
	
	// Configure led pin in output
	pinMode(LED_L_PIN, OUTPUT);
	
	// Led on
	digitalWrite(LED_L_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
}

/** @brief Arduino sketch loop function
 *  called in loop by Arduino sketch
 *  @return     never
 */
void loop()
{
	// Superloop of bridge
	invn_bridge.process();
}
