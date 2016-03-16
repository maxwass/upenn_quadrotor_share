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

// Warning are processed as error
#pragma GCC diagnostic push
#pragma GCC diagnostic warning "-Werror"

#include "ArduinoEmdAdapter.h"
#include "DeviceIoHal.h"
#include <SPI.h>
#include "Invn/InvError.h"

// Overload SPI function
#define SPI_begin				SPI.begin
#define SPI_beginTransaction	SPI.beginTransaction
#define SPI_endTransaction		SPI.endTransaction
#define SPI_transfer			SPI.transfer
#define SPI_end					SPI.end

// Interrupt line connected to Arduino²
#define GPIO0_INTERRUPT_LINE           5 
#define GPIO1_FSYNC_INTERRUPT_LINE     6 
// #define GPIO2_INTERRUPT_LINE           7 // Not used for ICM-30630 GA

// SPI chipSelect pin configured 
#define BRIDGE_SPI_CHIP_SELECT_PIN     10

// bit mask to notify a SPI read register
#define READ_BIT_MASK                  0x80

// SPI bus settings WARNING ICM-30630 CLK MAX IS 6,4 MHz
static SPISettings settings( (INV_ARDUINO_EMD_ADAPTER_SPI_CLOCK_DEFAULT * 1000), MSBFIRST, SPI_MODE3);

/**
 * Callback for GPIO interruption
 */
static void (*device_io_interrupt_cb)(void * context, int int_num) = NULL;

 /**
 * Conext for GPIO interruption callback
 */
static void *device_io_context;


/** @brief Callback called when an interrupt occurs on Sensor GPIO0
 *  @return     none
 */
static void isr_interrupt_sensor_gpio0(void)
{
	if(device_io_interrupt_cb != NULL)
	{
		device_io_interrupt_cb(device_io_context, 0);
	}
}


 /** @brief Callback called when an interrupt occurs on Sensor GPIO1
 *  @return     none
 */
static void isr_interrupt_sensor_gpio1(void)
{
	if(device_io_interrupt_cb != NULL)
	{
		device_io_interrupt_cb(device_io_context, 1);
	}
}


/** @brief Callback called when an interrupt occurs on Sensor GPIO2
 *  @return     none
 */
 // NOT USED IN ICM-30630 GA
/*
static void isr_interrupt_sensor_gpio2(void)
{
	if(device_io_interrupt_cb != NULL)
	{
		device_io_interrupt_cb(device_io_context, 2);
	}
}
*/


int inv_device_io_hal_init(void)
{
  // initialize the chip select pin (active low)
  pinMode(BRIDGE_SPI_CHIP_SELECT_PIN, OUTPUT);
  digitalWrite(BRIDGE_SPI_CHIP_SELECT_PIN, HIGH);

  // initialize SPI
  SPI_begin();
  
  return(INV_ERROR_SUCCESS);
}


int inv_device_io_hal_reg_write(uint8_t regAddr, const uint8_t *data, uint32_t len)
{ 
	uint32_t i;
	uint8_t WriteBuffer[1 + INV_ARDUINO_EMD_ADAPTER_SERIF_MAX_TRANSACTION_SIZE]; // Data max + ADDR

	if(len > INV_ARDUINO_EMD_ADAPTER_SERIF_MAX_TRANSACTION_SIZE)
		return INV_ERROR_SIZE;

	// In one buffer, transaction is faster    
	WriteBuffer[0] = regAddr;

	for(i=0;i<len;i++)
	{
		WriteBuffer[i+1] = data[i];
	}

	SPI_beginTransaction(settings);
	digitalWrite(BRIDGE_SPI_CHIP_SELECT_PIN, LOW);

	SPI_transfer((void *)WriteBuffer, len+1);

	digitalWrite(BRIDGE_SPI_CHIP_SELECT_PIN, HIGH);
	SPI_endTransaction();

	return(INV_ERROR_SUCCESS);
}


int inv_device_io_hal_reg_read(uint8_t regAddr, uint8_t *data, uint32_t len)
{
	uint32_t i;
	uint8_t ReadBuffer[1 + INV_ARDUINO_EMD_ADAPTER_SERIF_MAX_TRANSACTION_SIZE]; // Data max + ADDR

	if(len > INV_ARDUINO_EMD_ADAPTER_SERIF_MAX_TRANSACTION_SIZE)
		return INV_ERROR_SIZE;

	// In one buffer, transaction is faster
	ReadBuffer[0] = regAddr |= READ_BIT_MASK;

	for(i=0;i<len;i++)
	{
		ReadBuffer[i+1] = data[i];
	}

	SPI_beginTransaction(settings); 
	digitalWrite(BRIDGE_SPI_CHIP_SELECT_PIN, LOW);

	SPI_transfer((void *)ReadBuffer, len+1);

	// WARNING WE MUST COPY from read buffer to data buffer !
	for(uint8_t i=0; i<len; i++)
	{
		data[i] = ReadBuffer[i+1];
	}

	digitalWrite(BRIDGE_SPI_CHIP_SELECT_PIN, HIGH);
	SPI_endTransaction();

	return(INV_ERROR_SUCCESS);
}


int inv_device_io_hal_close(void)
{
	// Stop interrupt and reset callback
	inv_device_io_hal_register_interrupt_callback(NULL, NULL);

	// Stop SPI
	SPI_end();
	
	return(INV_ERROR_SUCCESS);
}


int inv_device_io_hal_register_interrupt_callback(void (*interrupt_cb)(void * context, int int_num), void  *context)
{
	// Critical section
	noInterrupts();
	
	// Test if callback is set or not
	if(interrupt_cb == NULL)
	{
		// Stop all interrupt
		detachInterrupt(GPIO0_INTERRUPT_LINE);
		detachInterrupt(GPIO1_FSYNC_INTERRUPT_LINE);
		// detachInterrupt(GPIO2_INTERRUPT_LINE); // NOT USED IN ICM-30630 GA
		
		// Update call back
		device_io_context      = NULL;
		device_io_interrupt_cb = NULL;
	}
	else
	{
		// Save callback
		device_io_context      = context;
		device_io_interrupt_cb = interrupt_cb;
		
		// Enable interrupt
		attachInterrupt(GPIO0_INTERRUPT_LINE, isr_interrupt_sensor_gpio0, RISING);
		attachInterrupt(GPIO1_FSYNC_INTERRUPT_LINE, isr_interrupt_sensor_gpio1, RISING);
		// attachInterrupt(GPIO2_INTERRUPT_LINE, isr_interrupt_sensor_gpio2, RISING); // NOT USED IN ICM-30630 GA
	}
	
	// End of critical section
	interrupts();
	
	return(INV_ERROR_SUCCESS);
}
