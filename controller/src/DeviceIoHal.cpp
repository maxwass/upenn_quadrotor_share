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

/**
 * Define target
 */
#define USE_ICM30670

/**
 * Overload SPI function
 */
#define SPI_begin				SPI.begin
#define SPI_beginTransaction	SPI.beginTransaction
#define SPI_endTransaction		SPI.endTransaction
#define SPI_transfer			SPI.transfer
#define SPI_end					SPI.end

/**
 * Interrupt line connected to Arduino pin
 */
#define GPIO_INTERRUPT_LINE           5 

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


/** @brief Callback called when an interrupt occurs on Sensor GPIO INT
 *  @return     none
 */
static void isr_interrupt_sensor_gpio_int(void)
{
	if(device_io_interrupt_cb != NULL)
	{
		device_io_interrupt_cb(device_io_context, INV_DEVICE_IO_INTERRUPT);
	}
}


 /** @brief Callback called when an interrupt occurs on Sensor GPIO FSYNC INT
 *  @return     none
 */
  // NOT USED
  /*
static void isr_interrupt_sensor_gpio_fsync_int(void)
{
	if(device_io_interrupt_cb != NULL)
	{
		device_io_interrupt_cb(device_io_context, INV_DEVICE_IO_FSYNC_INTERRUPT_LINE);
	}
}
*/

/** @brief Callback called when an interrupt occurs on Sensor GPIO
 *  @return     none
 */
 // NOT USED
/*
static void isr_interrupt_sensor_gpio(void)
{
	if(device_io_interrupt_cb != NULL)
	{
		device_io_interrupt_cb(device_io_context, INV_DEVICE_IO_LINE);
	}
}
*/

#if defined(USE_ICM30670)
 
/** @brief   Timer 0 interrupt handler
 *  @return  None
 */
void TCC0_Handler()
{
	/* Triggered every 350 ms */
	static uint32_t cnt = 0;
	Tcc* TCCx = (Tcc*) GetTC(0);       // get timer struct
	if (TCCx->INTFLAG.bit.OVF == 1) {  // A overflow caused the interrupt
		TCCx->INTFLAG.bit.OVF = 0x01;    // writing a one clears the flag ovf flag
		cnt++;
		if(cnt % 6 == 0)
		{
			if(device_io_interrupt_cb != NULL)
			{
				device_io_interrupt_cb(device_io_context, INV_DEVICE_TIMER_0_INTERRUPT);
			}
		}
	}
}

/** @brief   Initializes timer 0 to fire an overflow interrupt every 350ms. It also starts it.
 *  @return  None
 */
static void inv_device_io_hal_wchd_timer_init(void)
{
	Tcc* TCCx = (Tcc*) GetTC(0); // get timer struct
 
	// Enable clock for TCCx
	REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC0_TCC1) ;
	while ( GCLK->STATUS.bit.SYNCBUSY == 1 ); // wait for sync


	TCCx->CTRLA.reg &= ~TCC_CTRLA_ENABLE;   // Disable TCCx
	while (TCCx->SYNCBUSY.bit.ENABLE == 1); // wait for sync


	TCCx->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV256;   // Set perscaler


	TCCx->WAVE.reg |= TCC_WAVE_WAVEGEN_NFRQ;   // Set wave form configuration
	while (TCCx->SYNCBUSY.bit.WAVE == 1);      // wait for sync

	TCCx->PER.reg = 0xFFFF;              // Set counter Top using the PER register 
	while (TCCx->SYNCBUSY.bit.PER == 1); // wait for sync

	TCCx->CC[0].reg = 0xFFF;
	while (TCCx->SYNCBUSY.bit.CC0 == 1); // wait for sync

	// Interrupts
	TCCx->INTENSET.reg = 0;                 // disable all interrupts
	TCCx->INTENSET.reg |= TCC_INTENSET_OVF; // enable overfollow

	// Enable InterruptVector
	NVIC_EnableIRQ(TCC0_IRQn);

	// Enable TCCx
	TCCx->CTRLA.reg |= TCC_CTRLA_ENABLE ;
	while (TCCx->SYNCBUSY.bit.ENABLE == 1); // wait for sync
}

/** @brief   Stops timer 0
 *  @return  None
 */
static void inv_device_io_hal_wchd_timer_stop(void)
{
	Tcc* TCCx = (Tcc*) GetTC(0); // get timer struct
	
	TCCx->CTRLA.reg &= ~TCC_CTRLA_ENABLE;   // Disable TCCx
	while (TCCx->SYNCBUSY.bit.ENABLE == 1); // wait for sync	
}

#endif

int inv_device_io_hal_init(void)
{
	// initialize the chip select pin (active low)
	pinMode(BRIDGE_SPI_CHIP_SELECT_PIN, OUTPUT);
	digitalWrite(BRIDGE_SPI_CHIP_SELECT_PIN, HIGH);

	// initialize SPI
	SPI_begin();

#if defined(USE_ICM30670)  
	inv_device_io_hal_wchd_timer_init();
#endif
  
	return(INV_ERROR_SUCCESS);
}


int inv_device_io_hal_reg_write(uint8_t regAddr, const uint8_t *data, uint32_t len)
{ 
	uint32_t i;
	uint8_t WriteBuffer[1 + INV_ARDUINO_EMD_ADAPTER_SERIF_MAX_WRITE_TRANSACTION_SIZE]; // Data max + ADDR

	if(len > INV_ARDUINO_EMD_ADAPTER_SERIF_MAX_WRITE_TRANSACTION_SIZE)
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
	uint8_t ReadBuffer[1 + INV_ARDUINO_EMD_ADAPTER_SERIF_MAX_READ_TRANSACTION_SIZE]; // Data max + ADDR

	if(len > INV_ARDUINO_EMD_ADAPTER_SERIF_MAX_READ_TRANSACTION_SIZE)
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
	for(i=0; i<len; i++)
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
	
#if defined(USE_ICM30670) 	
	inv_device_io_hal_wchd_timer_stop();
#endif
	
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
		detachInterrupt(GPIO_INTERRUPT_LINE);
		// detachInterrupt(GPIO_FSYNC_INTERRUPT_LINE); // NOT USED
		// detachInterrupt(GPIO_LINE); // NOT USED
		
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
		attachInterrupt(GPIO_INTERRUPT_LINE, isr_interrupt_sensor_gpio_int, RISING);
		// attachInterrupt(GPIO_FSYNC_INTERRUPT_LINE, isr_interrupt_sensor_gpio_fsync_int, RISING); // NOT USED
		// attachInterrupt(GPIO_LINE, isr_interrupt_sensor_gpio, RISING); // NOT USED
	}
	
	// End of critical section
	interrupts();
	
	return(INV_ERROR_SUCCESS);
}
