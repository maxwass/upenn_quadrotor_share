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

#include "Invn/Utils/Message.h"
#include "DeviceIoHal.h"

#include <Arduino.h>

#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>


#define MAX_SPI_TRANSACTION  INV_ARDUINO_EMD_ADAPTER_SERIF_MAX_TRANSACTION_SIZE
#define READ_BIT             0x80

static int spi_clock = INV_ARDUINO_EMD_ADAPTER_SPI_CLOCK_DEFAULT;

// serif instance for arduino
static const inv_host_serif_t arduino_emd_serif_instance = {
	inv_arduino_emd_adapter_open,
	inv_arduino_emd_adapter_close,
	inv_arduino_emd_adapter_read_reg,
	inv_arduino_emd_adapter_write_reg,
	inv_arduino_emd_adapter_register_interrupt_callback,
	INV_ARDUINO_EMD_ADAPTER_SERIF_MAX_TRANSACTION_SIZE,
	INV_ARDUINO_EMD_ADAPTER_SERIF_TYPE,
};


const inv_host_serif_t * inv_arduino_emd_adapter_get_instance(void)
{
	return &arduino_emd_serif_instance;
}


void inv_arduino_emd_adapter_set_spi_clock(uint16_t clock)
{
	spi_clock = clock;
}


int inv_arduino_emd_adapter_open(void)
{
	return(inv_device_io_hal_init());
}


int inv_arduino_emd_adapter_close(void)
{
	return(inv_device_io_hal_close());
}


int inv_arduino_emd_adapter_read_reg(uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
	int rc;
	rc = inv_device_io_hal_reg_read(reg, rbuffer, rlen);
	
	return rc;
}


int inv_arduino_emd_adapter_write_reg(uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{
	int rc;
	rc = inv_device_io_hal_reg_write(reg, wbuffer, wlen);

	return rc;
}


int inv_arduino_emd_adapter_register_interrupt_callback(void (*interrupt_cb)(void * context, int int_num), void * context)
{
	int rc;
	
	rc = inv_device_io_hal_register_interrupt_callback(interrupt_cb, context);

	return rc;
}
