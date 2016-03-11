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
 * @brief Adapter used by the InvenSense device driver to access to the sensor with SPI
 */
 
#ifndef _INV_ARDUINO_EMD_ADAPTER_H_
#define _INV_ARDUINO_EMD_ADAPTER_H_

#include "Invn/Devices/HostSerif.h"

#ifdef __cplusplus
extern "C" {
#endif


/** @brief Return Serif Adapter instance for arduino
 */
const inv_host_serif_t * inv_arduino_emd_adapter_get_instance(void);

 
/**
 * default spi clock speed
 */
#define INV_ARDUINO_EMD_ADAPTER_SPI_CLOCK_DEFAULT            6000  /* kHz */


/**
 * Maximum number of bytes allowed per transaction
 */
#define INV_ARDUINO_EMD_ADAPTER_SERIF_MAX_TRANSACTION_SIZE   (48)


/**
 * Host adapter serif type
 */
#define INV_ARDUINO_EMD_ADAPTER_SERIF_TYPE                   (INV_HOST_SERIF_TYPE_SPI)


/** @brief Set SPI clock for arduino adapter must be called before open()
 *  @param[in]  clock spi clock in kHz
 *  @return     none
 */
void inv_arduino_emd_adapter_set_spi_clock(uint16_t clock);


/** @brief Initialyze SPI in mode 3 (CPOL=1, CPHA=1), SS active low and MSB first.
 *  @return     0 on success, negative value on error
 */
int inv_arduino_emd_adapter_open(void);


/** @brief Close the SPI and disable interrupt
 *  @return     0 on success, negative value on error
 */
int inv_arduino_emd_adapter_close(void);


/** @brief Read one register over SPI
 *  @param[in]  reg      register
 *  @param[out] rbuffer  pointer to output buffer
 *  @param[in]  rlen     number of byte to read (should not exceed MAX_TRANSACTION_SIZE-1)
 *  @return     0 on success, negative value on error
 */
int inv_arduino_emd_adapter_read_reg(uint8_t reg, uint8_t * rbuffer, uint32_t rlen);


/** @brief Write one register over SPI
 *  @param[in]  reg      register
 *  @param[in]  wbuffer  pointer to data buffer
 *  @param[in]  wlen     number of byte to write (should not exceed MAX_TRANSACTION_SIZE-1)
 *  @return     0 on success, negative value on error
 */
int inv_arduino_emd_adapter_write_reg(uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);


/** @brief Register a interrupt callback
 *  @param[in]  interrupt_cb   callback to call on interrupt
 *  @param[in]  context        context passed to callback
 *  @return     0 on success, negative value on error
 */
int inv_arduino_emd_adapter_register_interrupt_callback(void (*interrupt_cb)(void * context, int int_num), void * context);

#ifdef __cplusplus
}
#endif

#endif /* _INV_ARDUINO_EMD_ADAPTER_H_ */
