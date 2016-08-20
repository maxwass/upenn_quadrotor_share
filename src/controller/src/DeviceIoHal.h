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

/** @defgroup DeviceIoHal DeviceIoHal
 *  @brief    DeviceIo HAL implementation
 *  @ingroup  Sensor
 *  @{
 */

#ifndef _INV_SENSOR_DEVICE_IO_HAL_H_
#define _INV_SENSOR_DEVICE_IO_HAL_H_

// Integer
#include <stdint.h>
#include <stdbool.h>

/**
 * GPIO INTERRUPT number
 */
#define INV_DEVICE_IO_INTERRUPT          0

/**
 * GPIO FSYNC INTERRUPT line number
 */
// #define INV_DEVICE_IO_FSYNC_INTERRUPT_LINE    1 // NOT USED

/**
 * GPIO line number
 */
// #define INV_DEVICE_IO_LINE                    2 // NOT USED

/**
 * TIMER INTERRUPT number
 */
#define INV_DEVICE_TIMER_0_INTERRUPT     3

 /** @brief Initialyze low level driver to communicate with sensor
 *  @return     none
 */
int inv_device_io_hal_init(void);


/** @brief Write registers to a sensor
 *  @param[in]  regAddr  address of register to write
 *  @param[in]  data     pointer to a buffer
 *  @param[in]  len      length of data to write
 *  @return     none
 */
int inv_device_io_hal_reg_write(uint8_t regAddr, const uint8_t *data, uint32_t len);


/** @brief Read registers to a sensor
 *  @param[in]  regAddr  address of register to read
 *  @param[out] data     pointer to a buffer
 *  @param[in]  len      length of data to read
 *  @return     none
 */
int inv_device_io_hal_reg_read(uint8_t regAddr, uint8_t *data, uint32_t len);


/** @brief Close low level driver, interrupt are stopped in this function
 *  @return     none
 */
int inv_device_io_hal_close(void);


/** @brief Register a callback to the adapter
 *  @param[in]  interrupt_cb  callback to call on interrupt
 *  @param[in]  context       For multi-thread, give information of the context
 *  @return     none
 */
int inv_device_io_hal_register_interrupt_callback(void (*interrupt_cb)(void *context, int int_num), void *context);

#endif /* _INV_SENSOR_DEVICE_IO_HAL_H_ */
