/*
* ________________________________________________________________________________________________________
* Copyright (c) 2014-2015 InvenSense Inc. Portions Copyright (c) 2014-2015 Movea. All rights reserved.
*
* This software, related documentation and any modifications thereto (collectively "Software") is subject
* to InvenSense and its licensors' intellectual property rights under U.S. and international copyright and
* other intellectual property rights laws.
*
* InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
* and any use, reproduction, disclosure or distribution of the Software without an express license
* agreement from InvenSense is strictly prohibited.
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

/** @defgroup 	FifoProtocolDriverWrapper  FifoProtocolDriverWrapper
 	@brief 		Wrapper to abstract hardware FIFO
    @ingroup 	FifoProtocolAdapter
    @{ 
*/

#ifndef _FIFO_PROTOCOL_DRIVER_WRAPPER_H_
#define _FIFO_PROTOCOL_DRIVER_WRAPPER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "FifoProtocolDefs.h"

/** @brief 		Read data from a fifo
	@param[in]	fifo 	fifo type
	@param[out]	out 	data buffer
	@param[in]	len 	length of data to read
	@return 	FifoProtocolError
*/
extern int inv_fifo_protocol_driver_wrapper_fifo_read(FifoProtocolType fifo,
                uint8_t *data, uint16_t len);

/** @brief 		Write data to a fifo
	@param[in]	fifo 	fifo type
	@param[in]	out 	data buffer
	@param[in]	len 	length of data to write
	@return 	FifoProtocolError
*/
extern int inv_fifo_protocol_driver_wrapper_fifo_write(FifoProtocolType fifo,
                const uint8_t *data, uint16_t len);

/** @brief 		Return current number of byte hold in a fifo
	@param[in]	fifo 	fifo type
	@param[in]	len 	number of byte in the fifo
	@return 	FifoProtocolError
*/
extern int inv_fifo_protocol_driver_wrapper_fifo_size(FifoProtocolType fifo,
                uint16_t *len);

/** @brief Trigger an explicit flush
	@param[in]	fifo 		fifo to flush
	@param[in]	mustFlushNow the flush must happen quickly (in case of FIFO overflow for instance) or not
	@return 	FifoProtocolError
*/
extern int inv_fifo_protocol_driver_wrapper_trigger_flush(FifoProtocolType fifo,
                uint8_t mustFlushNow);

/**	@brief Update configuration of a FIFO.
	@param[in]	fifo 		fifo to configure
	@param[in]	opt 		option flag
	@param[in]	size 		requested size of the FIFO
	@param[in]	waterlark	watermark threshold
	@retrun  	FifoProtocolError
*/
extern int inv_fifo_protocol_driver_wrapper_set_fifo_config(
        FifoProtocolType fifo, FifoProtocolOpt opt, unsigned size, unsigned watermark);

/**	@brief Return configuration of a FIFO.
	@param[in]	fifo 		fifo to get configuration from
	@param[out]	opt 		current option flag
	@param[out]	size 		current size of the FIFO
	@param[out]	waterlark	current watermark threshold
	@retrun  	FifoProtocolError
*/
extern int inv_fifo_protocol_driver_wrapper_get_fifo_config(
        FifoProtocolType fifo, FifoProtocolOpt *opt, unsigned *size,
        unsigned *watermark);

#ifdef __cplusplus
}
#endif

#endif /* _FIFO_PROTOCOL_DRIVER_WRAPPER_H_ */

/** @} */
