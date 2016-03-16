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

/** @defgroup RingByteBuffer RingByteBuffer
 *  @brief Function to mannage circular buffer of bytes
 *  @ingroup Utils
 *  @{
*/

#ifndef _RING_BYTE_BUFFER_H_
#define _RING_BYTE_BUFFER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <assert.h>

/** @brief 	RingByteBuffer object definitions
 */
typedef struct
{
	uint8_t * 	buffer; 	/**< pointer to ring buffer data placeholder */
	uint32_t 	size;		/**< maximum size of the data buffer */
	uint32_t 	read;		/**< current start index  */
	uint32_t 	write;		/**< current end index  */
} RingByteBuffer;

/** @brief 		Initialize and reset a ring buffer
 *  @param[in]	pBuffer		pointer to buffer placeholder
 *  @param[in]	sizeBuffer	size of buffer placeholder
 *  @return none
 */
static inline void RingByteBuffer_init(RingByteBuffer * self, uint8_t * pBuffer,
		uint32_t sizeBuffer)
{
	assert(self && pBuffer && sizeBuffer);

	self->buffer = pBuffer;
	self->size   = sizeBuffer;
	self->read   = 0;
	self->write  = 0;
}

/** @brief 		Clear a ring buffer
 *  @return 	none
 */
static inline void RingByteBuffer_clear(RingByteBuffer * self)
{
	assert(self);

	self->read   = 0;
	self->write  = 0;	
}

/** @brief 		Get maximum size of a ring buffer
 *  @return 	Return maximum size of the ring buffer
 */
static inline uint32_t RingByteBuffer_maxSize(const RingByteBuffer * self)
{
	assert(self);

	return self->size;
}

/** @brief 		Get current size of a ring buffer
 *  @return 	Return number of byte contained in the ring buffer
 */
static inline uint32_t RingByteBuffer_size(const RingByteBuffer * self)
{
	assert(self);

	return self->write - self->read;
}

/** @brief 		Get number of empty slot of a ring buffer
 *  @return 	Return number of byte that can be stored in the ring buffer
 */
static inline uint32_t RingByteBuffer_available(const RingByteBuffer * self)
{
	assert(self);

	return (self->size - RingByteBuffer_size(self));
}

/** @brief 		Check for ring buffer fullness
 *  @return 	Return true (1) if ring buffer is full, false (0) otherwise
 */
static inline int RingByteBuffer_isFull(const RingByteBuffer * self)
{
	assert(self);

	return (RingByteBuffer_size(self) == self->size);
}

/** @brief 		Check for ring buffer emptyness
 *  @return 	Return true (1) if ring buffer is empty, false (0) otherwise
 */
static inline int RingByteBuffer_isEmpty(const RingByteBuffer * self)
{
	assert(self);

	return (RingByteBuffer_size(self) == 0);
}

/** @brief 		Push a byte to a ring buffer
 *              Fullness test must be done by the caller
 *  @param[in] 	byte 	byte to push to the ring buffer
 *  @return 	none
 */
static inline void RingByteBuffer_pushByte(RingByteBuffer * self, uint8_t byte)
{
	assert(self);

	self->buffer[self->write % self->size] = byte;
	++self->write;
}

/** @brief 		Pop a byte from a ring buffer
 *              Emptyness test must be done by the caller
 *  @return 	Byte pop from the ring buffer
 */
static inline uint8_t RingByteBuffer_popByte(RingByteBuffer * self)
{
	uint8_t byte;

	assert(self);

	byte = self->buffer[self->read % self->size];
	++self->read;

	return byte;
}

/** @brief 		Push a buffer of data to a ring buffer
 *              No fullness sanity check are performed
 *  @param[in] 	data 	pointer to data to push to the ring buffer
 *  @param[in] 	size  	size of data to push to the ring buffer
 *  @return 	none
*/
static inline void RingByteBuffer_pushBuffer(RingByteBuffer * self,
		const void * data, uint32_t len)
{
	const uint8_t * byte;
	uint32_t i;
	
	assert(self && data);

	byte = (const uint8_t *)data;

	for(i = 0; i < len; ++i) {
		RingByteBuffer_pushByte(self, byte[i]);
	}
}

/** @brief 		Pop a buffer of data to a ring buffer
 *              No emptyness sanity check are performed
 *  @param[in] 	data 	pointer to placeholder
 *  @param[in] 	size  	size of data to pop to the ring buffer
 *  @return 	none
*/
static inline void RingByteBuffer_popBuffer(RingByteBuffer * self, void * data,
		uint32_t len)
{
	uint8_t * byte;
	uint32_t i;
	
	assert(self && data);

	byte = (uint8_t *)data;

	for(i = 0; i < len; ++i) {
		byte[i] = RingByteBuffer_popByte(self);
	}
}

#ifdef __cplusplus
}
#endif

#endif /* _RING_BYTE_BUFFER_H_ */

/** @} */
