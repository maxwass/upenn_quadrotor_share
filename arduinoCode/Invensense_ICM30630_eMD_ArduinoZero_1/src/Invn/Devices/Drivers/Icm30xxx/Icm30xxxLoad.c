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

#include "Icm30xxxLoad.h"
#include "Icm30xxx.h"

#include "Invn/FifoProtocol/FifoProtocol.h"
#include "Invn/FifoProtocol/FifoProtocolCore.h"
#include "Invn/FifoProtocol/FifoProtocolHelper.h"

#include "Invn/Utils/Message.h"

static int read_flash_and_cmp(struct inv_icm30xxx * s, const uint8_t * image, uint32_t size)
{
	int rc;

	/* use user provided buffer to store image read from M0 */
	uint8_t * image_read = s->memory.buffer;
	const uint32_t size_max = (s->memory.buffer_len < size) ?
			s->memory.buffer_len : size;
	uint32_t bytes_cnt = 0;

	while(bytes_cnt < size) {
		uint32_t this_len = size - bytes_cnt;

		if(this_len > size_max)
			this_len = size_max;

		memset(image_read, 0, this_len);

		if((rc = inv_icm30xxx_flash_read(s, bytes_cnt, image_read, this_len, true)) != 0) {
			INV_MSG(INV_MSG_LEVEL_ERROR, "Error %d reading %d bytes from FLASH at offset",
					rc, this_len, bytes_cnt);
			return rc;
		}

		if(memcmp(image_read, &image[bytes_cnt], this_len) != 0) {
			return 1;
		}

		bytes_cnt += this_len;
	}

	return 0;
}

int inv_icm30xxx_load_flash(struct inv_icm30xxx * s, const uint8_t * image,
		uint32_t size, inv_bool_t verify, inv_bool_t force)
{
	int rc;

	// FIXME this function should rely on CRCs to avoid read image from M0

	/* reset device */
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "Resetting device...");
	if((rc = inv_icm30xxx_soft_reset(s)) != 0)
		return rc;

	/* first read flash and compare to provided image */
	if(!force) {
		INV_MSG(INV_MSG_LEVEL_VERBOSE, "Comparing provided image to device's FLASH memory...");
		if((rc = read_flash_and_cmp(s, image, size)) < 0) {
			return rc;
		} else if(rc == 0) {
			INV_MSG(INV_MSG_LEVEL_INFO, "Provided image identical to device's FLASH memory."
					" Ending load process.");
			return 0;
		}
	}

	/* do flash image */
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "Flashing image...");
	if((rc = inv_icm30xxx_flash_write(s, image, size)) != 0) {
		INV_MSG(INV_MSG_LEVEL_INFO, "Error %d while flashing image.", rc);
		return rc;
	}

	if(verify) {
		/* read flashed image and compare again */
		INV_MSG(INV_MSG_LEVEL_VERBOSE, "Verifying image...");
		if((rc = read_flash_and_cmp(s, image, size)) < 0) {
			return rc;
		} else if(rc > 0) {
			INV_MSG(INV_MSG_LEVEL_ERROR, "Image verification failed");
			return INV_ERROR;
		}
	}

	/* reset device */
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "Resetting device...");
	if((rc = inv_icm30xxx_soft_reset(s)) != 0)
		return rc;

	return 0;
}

int inv_icm30xxx_load_other(struct inv_icm30xxx * s, int who, const uint8_t * image,
		uint32_t size)
{
	int rc;
	uint32_t crc;

	/* First wake up icm30xxx */
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "Waking up icm30xxx...");
	if((rc = inv_icm30xxx_wake_up(s)) != 0)
		return rc;

	/* Initiate load command */
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "Initiate loading...");
	if((rc = inv_icm30xxx_cmd_load(s, who, FIFOPROTOCOL_LOAD_WHAT_MALLOC, size)) != 0)
		return rc;

	/* M0 responded with address location for loading image, do so... */
	if(s->response.payload.load.arg == 0) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "Cannot load %u to memory", size);
		return INV_ERROR;
	} else if (s->response.payload.load.arg < 0) {
		assert(0); // FIXME handle block load
	} else {
		INV_MSG(INV_MSG_LEVEL_VERBOSE, "Writing image to memory @ 0x%x...",
				s->response.payload.load.arg);

		if((rc = inv_icm30xxx_write_mem(s, s->response.payload.load.arg,
				image, size)) != 0)
			return rc;
	}

	/* Now that image was written to SRAM, ask for CRC check */
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "Checking image integrity...");

	crc = 0; // FIXME for now FW does not compute CRC and expect 0
	if((rc = inv_icm30xxx_cmd_load(s, who, FIFOPROTOCOL_LOAD_WHAT_CHECK, crc)) != 0)
		return rc;

	if(s->response.payload.load.arg != (int)crc) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "CRC check does not match (got 0x%x vs 0x%x expected)",
			s->response.payload.load.arg, crc);
		return INV_ERROR;
	}

	/* Restore icm30xxx power mode */
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "Putting Icm30xxx back in sleep mode...");
	if((rc = inv_icm30xxx_set_sleep(s)) != 0)
		return rc;

	return 0;
}
