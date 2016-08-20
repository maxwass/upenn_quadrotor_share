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

#include "FifoProtocol.h"
#include "FifoProtocolCore.h"
#include "FifoProtocolDriverWrapper.h"

/** @brief Pop command packet from the COMMAND FIFO 
*/
int inv_fifo_protocol_pop_command_packet(FifoProtocolType fifo)
{
	static struct FifoProtocolPacket lPacket;
	uint8_t raw_packet[FIFOPROTOCOL_PACKET_SIZE];
	int rc = FIFOPROTOCOL_ERROR_NO;

	if(fifo != FIFOPROTOCOL_TYPE_COMMAND)
		return FIFOPROTOCOL_ERROR_TYPE;

	do {
		uint16_t fifolen = 0;
		int cnt = 2;

		if((rc = inv_fifo_protocol_driver_wrapper_fifo_size(fifo, &fifolen)) != FIFOPROTOCOL_ERROR_NO)
			return rc;

		if(fifolen < FIFOPROTOCOL_PACKET_SIZE)
			return FIFOPROTOCOL_ERROR_FIFOEMPTY;

		if((rc = inv_fifo_protocol_driver_wrapper_fifo_read(fifo, raw_packet,
			FIFOPROTOCOL_PACKET_SIZE)) != FIFOPROTOCOL_ERROR_NO)
			return rc;

		do {
			rc = inv_fifo_protocol_parse_command_packet(&lPacket, raw_packet, FIFOPROTOCOL_PACKET_SIZE);
		} while(rc == FIFOPROTOCOL_ERROR_PARSE && --cnt > 0); /* retry if parsing error with last packet */
	} while(rc == FIFOPROTOCOL_ERROR_INCOMPLETE);

	if(rc == FIFOPROTOCOL_ERROR_NO) {
		inv_fifo_protocol_new_command_handler(lPacket.sensorid,
			(FifoProtocolCmd)lPacket.u.command.cmd, lPacket.arg, lPacket.size);
	}

	return rc;
}

static int inv_fifo_protocol_push_packet(FifoProtocolType fifo, struct FifoProtocolPacket * packet)
{
	int rc;

	uint8_t raw_packet[FIFOPROTOCOL_PACKET_SIZE];
	uint16_t size;

	inv_fifo_protocol_format_packet_reset(packet);

	do
	{
		rc = inv_fifo_protocol_format_packet(packet, raw_packet, &size);

		if(rc == FIFOPROTOCOL_ERROR_INCOMPLETE || rc == FIFOPROTOCOL_ERROR_NO) {
			int rc = inv_fifo_protocol_driver_wrapper_fifo_write(fifo, raw_packet, size);

			if(rc != FIFOPROTOCOL_ERROR_NO)
				break;
		}
	} while(rc == FIFOPROTOCOL_ERROR_INCOMPLETE);

	return rc;
}


/** @brief      Push sensor data to the specified DATA FIFO.
*/
int inv_fifo_protocol_push_data_packet(FifoProtocolType fifo, int sensorid,
                                       FifoProtocolStatus status, uint32_t timestamp,
                                       FifoProtocolAccuracy accuracy, const void *data, uint16_t len)
{
	struct FifoProtocolPacket packet;

	if(fifo == FIFOPROTOCOL_TYPE_COMMAND)
		return FIFOPROTOCOL_ERROR_TYPE;

	inv_fifo_protocol_data_packet_set(&packet, sensorid, timestamp, 
		(uint8_t)accuracy, (uint8_t)status, data, len);

	return inv_fifo_protocol_push_packet(fifo, &packet);
}


/** @brief      Push an answer to the specified FIFO.
*/
int inv_fifo_protocol_push_answer_packet(FifoProtocolType fifo, int sensorid,
                FifoProtocolCmd command, const void *data, uint16_t len)
{
	struct FifoProtocolPacket packet;

	if(fifo == FIFOPROTOCOL_TYPE_COMMAND)
		return FIFOPROTOCOL_ERROR_TYPE;

	inv_fifo_protocol_answer_packet_set(&packet, sensorid, command, data, len);

	return inv_fifo_protocol_push_packet(fifo, &packet);
}

/** @brief Trigger an explicit flush
*/
int inv_fifo_protocol_trigger_flush(FifoProtocolType fifo, uint8_t mustFlushNow)
{
	return inv_fifo_protocol_driver_wrapper_trigger_flush(fifo, mustFlushNow);
}

#define FIFOPROTOCOL_NORMAL_DATA	0
#define FIFOPROTOCOL_WAKEUP_DATA	1

/** @brief Pop data packet from the a FIFO 
*/
int inv_fifo_protocol_pop_data_packet(FifoProtocolType fifo)
{
	static struct FifoProtocolPacket lPacket[2 /* for NL and WU fifo*/];
	uint8_t raw_packet[FIFOPROTOCOL_PACKET_SIZE];
	int rc = FIFOPROTOCOL_ERROR_NO;
	uint8_t fifoidx;

	// check FIFO for NORMAL or WAKEUP data
	if(fifo == FIFOPROTOCOL_TYPE_NORMAL) {
		fifoidx = FIFOPROTOCOL_NORMAL_DATA;
	} else if(fifo == FIFOPROTOCOL_TYPE_WAKEUP) {
		fifoidx = FIFOPROTOCOL_WAKEUP_DATA;
	} else {
		return FIFOPROTOCOL_ERROR_TYPE;
	}

	do {
		uint16_t fifolen = 0;
		int cnt = 2;

		if((rc = inv_fifo_protocol_driver_wrapper_fifo_size(fifo, &fifolen)) != FIFOPROTOCOL_ERROR_NO)
			return rc;

		if(fifolen < FIFOPROTOCOL_PACKET_SIZE)
			return FIFOPROTOCOL_ERROR_FIFOEMPTY;

		if((rc = inv_fifo_protocol_driver_wrapper_fifo_read(fifo, raw_packet,
			FIFOPROTOCOL_PACKET_SIZE)) != FIFOPROTOCOL_ERROR_NO)
			return rc;

		do {
			do {
				uint8_t cnt_bytes_after_header;
				lPacket[fifoidx].protected_header_idx = 0;
				/* Look for protected header token */
				rc = inv_fifo_protocol_parse_header_packet(&lPacket[fifoidx], raw_packet);
				
				if (rc == FIFOPROTOCOL_ERROR_INCOMPLETE) {
				/* We found the token but not at index 0 like expected originally */
					uint8_t byte_idx;
					cnt_bytes_after_header = FIFOPROTOCOL_PACKET_SIZE-lPacket[fifoidx].protected_header_idx;
					/* Then discard beginning of raw packet, and move end of raw packet to the beginning */
					for (byte_idx = 0 ; byte_idx < cnt_bytes_after_header ; byte_idx++) {
						raw_packet[byte_idx] = raw_packet[lPacket[fifoidx].protected_header_idx+byte_idx];
					}
				}
				
				if (rc == FIFOPROTOCOL_ERROR_NO) {
				/* We have a complete packet with protected header checked, now check footer */
					rc = inv_fifo_protocol_parse_footer_packet(&lPacket[fifoidx], raw_packet);
					if (rc != FIFOPROTOCOL_ERROR_NO) {
					/* Header was fine but we had an issue with footer, shift raw packet by 1 byte */
						uint8_t byte_idx;
						/* Then discard 1st byte of current packet, since we know it can't have a good footer */
						for (byte_idx = 0 ; byte_idx < FIFOPROTOCOL_PACKET_SIZE-1 ; byte_idx++) {
							raw_packet[byte_idx] = raw_packet[byte_idx+1];
						}
					}
						
				}
				
				if (rc != FIFOPROTOCOL_ERROR_NO) {
				/* Either we found the token but not at index 0,
				Either we did not found any token at all in current packet
				Either we have a complete packet with protected header checked but
				protected footer token of current packet is not correct */
					int ret;
					if((ret = inv_fifo_protocol_driver_wrapper_fifo_size(fifo, &fifolen)) != FIFOPROTOCOL_ERROR_NO)
						return ret;
				
					/* We still need to read protected_header_idx
					so check if we have at least that amount of bytes available in the FIFO */
					if(fifolen < lPacket[fifoidx].protected_header_idx)
						return FIFOPROTOCOL_ERROR_FIFOEMPTY;
				
					/* So fill in remaining bytes of raw packet from FIFO based on protected_header_idx value */
					cnt_bytes_after_header = FIFOPROTOCOL_PACKET_SIZE-lPacket[fifoidx].protected_header_idx;
					if((ret = inv_fifo_protocol_driver_wrapper_fifo_read(fifo,
								&raw_packet[cnt_bytes_after_header],
								lPacket[fifoidx].protected_header_idx)) != FIFOPROTOCOL_ERROR_NO)
						return ret;
				
				}
			} while (rc != FIFOPROTOCOL_ERROR_NO);
			/* keep on fetching full packet until FIFO is empty or packet is declared as being valid */
                                                    
			/* Now we have a complete packet with protected header and footer checked,
			we can decode packet content safely now */
			rc = inv_fifo_protocol_parse_packet(&lPacket[fifoidx], raw_packet, FIFOPROTOCOL_PACKET_SIZE);
		} while(rc == FIFOPROTOCOL_ERROR_PARSE && --cnt > 0); /* retry if parsing error with last packet */
	} while(rc == FIFOPROTOCOL_ERROR_INCOMPLETE);

	if(rc == FIFOPROTOCOL_ERROR_NO) {
		if(lPacket[fifoidx].type == FIFOPROTOCOL_PACKET_TYPE_ANSWER) {
			inv_fifo_protocol_new_answer_handler(lPacket[fifoidx].sensorid,
				(FifoProtocolCmd)lPacket[fifoidx].u.command.cmd, lPacket[fifoidx].arg, lPacket[fifoidx].size);
		} else {
			inv_fifo_protocol_new_data_handler(lPacket[fifoidx].sensorid,
			(FifoProtocolStatus)lPacket[fifoidx].u.data.status, (FifoProtocolStatus)lPacket[fifoidx].u.data.timestamp,
				(FifoProtocolAccuracy)(lPacket[fifoidx].u.data.accuracy), lPacket[fifoidx].arg, lPacket[fifoidx].size
			);
		}
	}

	return rc;
}

/** @brief Push a command packet to a FIFO
*/
int inv_fifo_protocol_push_command_packet(FifoProtocolType fifo, int sensorid,
                FifoProtocolCmd command, const void *data, uint16_t len)
{
	struct FifoProtocolPacket packet;		

	if(fifo != FIFOPROTOCOL_TYPE_COMMAND)
		return FIFOPROTOCOL_ERROR_TYPE;
	
	inv_fifo_protocol_command_packet_set(&packet, sensorid, command, data, len);
	
	return inv_fifo_protocol_push_packet(fifo, &packet);
}

/*	Common methods that can be called either by the sensor hub or the host */

/**	@brief Update configuration of a FIFO
*/
int inv_fifo_protocol_set_fifo_config(FifoProtocolType fifo,
                                      FifoProtocolOpt opt, unsigned size, unsigned watermark)
{
	return inv_fifo_protocol_driver_wrapper_set_fifo_config(fifo, opt, size,
	                watermark);
}

/**	@brief Return configuration of a FIFO
*/
int inv_fifo_protocol_get_fifo_config(FifoProtocolType fifo,
                                      FifoProtocolOpt *opt, unsigned *size, unsigned *watermark)
{
	return inv_fifo_protocol_driver_wrapper_get_fifo_config(fifo, opt, size,
	                watermark);
}
