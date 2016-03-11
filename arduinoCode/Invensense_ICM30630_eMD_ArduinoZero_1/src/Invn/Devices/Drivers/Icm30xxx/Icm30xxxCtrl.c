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

#include "Icm30xxxCtrl.h"
#include "Icm30xxx.h"

#include "Invn/FifoProtocol/FifoProtocol.h"
#include "Invn/FifoProtocol/FifoProtocolCore.h"
#include "Invn/FifoProtocol/FifoProtocolHelper.h"

#include "Invn/Utils/Message.h"


#include <assert.h>

int inv_icm30xxx_ctrl_retrieve_fifo_config(struct inv_icm30xxx * s)
{
	int rc;

	memset(s->fifo_states, 0, sizeof(s->fifo_states));

	if((rc = inv_icm30xxx_fifo_buffer_setup(s, INV_ICM30XXX_FIFO_ID_0)) != 0)
		return rc;

	if((rc = inv_icm30xxx_fifo_buffer_setup(s, INV_ICM30XXX_FIFO_ID_1)) != 0)
		return rc;

	if((rc = inv_icm30xxx_fifo_buffer_setup(s, INV_ICM30XXX_FIFO_ID_3)) != 0)
		return rc;

	/* set data FIFO ring buffer to some location from the global memory buffer */
	s->fifo_states[INV_ICM30XXX_FIFO_ID_1].buffer     = &s->memory.buffer[0];
	s->fifo_states[INV_ICM30XXX_FIFO_ID_1].buffer_max = s->memory.buffer_len / 2;
	s->fifo_states[INV_ICM30XXX_FIFO_ID_1].buffer_len = 0;
	s->fifo_states[INV_ICM30XXX_FIFO_ID_3].buffer     = &s->memory.buffer[s->memory.buffer_len / 2];
	s->fifo_states[INV_ICM30XXX_FIFO_ID_3].buffer_max = s->memory.buffer_len / 2;
	s->fifo_states[INV_ICM30XXX_FIFO_ID_3].buffer_len = 0;

	return 0;
}

static int push_command_down(struct inv_icm30xxx * s, const uint8_t * buffer,
		uint16_t len)
{
	int rc = 0;
	const struct inv_icm30xxx_fifo_state * fifo;
	uint16_t nbpkt_to_push;
	int timeout = 1000; /* 1s */

	assert(s && buffer);

	fifo = &s->fifo_states[INV_ICM30XXX_FIFO_ID_0];

	nbpkt_to_push = len/fifo->pkt_size_byte;

	/* nb byte is not aligned on pkt size, this is wrong... */
	assert(len%fifo->pkt_size_byte == 0);
	/* too many bytes to push at once. this is also wrong... */
	assert(len <= fifo->size_byte);

	/* device must be in on */
	if((rc = inv_icm30xxx_lp_enable(s, false)) != 0)
		return rc;

	do {
		uint16_t current_cnt;

		/* retrieve current hw fifo count */
		if((rc = inv_icm30xxx_fifo_get_count(s, fifo->fifo_idx, &current_cnt)) != 0)
			return rc;

		/* check if there is space in the fifo */
		if(nbpkt_to_push <= (fifo->size_byte/fifo->pkt_size_byte) - current_cnt)
			break;

		/* sleep for a bit, letting device process pending command */
		inv_icm30xxx_sleep(10);
		timeout -= 10;

		if(timeout <= 0) {
			rc = INV_ERROR_TIMEOUT;
			goto end;
		}
	} while(1);

	/* write all bytes to fifo */
	if((rc = inv_icm30xxx_fifo_write(s, fifo->fifo_idx, buffer, len)) != 0) {
		return rc;
	}

	/* trigger scratch int */
	if((rc = inv_icm30xxx_set_scratch_m0_int(s, 0x80)) != 0) // FIXME need to create a define for this
			goto end;

end:
	inv_icm30xxx_lp_enable(s, true);
	return rc;
}

int inv_icm30xxx_ctrl_send_command(struct inv_icm30xxx * s,	uint8_t sensorid,
		uint8_t command, const void * arg, uint16_t size)
{
	struct FifoProtocolPacket packet;
	uint8_t buffer[128];
	uint16_t idx = 0;
	int rc;

	inv_fifo_protocol_command_packet_set(&packet, sensorid,
			(FifoProtocolCmd)command, arg, size);

	do {
		uint16_t size;
		rc = inv_fifo_protocol_format_packet(&packet, &buffer[idx], &size);
		idx += size;
		assert(idx <= sizeof(buffer));
	} while(rc == FIFOPROTOCOL_ERROR_INCOMPLETE);

	return push_command_down(s, buffer, idx);
}

static int read_int_status(struct inv_icm30xxx * s, uint8_t * fifo_mask)
{
	int rc = 0;
	uint8_t int_status;

	assert(s && fifo_mask);

	*fifo_mask = 0;

	if((rc = inv_icm30xxx_get_scratch_int0_status(s, &int_status)) != 0)
		return rc;

	if(int_status)
		*fifo_mask |= int_status;

	if(s->variant == INV_ICM30XXX_VARIANT_ICM30630) {
		if((rc = inv_icm30xxx_get_scratch_int1_status(s, &int_status)) != 0)
			return rc;
	} else if(s->variant == INV_ICM30XXX_VARIANT_ICM30670) {
		if((rc = inv_icm30xxx_get_scratch_int2_status(s, &int_status)) != 0)
			return rc;
	} else {
		assert(0);
	}

	if(int_status)
		*fifo_mask |= int_status;

	return rc;
}

static void response_handler(struct inv_icm30xxx * s, uint8_t sensorid,
	uint8_t command, const uint8_t * data, uint16_t size)
{
	INV_MSG(INV_MSG_LEVEL_DEBUG, "Receive response (id=%d, command=%d, payload=%d)",
		sensorid, command, size);

	s->response.sensorid = sensorid;
	s->response.command  = command;

	switch(command) {
	case FIFOPROTOCOL_CMD_GETFIRMWAREINFO:
		inv_fifo_protocol_helper_decode_firmware(data, size,
				&s->response.payload.fwversion.major,
				&s->response.payload.fwversion.minor,
				&s->response.payload.fwversion.patch,
				(uint8_t *)s->response.payload.fwversion.hash,
				&s->response.payload.fwversion.crc);
		break;

	case FIFOPROTOCOL_CMD_LOAD:
		s->response.payload.load.who   = data[0];
		s->response.payload.load.what  = data[1];
		s->response.payload.load.arg   = data[2];
		s->response.payload.load.arg  |= data[3] << 8;
		s->response.payload.load.arg  |= data[4] << 16;
		s->response.payload.load.arg  |= data[5] << 24;
		break;

	case FIFOPROTOCOL_CMD_GETCALIBRATIONOFFSETS:
		inv_fifo_protocol_helper_decode_calibrationoffsets(data, size,
				s->response.payload.bias);
		break;

	case FIFOPROTOCOL_CMD_GETCALIBRATIONGAINS:
		inv_fifo_protocol_helper_decode_calibrationgains(data, size,
				s->response.payload.gain);
		break;

	case FIFOPROTOCOL_CMD_PING:
		s->response.payload.ping = (data[0] != 0);
		break;

	case FIFOPROTOCOL_CMD_GETCLOCKRATE:
		s->response.payload.clock_rate =
				inv_fifo_protocol_helper_decode_clockrate(data, size);
		break;

	default:
		/* unexpected command id */
		INV_MSG(INV_MSG_LEVEL_WARNING, "Unexpected response data frame "
				"received for command id %d. Ignored.", command);
		return;
	}

	/* signal the response */
	s->response.event = true;
}

static void data_handler(struct inv_icm30xxx * s, uint8_t sensorid,
		uint32_t timestamp, uint8_t status, uint8_t accuracy,
		const uint8_t * data, uint16_t size)
{
	/* correspond to a GETDATA command */
	if(status == FIFOPROTOCOL_STATUS_POLL) {
		/* copy response back */
		s->response.sensorid = sensorid;
		s->response.command  = FIFOPROTOCOL_CMD_GETDATA;
		s->response.payload.sensor_data.timestamp = timestamp;
		assert(size <= sizeof(s->response.payload.sensor_data.data));
		memcpy(s->response.payload.sensor_data.data, data, size);
		s->response.payload.sensor_data.len = size;
		/* signal the response */
		s->response.event = true;
	} else { 	/* asynchronous sensor event */
		/* notify upper layer */
		s->handler.data(s->handler.context, sensorid,
				timestamp, status, accuracy, data, size);
	}
}

static int parse_fifo(struct inv_icm30xxx * s, const uint8_t * buffer, uint32_t len)
{
	int rc;
	struct FifoProtocolPacket packet;
	uint32_t index = 0;

	inv_fifo_protocol_parse_packet_reset(&packet);

	do {
		int cnt = 2;

		do {
			do {
				packet.protected_header_idx = 0;
				/* Look for protected header token */
				rc = inv_fifo_protocol_parse_header_packet(&packet, &buffer[index]);
				
				if ( 	(rc == FIFOPROTOCOL_ERROR_INCOMPLETE) ||
					(rc == FIFOPROTOCOL_ERROR_PARSE) ) {
				/* We found the token but not at index 0 like expected originally */
				/* Or We did not found the token at all in current packet */
					/* Then discard beginning of (or even whole) raw packet */
					index += packet.protected_header_idx;
				}
				
				if (rc == FIFOPROTOCOL_ERROR_NO) {
				/* We have a complete packet with protected header checked, now check footer */
					rc = inv_fifo_protocol_parse_footer_packet(&packet, &buffer[index]);
					if (rc != FIFOPROTOCOL_ERROR_NO) {
					/* Header was fine but we had an issue with footer, discard 1st byte then */
						index++;
					}
						
				}
				
				if (rc != FIFOPROTOCOL_ERROR_NO) {
				/* Either we found the token but not at index 0,
				Either we did not found any token at all in current packet
				Either we have a complete packet with protected header checked but
				protected footer token of current packet is not correct */			
				/* We still need to read protected_header_idx
				so check if we have at least that amount of bytes available in the FIFO */
					if((len - index) < packet.protected_header_idx)
						return INV_ERROR; //FIFOPROTOCOL_ERROR_FIFOEMPTY;
				
				}
			} while (rc != FIFOPROTOCOL_ERROR_NO);
			/* keep on fetching full packet until FIFO is empty or packet is declared as being valid */
                                                  
			rc = inv_fifo_protocol_parse_packet(&packet, &buffer[index],
					FIFOPROTOCOL_PACKET_SIZE);

		/* retry if parsing error with last packet */
		} while(rc == FIFOPROTOCOL_ERROR_PARSE && --cnt > 0);

		if(rc == FIFOPROTOCOL_ERROR_NO) {
			if(packet.type == FIFOPROTOCOL_PACKET_TYPE_ANSWER) {
				response_handler(s, packet.sensorid,
						(FifoProtocolCmd)packet.u.command.cmd,
						packet.arg, packet.size);
			} else {
				data_handler(s, packet.sensorid,
						packet.u.data.timestamp, packet.u.data.status,
						packet.u.data.accuracy, packet.arg, packet.size);
			}
		} else if(rc == FIFOPROTOCOL_ERROR_SIZE) {
			return INV_ERROR;
		}

		index += FIFOPROTOCOL_PACKET_SIZE;

	} while((len - index) >= FIFOPROTOCOL_PACKET_SIZE);

	/* If we did not receive all packets needed for current sensor id to be fully parsed, then make
	sure the already analyzed packet(s) will be part of next call to this function */
	if (rc == FIFOPROTOCOL_ERROR_INCOMPLETE)
		index -=  packet.states.packet_cnt * FIFOPROTOCOL_PACKET_SIZE;
		
	/* return number of remaining bytes from input buffer */
	return (len - index);
}

static int read_and_parse_fifo(struct inv_icm30xxx * s, uint8_t fifo_idx)
{
	int rc = 0;
	uint16_t count;
	uint32_t total_bytes;
	uint32_t bytes_read;
	struct inv_icm30xxx_fifo_state * fifo;
	inv_bool_t reset_wm = false;

	assert(fifo_idx < INV_ICM30XXX_FIFO_ID_MAX);

	fifo = &s->fifo_states[fifo_idx];

	if((rc = inv_icm30xxx_lp_enable(s, false)) != 0)
		return rc;

	/* retrieve current hw fifo count */
	if((rc = inv_icm30xxx_fifo_get_count(s, fifo->fifo_idx, &count)) != 0) {
		/* something bad happen, better reset the fifo to
		   maybe be able to recover later... */
		fifo->buffer_len = 0;
		goto end;
	}

	/* compute total bytes */
	total_bytes = count*fifo->pkt_size_byte;
	bytes_read = 0;

	/* disable WM if above threshold to avoid multiple-interrupts */
	if(total_bytes >= fifo->wm_byte) {
		if((rc = inv_icm30xxx_fifo_set_wm(s, fifo_idx, 0xFF)) != 0) {
			/* something bad happen, better reset the fifo to
			   maybe be able to recover later... */
			fifo->buffer_len = 0;
			goto end;
		}
		reset_wm = true;
	}

	while(bytes_read < total_bytes) {
		const uint32_t max_len = fifo->buffer_max - fifo->buffer_len;
		uint32_t this_len = (total_bytes - bytes_read);
		uint8_t * buffer = &fifo->buffer[fifo->buffer_len];

		if(this_len > max_len) {
			this_len = max_len;
		}

		if((rc = inv_icm30xxx_fifo_read(s, fifo->fifo_idx, buffer, this_len)) != 0) {
			/* something bad happen, better reset the fifo to
			   maybe be able to recover later... */
			fifo->buffer_len = 0;
			goto end;
		}

		fifo->buffer_len += this_len;

		if((rc = parse_fifo(s, fifo->buffer, fifo->buffer_len)) < 0) {
			/* something bad happen, better reset the fifo to
			   maybe be able to recover later... */
			fifo->buffer_len = 0;
			goto end;
		} else if(rc > 0) { /* incomplete packet, move bytes at the top of the buffer */
			memmove(fifo->buffer, &fifo->buffer[fifo->buffer_len] - rc, rc);
			fifo->buffer_len = rc;
			rc = 0;
		} else {
			fifo->buffer_len = 0;
		}

		bytes_read += this_len;
	}

	/* reset back WM value */
	if(reset_wm) {
		if((rc = inv_icm30xxx_fifo_set_wm(s, fifo_idx, fifo->wm)) != 0){
			/* something bad happen, better reset the fifo to
			   maybe be able to recover later... */
			fifo->buffer_len = 0;
			goto end;
		}
	}

end:
	inv_icm30xxx_lp_enable(s, true);

	return rc;
}

int inv_icm30xxx_ctrl_poll(struct inv_icm30xxx * s)
{
	uint8_t fifo_mask;

	int rc;

	if((rc = read_int_status(s, &fifo_mask)) != 0)
		return rc;

	if(fifo_mask & INV_ICM30XXX_FIFO_ID_2_MASK(INV_ICM30XXX_FIFO_ID_1)) {
		if((rc = read_and_parse_fifo(s, INV_ICM30XXX_FIFO_ID_1)) != 0)
			return rc;
	}

	if(fifo_mask & INV_ICM30XXX_FIFO_ID_2_MASK(INV_ICM30XXX_FIFO_ID_3)) {
		if((rc = read_and_parse_fifo(s, INV_ICM30XXX_FIFO_ID_3)) != 0)
			return rc;
	}

	return 0;
}
