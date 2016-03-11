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

#include "Icm30xxxUartFifo.h"
#include "Icm30xxx.h"

#include "Invn/Utils/Message.h"


#include <assert.h>
#include <stdio.h>

/*  @brief 	Size for a data sensor binary frame common to all sensors*/
#define SENSOR_BINARY_DATA_FRAME_SIZE 33
/*  @brief 	Token indicating start of classical printf */
#define START_PRINTF_TOKEN 0xfe
/*  @brief 	Token indicating end of classical printf */
#define END_PRINTF_TOKEN 0xff

/*  @brief 	End of a command sent to Icm30xxx through FIFO */
#define ENTER_KEYCODE   0x0D

	/** @brief Binary frame received format */
typedef struct{
	unsigned char sensor_id;
	char * name;
	char decodeFormat[10];
}frame_decode_type;


/** @brief Way to decode an incoming binary frame, must be in line with embedded code
 *  Format is always sensorid.timestamp.fields
 *  f is for float
 *  F is for float and end of data payload
 *  i is for unsigned integer
 *  I is for unsigned integer and end of data payload
 *  0 means no field to decode, frame is done
 */
static const frame_decode_type decode_Frame[] = {
	{ ANDROID_SENSOR_ACCELEROMETER,               "accel",       {'f', 'f', 'f', 'I'} },
	{ ANDROID_SENSOR_GYROSCOPE,                   "gyro" ,       {'f', 'f', 'f', 'I'} },
	{ ANDROID_SENSOR_GYROSCOPE_UNCALIBRATED,      "gyroraw",     {'f', 'f', 'f', 'f', 'f', 'F'} },
	{ ANDROID_SENSOR_GEOMAGNETIC_FIELD,           "mag",         {'f', 'f', 'f', 'I'} },
	{ ANDROID_SENSOR_MAGNETIC_FIELD_UNCALIBRATED, "magraw",      {'f', 'f', 'f', 'f', 'f', 'F'} },
	{ ANDROID_SENSOR_STEP_COUNTER,                "stepcounter", {'I'} },
	{ ANDROID_SENSOR_STEP_DETECTOR,               "stepdetect",  {'I'} },
	{ ANDROID_SENSOR_PRESSURE,                    "pressure",    {'f', 'f', 'f', 'I'} },
	{ ANDROID_SENSOR_GAME_ROTATION_VECTOR,        "grv",         {'f', 'f', 'f', 'F'} },
	{ ANDROID_SENSOR_GEOMAGNETIC_ROTATION_VECTOR, "gmrv",        {'f', 'f', 'f', 'f', 'F'} },
	{ ANDROID_SENSOR_ROTATION_VECTOR,             "rv",          {'f', 'f', 'f', 'f', 'F'} },
	{ ANDROID_SENSOR_GRAVITY,                     "gravity",     {'f', 'f', 'F'} },
	{ ANDROID_SENSOR_LINEAR_ACCELERATION,         "linearacc",   {'f', 'f', 'F'} },
	{ ANDROID_SENSOR_ORIENTATION,                 "orientation", {'f', 'f', 'F'} },
	{ ANDROID_SENSOR_WAKEUP_SIGNIFICANT_MOTION,   "smd",         {'I'} },
	{ ANDROID_SENSOR_ACTIVITY_CLASSIFICATON,      "bac",         {'i', 'I'} },
	{ ANDROID_SENSOR_WAKEUP_TILT_DETECTOR,        "tilt",        {'i', 'I'} },
	{ ANDROID_SENSOR_FLIP_PICKUP,                 "pickup",      {'I'} },
	{ SELF_TEST,                                  "selftest",    {'I'} },
	{ SETUP,                                      "info",        {'i', 'i', 'i', 'i', 'I'} },
	{ 0xFF, "0", {'0'} }
};
                                  
/*---------------------------INIT--------------------------------*/
int inv_icm30xxx_uartfifo_retrieve_fifo_config(struct inv_icm30xxx * s)
{
	int rc;

	memset(s->fifo_states, 0, sizeof(s->fifo_states));

	if((rc = inv_icm30xxx_fifo_buffer_setup(s, INV_ICM30XXX_FIFO_ID_0)) != 0)
		return rc;

	if((rc = inv_icm30xxx_fifo_buffer_setup(s, INV_ICM30XXX_FIFO_ID_1)) != 0)
		return rc;

	/* set data FIFO ring buffer to some location from the global memory buffer */
	s->fifo_states[INV_ICM30XXX_FIFO_ID_1].buffer     = &s->memory.buffer[0];
	s->fifo_states[INV_ICM30XXX_FIFO_ID_1].buffer_max = s->memory.buffer_len;
	s->fifo_states[INV_ICM30XXX_FIFO_ID_1].buffer_len = 0;

	return 0;
}

/*---------------------PROCESS INCOMING DATA------------------------*/

static void data_handler(struct inv_icm30xxx * s, uint8_t sensorid,
		uint32_t timestamp, const uint8_t * data, uint16_t size)
{
    /* notify upper layer */
    s->handler.data(s->handler.context, sensorid, timestamp, 0, 0, data, size);
}

/*!
******************************************************************************
* @brief 	Decode byte per byte for each binary frame, being sensor data themselves
*			Incoming format is always sensorid timestamp fields
*			Output format will always be sensorname, timestamp, fields formated
* @param[in] data		pointer to next incoming token to be parsed 
* @param[in] tokenId	token ID in the binary frame, start from 0
* @param[inout] bytesEaten		the length of the frame analyzed, incremented based on fields length decoded
* @return 0 keeps on processing next token / -1 end of frame
******************************************************************************
*/ 
static int decode_one_binary_token(struct inv_icm30xxx * s, char * data, unsigned char tokenId ,unsigned short *bytesEaten) 
{
	unsigned char lIndx = 0;

	// this is sensorid field
	if ( tokenId == 0)
	{
		// parse decode_Frame array to check sensor_id is known
		do
		{
			if ( *data == decode_Frame[lIndx].sensor_id )
			{
				// sensor found, print it on console
				s->response.sensorid = lIndx;
				// number of bytes analyzed for this is 1 char
				*bytesEaten = 1;
				// keep on processing current frame
				return 0;
			}
			lIndx++;
		} while (decode_Frame[lIndx].sensor_id != 0xFF );
		// we reached end of decode_Frame array, this is an error, sensor id is unknown
		// stop processing current frame this has no sense
		return -1;
	}
	// this is timestamp field
	else if ( tokenId == 1)
	{
		// print timestamp on console
		unsigned long long int timestamp_64b =  *( unsigned long long int*) data;
		s->response.payload.sensor_data.timestamp = (uint32_t) timestamp_64b;
		// number of bytes analyzed for this is 1 long long
		*bytesEaten += 8;
	}
	// this is payload data fields
	else 
	{
		switch (  decode_Frame[s->response.sensorid].decodeFormat[tokenId-2] )
		{
			case 'f' :
				// print float on console
				memcpy(&s->response.payload.sensor_data.data[4*(tokenId-2)], data, 4);
				// number of bytes analyzed for this is 1 float
				*bytesEaten += 4;
			break;

			case 'i' :
				// print integer on console
				memcpy(&s->response.payload.sensor_data.data[4*(tokenId-2)], data, 4);
				// number of bytes analyzed for this is 1 int
				*bytesEaten += 4;
			break;

			case 'F' :
				// print float on console together with carriage return
				memcpy(&s->response.payload.sensor_data.data[4*(tokenId-2)], data, 4);
				// number of bytes analyzed for this is 1 int
				*bytesEaten += 4;
				s->response.payload.sensor_data.len = *bytesEaten;
				/* signal the response */
				s->response.event = true;
				// and end processing frame
				data_handler(s, decode_Frame[s->response.sensorid].sensor_id,
						s->response.payload.sensor_data.timestamp,
						(const uint8_t *)s->response.payload.sensor_data.data,
						*bytesEaten);
				return -1;

			case 'I' :
				// print integer on console together with carriage return
				memcpy(&s->response.payload.sensor_data.data[4*(tokenId-2)], data, 4);
				// number of bytes analyzed for this is 1 int
				*bytesEaten += 4;
				s->response.payload.sensor_data.len = *bytesEaten;
				/* signal the response */
				s->response.event = true;
				// and end processing frame
				data_handler(s, decode_Frame[s->response.sensorid].sensor_id,
						s->response.payload.sensor_data.timestamp,
						(const uint8_t *)s->response.payload.sensor_data.data,
						*bytesEaten);
				return -1;

			default:
				// this is a 0 or unknown field, so end processing frame
				return -1;
			break;
		}
	}

	return 0;	
}

/*!
******************************************************************************
* @brief 	Decode incoming binary Frame being sensor data
* @param[in] data		pointer to binary frame 
* @return None
******************************************************************************
*/ 
static void decode_one_binary_frame(struct inv_icm30xxx * s, char * data) 
{
	unsigned char lTokenId = 0; // Binary frame token index currently parsed
	unsigned short lByteIdx = 0; // Number of bytes already analyzed in current frame

	while (decode_one_binary_token(s, data+lByteIdx ,  lTokenId++, &lByteIdx) == 0);
}

/*!
******************************************************************************
* @brief	Decode whole FIFO data content, containing both
*			- sensor data in binary frame with 33 bytes fixed size
*			- classical printf prefixed with 0xFE and suffixed with 0xFF
* @param[in] data	incoming FIFO data
* @param[in] len	the length of the incoming FIFO data
* @return None
******************************************************************************
*/
static void decode_fifo_content(struct inv_icm30xxx * s, uint8_t fifo_idx, unsigned short len )
{
	struct inv_icm30xxx_fifo_state * fifo = &s->fifo_states[fifo_idx];
	
	unsigned short lNbBytesToParse = len + fifo->buffer_len; // total number of bytes to be analyzed in frame
	unsigned short lNextIndexToParse = 0; // index in frame at which next frame to be analyzed starts
	char * data = (char *)fifo->buffer; // HW FIFO mirroring

	do
	{
		// First byte is either a known sensor id or the printf start token
		// Check for printf start token first
		if (data[lNextIndexToParse] == (char)START_PRINTF_TOKEN)
		{
			// This is a classical printf, and not sensor data in binary format, so look for printf end token now
			unsigned short index;
			unsigned short lFirstCharIndex = lNextIndexToParse+1; // Very first payload byte after start printf token

			// loop through whole rest of FIFO data or until the end printf token is found
			for ( index = lFirstCharIndex; index < lNbBytesToParse ; index ++)
			{
				// Printf end token found
				if ( data[index] == (char)END_PRINTF_TOKEN )
				{
					unsigned short i = lFirstCharIndex;
					unsigned short lframesize = (index - lFirstCharIndex);
					char lStrToPrint[256]={'\0'};

					// print on console the bytes we looped through
					while(lframesize--) 
					{	
						strncat(lStrToPrint, &data[i++], 1);
					}
					INV_MSG(INV_MSG_LEVEL_VERBOSE, "%s", lStrToPrint);

					// Next time frame must be parsed, it must discard the bytes we just printed out
					lNextIndexToParse=index+1;
					fifo->buffer_len = 0;
					break;
				}
			}

			// we found a printf start token but no associated end token, so save buffer for next time
			if ( (index == lNbBytesToParse) && (data[lNbBytesToParse-1] != (char)END_PRINTF_TOKEN) )
			{
				memmove(&data[fifo->buffer_len],
					&data[fifo->buffer_len+lNextIndexToParse],
					lNbBytesToParse - lNextIndexToParse);
				fifo->buffer_len += lNbBytesToParse - lNextIndexToParse;
				memset (&data[fifo->buffer_len],
					0x00 ,
					fifo->buffer_max-fifo->buffer_len);
				break;
			}

		} else
		{
			//This is a sensor data frame in binary format, look for SENSOR_BINARY_DATA_FRAME_SIZE bytes
			if (lNbBytesToParse-lNextIndexToParse < SENSOR_BINARY_DATA_FRAME_SIZE )
			{
				// we do not have enough bytes to decode one entire binary frame, so wait for next round to have more data
				// and  then save remaining buffer for next time				
				memmove(&data[fifo->buffer_len],
					&data[fifo->buffer_len+lNextIndexToParse],
					lNbBytesToParse - lNextIndexToParse);
				fifo->buffer_len += lNbBytesToParse - lNextIndexToParse;
				memset (&data[fifo->buffer_len],
					0x00 ,
					fifo->buffer_max-fifo->buffer_len);
				break;
			} else
			{
				// process SENSOR_BINARY_DATA_FRAME_SIZE bytes to print them correctly
				decode_one_binary_frame(s, data+lNextIndexToParse);
				// we just ate SENSOR_BINARY_DATA_FRAME_SIZE bytes, remove them from list of bytes still to be parsed
				lNextIndexToParse += SENSOR_BINARY_DATA_FRAME_SIZE;
				fifo->buffer_len = 0;
			}
		}
	} while ( lNextIndexToParse<lNbBytesToParse); // loop through whole frame buffer 

	
}

static int read_and_parse_fifo(struct inv_icm30xxx * s, uint8_t fifo_idx)
{
	int rc = 0;
	uint16_t count;
	uint32_t total_bytes;
	struct inv_icm30xxx_fifo_state * fifo;

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

	if((rc = inv_icm30xxx_fifo_read(s, fifo->fifo_idx, &fifo->buffer[fifo->buffer_len], total_bytes)) != 0) {
		/* something bad happen, better reset the fifo to
		   maybe be able to recover later... */
		fifo->buffer_len = 0;
		goto end;
	}

	// Copy the data 
	decode_fifo_content(s, fifo_idx, total_bytes);

end:
	inv_icm30xxx_lp_enable(s, true);

	return rc;
}

int inv_icm30xxx_uartfifo_poll(struct inv_icm30xxx * s)
{
	return read_and_parse_fifo(s, INV_ICM30XXX_FIFO_ID_1);
}



/*---------------------SEND COMMANDS------------------------*/
static int uartfifo_send_command(struct inv_icm30xxx * s, char sensorid,
		uint8_t command, uint32_t arg, uint16_t size)
{
	int rc;
	uint8_t lIdx = 0;
	uint8_t buffer[128];
	const struct inv_icm30xxx_fifo_state * fifo = &s->fifo_states[INV_ICM30XXX_FIFO_ID_0];

	/* device must be in on */
	if((rc = inv_icm30xxx_lp_enable(s, false)) != 0)
		return rc;
    
	switch (command)
	{
		case UARTFIFOPROTOCOL_CMD_GETINFO:
			buffer[lIdx++] = 'e';
			buffer[lIdx++] = ' ';
			buffer[lIdx++] = 'i';
			buffer[lIdx++] = ENTER_KEYCODE;
		break;

		case UARTFIFOPROTOCOL_CMD_ENABLE:
			buffer[lIdx++] = 'e';
			buffer[lIdx++] = ' ';
			buffer[lIdx++] = sensorid;
			buffer[lIdx++] = ENTER_KEYCODE;
		break;
		
		case UARTFIFOPROTOCOL_CMD_DISABLE:
			buffer[lIdx++] = 'd';
			buffer[lIdx++] = ' ';
			buffer[lIdx++] = sensorid;
			buffer[lIdx++] = ENTER_KEYCODE;
		break;
	
		case UARTFIFOPROTOCOL_CMD_SETODR:
		{
			char lOdr[5];
			buffer[lIdx++] = 'o';
			buffer[lIdx++] = ' ';
			buffer[lIdx++] = sensorid;
			buffer[lIdx++] = ' ';
			sprintf(lOdr, "%u", (unsigned)arg);
			strcpy((char *)&buffer[lIdx], lOdr);
			lIdx += strlen(lOdr);
			buffer[lIdx++] = ENTER_KEYCODE;
		break;
		}
		
		case UARTFIFOPROTOCOL_CMD_BATCH:
		{
			char lBatch[8];
			buffer[lIdx++] = 'b';
			buffer[lIdx++] = ' ';
			sprintf(lBatch, "%u", (unsigned)arg);
			strcpy((char *)&buffer[lIdx], lBatch);
			lIdx += strlen(lBatch);
			buffer[lIdx++] = ENTER_KEYCODE;
		break;
		}
		
		case UARTFIFOPROTOCOL_CMD_SELFTEST:
		{
			buffer[lIdx++] = 'e';
			buffer[lIdx++] = ' ';
			buffer[lIdx++] = 'T';
			buffer[lIdx++] = ENTER_KEYCODE;
		break;
		}
		
		default:
			INV_MSG(INV_MSG_LEVEL_ERROR, "Error while sending command '%d'", command);
		break;
	}

	size = lIdx;

	/* write all bytes to fifo */
	if((rc = inv_icm30xxx_fifo_write(s, fifo->fifo_idx, buffer, size)) != 0)
		return rc;

	inv_icm30xxx_lp_enable(s, true);
	return rc;
}


static int send_cmd(struct inv_icm30xxx * s, char sensor, uint8_t cmd,
		uint32_t payload, uint16_t len, uint32_t wait_response)
{
	int rc = 0;

	if(wait_response) {
		s->response.event = false;
	}

	if((rc = uartfifo_send_command(s, sensor, cmd, payload, len)) != 0) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "Error %d while sending command '%d'"
				" (payload: %d bytes)", rc, cmd, len);
		return rc;
	}

	if(wait_response) {
		int timeout;

		if(wait_response != 1)
			timeout = wait_response;
		else
			timeout = 1000; /* 1s by default */

		/* poll FIFO until we received the answer */
		do {
			if((rc = inv_icm30xxx_uartfifo_poll(s)) != 0)
				return rc;

			inv_icm30xxx_sleep(10);
			timeout -= 10;

			if(timeout <= 0)
				return INV_ERROR_TIMEOUT;

		} while(!s->response.event);
	}

	return rc;
}

int inv_icm30xxx_uartfifo_get_fw_info(struct inv_icm30xxx * s, struct inv_icm30xxx_fw_version * fw)
{
	int rc;

	if((rc = send_cmd(s, '0', UARTFIFOPROTOCOL_CMD_GETINFO, 0, 0, 1)) != 0)
		return rc;

	fw->major = s->response.payload.sensor_data.data[2];
	fw->minor = s->response.payload.sensor_data.data[1];
	fw->patch = s->response.payload.sensor_data.data[0];

	memset(fw->hash, '\0', sizeof(fw->hash));
	fw->hash[0] = s->response.payload.sensor_data.data[4];
	fw->hash[1] = s->response.payload.sensor_data.data[5];
	fw->hash[2] = s->response.payload.sensor_data.data[6];
	fw->hash[3] = s->response.payload.sensor_data.data[7];

	fw->crc = (((uint32_t)s->response.payload.sensor_data.data[11])<<24) |
		(((uint32_t)s->response.payload.sensor_data.data[10])<<16) |
		(((uint32_t)s->response.payload.sensor_data.data[9])<<8) |
		(((uint32_t)s->response.payload.sensor_data.data[8]));
    
	return 0;
}

int inv_icm30xxx_uartfifo_enable_sensor(struct inv_icm30xxx * s, char sensorid,	inv_bool_t state)
{
	const int cmd = (state) ? UARTFIFOPROTOCOL_CMD_ENABLE : UARTFIFOPROTOCOL_CMD_DISABLE;

	return send_cmd(s, sensorid, cmd, 0, 0, 0);
}

int inv_icm30xxx_uartfifo_set_sensor_period(struct inv_icm30xxx * s, char sensorid,	uint32_t ms_period)
{
	return send_cmd(s, sensorid, UARTFIFOPROTOCOL_CMD_SETODR, ms_period, 0, 0);
}

int inv_icm30xxx_uartfifo_set_sensor_timeout(struct inv_icm30xxx * s, char sensorid, uint32_t ms_timeout)
{
	return send_cmd(s, sensorid, UARTFIFOPROTOCOL_CMD_BATCH, ms_timeout, 0, false);
}

int inv_icm30xxx_uartfifo_get_self_test(struct inv_icm30xxx * s)
{
	int rc;

	/* For self test need to wait 10 seconds to get an answer */
	if((rc = send_cmd(s, '0', UARTFIFOPROTOCOL_CMD_SELFTEST, 0, 0, 10000)) != 0)
		return rc;

	INV_MSG(INV_MSG_LEVEL_INFO, "SelfTest result is %d", s->response.payload.sensor_data.data[0]);

	return 0;
}
