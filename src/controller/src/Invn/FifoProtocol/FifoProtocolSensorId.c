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

/*
 * This file is just there to perform some sanity checks regarding sensor ids at compile time
 */

#include "FifoProtocolSensorId.h"

#ifdef  CASSERT
	#undef  CASSERT
#endif

#define _impl_PASTE(a,b) a##b
#define _impl_CASSERT_LINE(predicate, line) \
		typedef char _impl_PASTE(assertion_failed_,line)[2*!!(predicate)-1];

#define CASSERT(predicate) _impl_CASSERT_LINE(predicate, __LINE__)

CASSERT(FIFOPROTOCOL_SENSOR_ID_MAX      == 32);

CASSERT(FIFOPROTOCOL_SENSOR_ID_EXT_BASE == 32);
CASSERT(FIFOPROTOCOL_SENSOR_ID_EXT_MAX  == (32+24));
CASSERT(FIFOPROTOCOL_SENSOR_ID_EXT_CNT  == 24);

CASSERT(FIFOPROTOCOL_SENSOR_ID_CUST_BASE == 56);
CASSERT(FIFOPROTOCOL_SENSOR_ID_CUST_MAX  == (56+8));
CASSERT(FIFOPROTOCOL_SENSOR_ID_CUST_CNT  == 8);

CASSERT(FIFOPROTOCOL_SENSOR_ID_AUDIO_BASE == 48);
CASSERT(FIFOPROTOCOL_SENSOR_ID_AUDIO_MAX  == (48+8));
CASSERT(FIFOPROTOCOL_SENSOR_ID_AUDIO_CNT  == 8);

CASSERT(FIFOPROTOCOL_SENSOR_ID_WAKEUP_FLAG == 0x80);
