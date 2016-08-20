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

#ifndef INVN_BRIDGE_H_
#define INVN_BRIDGE_H_

 /**
 * @file
 * @brief Encapsule bridge in C++ for easy arduino usage
 */

// Fifo Protocol API is include in the remote sensor protocol library
#include <stdint.h>
#include "Bridge.h"

//! \class InvnBridge
//! \brief Encapsule bridge in C++ for easy arduino usage
class InvnBridge
{
	public:
		//! \brief Constructor
		InvnBridge(void);
		
		//! \brief Init bridge
		void init(void);
		
		//! \brief Superloop for processing received char
		void process(void);
		
	private:
		//! \brief buffer for UART command received
		uint8_t recvBuffer[BRIDGE_MAX_DATA_RCV];
		
		//! \brief Flag = 0 if not inited
		uint8_t isinit = 0;	
};

#endif /* INVN_BRIDGE_H_ */ 
