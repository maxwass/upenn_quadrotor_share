/*
Copyright (c) <2015>, <University of Pennsylvania:GRASP Lab>                                                             
All rights reserved.
 
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the university of pennsylvania nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL UNIVERSITY OF PENNSYLVANIA  BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/



//Code for sending joystick data

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <math.h>
#include <fcntl.h>
#include <termios.h>

#define BAUDRATE B115200

int open_port()
{
    struct termios newtio;
	int port;
	
	port = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
	
	tcgetattr(port, &newtio);
	cfsetospeed(&newtio, BAUDRATE);
   	cfsetispeed(&newtio, BAUDRATE);
    
       //set the number of data bits.
       newtio.c_cflag &= ~CSIZE;  // Mask the character size bits
       newtio.c_cflag |= CS8;
    
       //set the number of stop bits to 1
       newtio.c_cflag &= ~CSTOPB;
    
       //Set parity to None
       newtio.c_cflag &=~PARENB;
    
       //set for non-canonical (raw processing, no echo, etc.)
       newtio.c_iflag = IGNPAR; // ignore parity check close_port(int
       newtio.c_oflag = 0; // raw output
       newtio.c_lflag = 0; // raw input
    
       //Time-Outs -- won't work with NDELAY option in the call to open
       newtio.c_cc[VMIN]  = 24;   // block reading until RX x characers. If x = 0, it is non-blocking.
       newtio.c_cc[VTIME] = 0;   // Inter-Character Timer -- i.e. timeout= x*.1 s
    
       //Set local mode and enable the receiver
       newtio.c_cflag |= (CLOCAL | CREAD);
    
       //tcflush(port, TCIFLUSH);
	//Set the new options for the port...
	int status=tcsetattr(port, TCSANOW, &newtio);
	
	if (status != 0){ //For error message
		printf("Configuring comport failed\n");
		return status;
	}

	return port;
}

