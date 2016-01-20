#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <math.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>

#define BAUDRATE B115200

//this only for recieving two bytes!!!!
int open_usbport_twobytes()
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
       newtio.c_cc[VMIN]  = 2;   // block reading until RX x characers. If x = 0, it is non-blocking.
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

