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

//=================================
// include guard
#ifndef IMU
#define IMU

//=================================
// included dependenciesa
#include "data_structs.h"
#include "logger.h"
#include "utility.h"
#include "psi.h"


#include <iostream>
#include <stdio.h>   /* Standard input/output definitions */
#include <iostream>
#include <stdlib.h>
#include <unistd.h>  /* UNIX standard function definitions */
#include <sys/time.h>
#include <time.h>
#include <math.h>
#include <fcntl.h>   /* File control definitions */
#include <termios.h> /* POSIX terminal control definitions, Unix API for terminal I/O */
#include <string.h>

#define BAUDRATE_IMU B230400

using namespace std;

class Imu {
	int port;
	int data_size;
        bool first_loop;
        float dt;
	//for select()
	fd_set read_fds;
	struct timeval no_timeout;

	//for bias
	bool calibrated;
	State gyro_bias;

	//for Psi
	Psi p;
	Psi* pg;

	//for findDt to integrate gyro
	timespec oldT, newT;
        float calc_dt;
	float scale_factor;

  public:
    int get_imu_data(State&);
    int get_imu_calibrated_data(State&);
    void unpack_data(State&, const unsigned char[]);
    Imu(std::string PATH2IMU, int DATASIZE, float dt)
	{
		(this->data_size) = DATASIZE;
		Psi *psiGyro = new Psi(0.00005);
		(this->pg) = psiGyro;
		this->calibrated = false;
		(this->dt) = dt;
		clock_gettime(CLOCK_REALTIME,&oldT);
		clock_gettime(CLOCK_REALTIME,&newT);
		(this->scale_factor) = 0.065;
		gyro_bias = {0};
		struct termios newtio;

		printf("opening usb port for imu...\n");
		int port; /* File descriptor for the port */
		port = open(PATH2IMU.c_str(), O_RDWR | O_NOCTTY);

		if(port > -1){ cout << "opened port successfully: " << PATH2IMU <<", Port number: " << port << endl; }
		else         { cout << "unable to open port: "      << PATH2IMU << endl; }
		//sets the parameters associated with the terminal
		//from the termios structure referred to by newtio.
		tcgetattr(port, &newtio);
		//set input/output baudrate
		cfsetospeed(&newtio, BAUDRATE_IMU);
		cfsetispeed(&newtio, BAUDRATE_IMU);

		//set character size mask.
		//set the number of data bits.

		//CSIZE flag is a mask that specifies the number of bits per byte for both transmission 
		//and reception. This size does not include the parity bit, if any. The values for the 
		//field defined by this mask are CS5, CS6, CS7, and CS8, for 5, 6, 7,and 8 bits per byte, respectively
		newtio.c_cflag &= ~CSIZE;
		newtio.c_cflag |= CS8;     //8 bits/byte

		//set the number of stop bits to 1
		newtio.c_cflag &= ~CSTOPB;

		//Set parity to None
		newtio.c_cflag &=~PARENB;

		//set for non-canonical (raw processing, no echo, etc.)
		newtio.c_iflag = IGNPAR; // ignore parity check close_port(int
		newtio.c_oflag = 0; // raw output
		newtio.c_lflag = 0; // raw input (this puts us in non-canonical mode!)


		//Time-Outs -- won't work with NDELAY option in the call to open
		//Will read until recieved a minimum of 26 bytes, no time limit
		newtio.c_cc[VMIN]  = (this->data_size);// DATASIZE   // block reading until RX x characers. If x = 0, it is non-blocking.
		newtio.c_cc[VTIME] = 0;   // Inter-Character Timer -- i.e. timeout= x*.1 s

		//Set local mode and enable the receiver
		newtio.c_cflag |= (CLOCAL | CREAD);

		//Set the new options for the port...
		//TCSANOW - options go into affect immediately
		int status = tcsetattr(port, TCSANOW, &newtio);
		if (status != 0){ //For error message
		printf("Configuring comport failed\n");
		//return status;
		}
		 if (port > 0) printf("Done!\n");
		 else printf("Fail to open usb port!\n");

		 (this->port) = port;

	} 
    
    int calibrate(void)
	{

	//calculates gyro biases and psi heading
	//returns 1 if imu is not disturbed during calibration, -1 if not
		State gyro_bias = {0.0};

		int disable_motors = 1;
		int i = 0;
		timespec start, current;
		clock_gettime(CLOCK_REALTIME,&start);
		clock_gettime(CLOCK_REALTIME,&current);

		double seconds = UTILITY::timespec2float(time_diff(start, current));

		State imu_data = {0};

		while(seconds < 5)
		{
			int suc_read = get_imu_data(imu_data);
			//suc_read: -1: first byte wrong or failed read()
				  //-2: no file descriptors ready to read
				  //-3: select() returned an error()
				  // 1: file descriptor ready, successful read        
			if(suc_read<0);// printf("printing old values\n \n \n \n \n \n");
			else if(suc_read == 1)
			{
				if((imu_value_check(imu_data) == -1)) disable_motors = -1;
				gyro_bias.theta_dot += imu_data.theta_dot;
				gyro_bias.phi_dot   += imu_data.phi_dot;
				gyro_bias.psi_dot   += imu_data.psi_dot;
				gyro_bias.psi       += imu_data.psi;
				i++;
				//print_data(imu_data);
			}

			clock_gettime(CLOCK_REALTIME,&current);
			seconds = timespec2float(time_diff(start, current));
		}

		printf("gyro_bias (averaged) :  theta_dot: %5.3f  phi_dot: %5.3f  psi_dot: %5.3f psi: %5.3f  iterations: %i  \n\n", gyro_bias.theta_dot/i, gyro_bias.phi_dot/i, gyro_bias.psi_dot/i,gyro_bias.psi/i, i);
		gyro_bias.theta_dot = gyro_bias.theta_dot/i;
		gyro_bias.phi_dot   = gyro_bias.phi_dot/i;
		gyro_bias.psi_dot   = gyro_bias.psi_dot/i;
		gyro_bias.psi       = gyro_bias.psi/i;

		this->calibrated = true;
		(this->gyro_bias) = gyro_bias;
		printf("IN CALIBRATE \n\n");
		return disable_motors;
	} 
    
    int imu_check(const State& imu_data)
	{
		int crazy_value = 1;
		int angle_cap = 360;
		int gyro_cap = 1200;

		if( (imu_data.theta > angle_cap) || (imu_data.theta <  -angle_cap) )              crazy_value = -8;
		else if( (imu_data.phi > angle_cap) || (imu_data.phi < -angle_cap) )              crazy_value = -9;
		else if( (imu_data.psi > 2000) || (imu_data.psi < -2000) )                        crazy_value = -10;
		else if( (imu_data.theta_dot > gyro_cap) || (imu_data.theta_dot < -gyro_cap) )    crazy_value = -11;
		else if( (imu_data.phi_dot > gyro_cap) || (imu_data.phi_dot < -gyro_cap) )        crazy_value = -12;
		else if( (imu_data.psi_dot > gyro_cap) || (imu_data.psi_dot < -gyro_cap) )        crazy_value = -13;

		return crazy_value;
	}

    int imu_value_check(State& imu_data)
	{
	    bool crazy_value = false;
		if( (imu_data.theta > 10) || (imu_data.theta <  -10) )              crazy_value = true;
		else if( (imu_data.phi > 10) || (imu_data.phi < -10) )              crazy_value = true;
		else if( (imu_data.psi > 360) || (imu_data.psi < -360) )            crazy_value = true;
		else if( (imu_data.theta_dot > 100) || (imu_data.theta_dot < -100) )crazy_value = true;
		else if( (imu_data.phi_dot > 100) || (imu_data.phi_dot < -100) )    crazy_value = true;
		else if( (imu_data.psi_dot > 100) || (imu_data.psi_dot < -100) )    crazy_value = true;

	    if(crazy_value) {
		    printf("\n\n\x1b[31mVALUE OUT OF NORMAL RANGE DURING IMU CALIBRATION:\x1b[0m");
		    print_data(imu_data);
		    //state2rawBytes(imu_data);
		    printf("\n\x1bPaused for 5 seconds if you want to exit before start: motors will be disabled\x1b[0m");
		    printf("\n\n");
		    usleep(1000000*5);
		    return -1;
		}

	    else return 1;

	}

    void print_data(const State& imu_data)
	{ 
	    printf("theta: %f  phi: %f  psi_mag: %f psi_integ: %f  theta_dot: %f  phi_dot: %f  psi_dot: %f\n\n", imu_data.theta, imu_data.phi, imu_data.psi, imu_data.psi_gyro_integration, imu_data.theta_dot, imu_data.phi_dot, imu_data.psi_dot); 
	 
	}
    void setDt(float dt)
	{
		(this-> dt) = dt;
	}
    void findDt(void);
    void print_raw_bytes(const unsigned char arr[], int arr_length)
	{
		printf("Bytes: \n");
		for(int i = 0; i < arr_length; i++) printf("%i : %04x ,", i, arr[i]);
		printf("\n\n\n");
	}
    void state2rawBytes(const State& imu_data)
	{
	    printf("theta: %04x   phi: %04x  psi: %04x \n  theta_dot: %04x  phi_dot: %04x  psi_dot: %04x \n\n", (unsigned char) imu_data.theta, (unsigned char) imu_data.phi, (unsigned char) imu_data.psi, (unsigned char) imu_data.theta_dot, (unsigned char) imu_data.phi_dot, (unsigned char) imu_data.psi_dot);
	}
};

#endif
// __IMU_INCLUDED__
