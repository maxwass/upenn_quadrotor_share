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
#ifndef XBEE1_H
#define XBEE1_H

#include "utility.h"
#include "data_structs.h"
//#include "logger.h"

#include <pthread.h>
#include <termios.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
  // #include "serial1.h"
#include <sys/time.h>
#include <time.h>
#include <iostream>
#include <curses.h> //for getch()

#define PI 3.14159265359
#define XBEE_START_BYTE 0xFD

#define num_bytes_per_read 1
using namespace std;

#define BAUDRATE_XBEE  B115200


//int onesec = 1000000;

class Xbee {
        private:
        int port;
        struct termios newtio;
        timespec oldT, newT;
        float calc_dt;
        std::string PATH2XBEE;
	int DATASIZE;

        bool foundFirstByte = false, foundLastByte = false;
        int index = 0;

        long num_fds_n1 = 0, num_fds_0 = 0, num_fds_1 = 0, num_fds_p = 0;

        fd_set read_fds;
        struct timeval no_timeout;

        public:
        Xbee(std::string PATH2XBEE, int DATASIZE);
        int open_port(std::string PATH2XBEE, int DATASIZE);
        int get_xbee_data(Angles& joystick_des_angles, uint8_t& joystick_thrust, uint8_t& flight_mode);
        int get_xbee_helper(Angles& joystick_des_angles, uint8_t& joystick_thrust, uint8_t& flight_mode);
        int checksum(const uint8_t arr[], int arr_length);
	void print_raw_bytes(const uint8_t arr[], int arr_length);
	void printData(Angles& joystick_des_angles, uint8_t& joystick_thrust, uint8_t& flight_mode);
	void unpack_joystick_data(Angles& joystick_des_angles, uint8_t& joystick_thrust, uint8_t& flight_mode, uint8_t arr[]);
	int check_start_thrust(void);
	void get_stats(void);
        void printStats(void);
        void print_raw_bytes(const char arr[], int arr_length);
        float calcDt(void);
	float calcDt(timespec& oldT, timespec& newT);
        float getDt(void);
        void clean(void);
};











#endif
