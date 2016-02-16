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
#ifndef IMU_INV_H
#define IMU_INV_H


//=================================
// included dependenciesa
#include "data_structs.h"
#include  "logger.h"

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

#include "utility.h"

#define BAUDRATE_INV B230400
#define PATH2IMU "/dev/ttyACM0"
#define  ivn_imu_data_size 26
//int ivn_imu_data_size = 26;

using namespace std;

//function prototypes
int open_imu_ivn_port(void);
void print_data(const State& imu_data);
int unpack_ivn_data(State& imu_data, const unsigned char arr[]);
void imu_value_check(State& imu_data);
int imu_check(const State& imu_data);
template <typename T>
T saturate(T imu_val);
void print_raw_bytes(const unsigned char arr[], int arr_length);
void state2rawBytes(const State& imu_data);
void print_stats(int last_return);
int find_gyro_bias(const int port);
int get_imu_ivn_data(const int port, State& imu_data);
float make_psi_contin();//State& old_raw_psi, State& new_raw_psi);
/*
timespec time_diff(timespec start, timespec end);
double timespec2float (const timespec& time);

timespec time_diff(timespec start, timespec end){
     timespec temp;
     if ((end.tv_nsec-start.tv_nsec)<0) {
         temp.tv_sec = end.tv_sec-start.tv_sec-1;
         temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
     } else {
         temp.tv_sec = end.tv_sec-start.tv_sec;
         temp.tv_nsec = end.tv_nsec-start.tv_nsec;
     }
     return temp;
}
double timespec2float (const timespec& time){
     return ((double) time.tv_sec + (time.tv_nsec / 1000000000.0));
}
*/
#endif
// __IMU_INV_H_INCLUDED__
