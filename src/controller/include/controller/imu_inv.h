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
int get_imu_ivn_calibrated_data(const int port, State& imu_data);

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
