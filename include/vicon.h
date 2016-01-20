//=================================
// include guard
#ifndef VICON_H
#define VICON_H

//=================================
// included dependencie
#include "data_structs.h" // user defined structs (state, control_command,gains, desired_angles)
//#include "Xbee.h"
//#include "receiver.h"

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

//=================================
// forward declared dependencies
//struct State;
//#define XBEE_START_BYTE 0xFD
#define PI 3.14159265359
#define BAUDRATE B115200
#define XBEE_START_BYTE 0xFD

#define LIDAR_LITE_ADRS 0x62
#define MEASURE_VAL 0x04
#define MEASURE_REG 0x00
#define STATUS_REG  0x47
#define DISTANCE_REG_HI 0x0f
#define DISTANCE_REG_LO 0x10  
#define VERSION_REG 0x41
#define ERROR_READ -1
// Status Bits
#define STAT_BUSY               0x01
#define STAT_REF_OVER           0x02
#define STAT_SIG_OVER           0x04
#define STAT_PIN                0x08
#define STAT_SECOND_PEAK        0x10
#define STAT_TIME               0x20
#define STAT_INVALID            0x40
#define STAT_EYE                0x80
#define DISTANCE_CONT		    0x8f



void get_vicon_data(int port, Vicon& vicon_data);
void get_joystick_data(int port, Angles& joystick_des_angles, float& joystick_thrust, float& flight_mode);
void unpack_vicon_data(Vicon& vicon_data, float arr[]);
void unpack_joystick_data(Angles& joystick_des_angles, float& joystick_thrust, float& flight_mode, float arr[]);

Vicon filter_vicon_data(Vicon& new_v, Vicon& prev1_v, Vicon& prev2_v,Weights& weights);
float filt(float new_data, float old_data, float old_old_data, Weights& weights);
void pushback(Vicon& new_vicon, Vicon& old_vicon, Vicon& old_old_Vicon); 
void display_vicon_data(const Vicon& vicon_data);
int open_xbee_port();
int recieve_data(int fd_xbee, float data_received[], int data_size);
void xbee_init(void);

int main(void);



#endif 
// __VICON_H_INCLUDED__
