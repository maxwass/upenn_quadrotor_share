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
#ifndef CONTROLLER_H
#define CONTROLLER_H

//=================================
// included dependencies
#include "motor.h"   //#include because controller contains a motor object
//#include "imu_inv.h"
#include "data_structs.h" // user defined structs (state, control_command,gains, desired_angles)

#include "vicon.h" // change to xbee.h
#include "logger.h"
#include "imu.h"
#include "utility.h"
#include "sonar.h"
#include "xbee1.h"

#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdint.h>
//#include "receiver.h"
#include <time.h>
#include <sys/time.h>
#include <curses.h>
#include <ncurses.h>
#include <queue>
#include <vector>
#include <ctime>
#include <iostream>

//time utility change

#define diff time_diff
#define tv2float timespec2float

#define NUM_THREADS 3
#define PI 3.14159265359

//To comply with NCURSES create macro to go back and forth between printf and required printw 
//#define printf(...) printw(__VA_ARGS__)
//=================================

double Ct=0.013257116418667*10;
double d=0.169;
//signal frequency = 1/frequency = frequency in Hz
int onesecond = 1000000;
int frequency = 200;
int signal_frequency = onesecond/frequency;
float delta_position = 0.1;
int max_thrust = 460;
int delta_thrust = 30;
float delta_time = 500000; // .5 ms
//The address of i2c: {between +X/+Y, -X/+Y, -X/-Y, +X/-Y}                                                                                                                           
//int address[4] = {0x2b, 0x2a, 0x2c, 0x29};
int address[4] = {0x2e, 0x30, 0x2d, 0x2f};// this is not correct

//function prototypes
void *command_input(void *thread_id);
void *control_stabilizer(void *thread_id);
void *motor_signals(void *thread_id);
void init(void);
void start_motors(void);
void stop_motors(void);
void controller_on_off(bool& CONTROLLER_RUN);
void display_on_off(bool& DISPLAY_RUN);
State_Error error_vicon(State_Error& error, const Vicon& pos_filt, const Vicon& vel_filt, const Positions& desired_positions, const Times& times);
void desired_angles_calc(Angles& desired_angles, const State_Error& error, const Gains& gains);
State error_imu(const State& imu_data, const Angles& desired_angles);
Control_command thrust(const State& imu_error, const State_Error& vicon_error, const Control_command& U_trim, const int joystick_thrust, const Gains& gains);
void set_forces(const Control_command& U, double Ct, double d);
Vicon vicon_velocity(Vicon& current, Vicon& old);
void log_data(const Distances& sonar_distances, const float& dt, const Vicon& new_vicon, const Vicon& new_vicon_vel, const Vicon& new_filt_vicon, const Vicon& new_filt_vicon_vel, const State_Error& vicon_error, const State& imu_data, const State& imu_error, const Angles& desired_angles);
void display_info(const Distances& sonar_distances, const int suc_read,  const State& imu_data, const State_Error& vicon_error, const State& imu_error, const Control_command& U, const Vicon& vicon, const Vicon& vicon_filt, const Vicon& vicon_vel, const Vicon& vicon_vel_filt, const Angles& desired_angles, const int joystick_thrust, const int flight_mode, const Times& times, const Times& time_m);
void configure_threads(void);

void set_timespec(timespec& x, timespec& y){
//set x to equal y
             x.tv_sec = y.tv_sec;
             x.tv_nsec = y.tv_nsec;
}

void time_calc(Times& times){
    //get current time, swap current and past, calc delta_t
    //printf("current time: %.8f  old time: %.8f  delta_t: %.8f\n", tv2float(times.current), tv2float(times.old), tv2float(times.delta));
 
    //update current time
    clock_gettime(CLOCK_REALTIME,&(times.current));
    times.delta = diff(times.old, times.current);

    //shift times back: set time_old_old to time_old
    set_timespec(times.old_old, times.old);

    //set time_old to current time
    set_timespec(times.old, times.current);

    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];
    time (&rawtime);
    timeinfo = localtime(&rawtime);
    strftime(buffer,80,"%d-%m-%Y %I:%M:%S",timeinfo);

    //times.date_time = buffer;
    strcpy(times.date_time,buffer); // myStr.c_str());
}
void set_initial_times(Times& times){                                                                                                                                                                       
        clock_gettime(CLOCK_REALTIME,&(times.current));
        clock_gettime(CLOCK_REALTIME,&(times.old));
        clock_gettime(CLOCK_REALTIME,&(times.old_old));
        clock_gettime(CLOCK_REALTIME,&(times.delta));
}

void set_gains(Gains& gains){
    gains.kp_theta = 19.0; //21.0;
    gains.kd_theta = 0.33; //0.32;
     
    gains.kp_phi = gains.kp_theta; //21.0;
    gains.kd_phi = gains.kd_theta; //0.37;
     
    gains.kp_psi = 20;//5.2;
    gains.kd_psi = 0.2;//0.3; 
 
    gains.kp_x = 19.5;
    gains.kd_x = 2.7;
    gains.ki_x = .05;
 
    gains.kp_y = gains.kp_x;
    gains.kd_y = gains.kd_x;
    gains.ki_y = gains.ki_x;

    gains.kp_z = 12.0;
    gains.kd_z = 5.0;
    gains.ki_z = 5.0;

}
#endif 
// __CONTROLLER_H_INCLUDED__
