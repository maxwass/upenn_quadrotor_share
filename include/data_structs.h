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
#ifndef DATA_STRUCTS_H
#define DATA_STRUCTS_H

#define VICON_MEM_LOC "VICON_MEM_LOC"
#define IMU_MEM_LOC   "IMU_MEM_LOC"

#include <sys/time.h>
#include <time.h> // for struct timeval
#include <string>


typedef struct sonartest {
int distance;
int succ_read;
int num_fds;
char lastByte;
float dt;
int index;
bool foundFirstByte, foundLastByte;
long num_fds_n1, num_fds_0, num_fds_1, num_fds_p;
} SonarTest;


//cal = calibrated, contin = continuous
typedef struct state {
float theta, phi, psi, psi_contin, theta_dot, phi_dot, psi_dot;
float psi_cal, psi_contin_cal, theta_dot_cal, phi_dot_cal, psi_dot_cal;
float psi_gyro_integration, psi_gyro_integration_cal;
int succ_read;
float dt;
float numPsiRot;

} State ;

typedef struct repforces {
//int one, two, three, four, five, six;
float x_pos, x_neg, y_pos, y_neg, up, down;
} RepForces;



 
typedef struct distances {
//int one, two, three, four, five, six;
int x_pos, x_neg, y_pos, y_neg, up, down;

int succ_read;
float dt;
} Distances;


typedef struct angles {
float theta, phi, psi;
int succ_read;
} Angles ;

typedef struct vicon {
float x, y, z, phi, theta, psi;
} Vicon ;

typedef struct positions {
double x, y, z;
} Positions;

typedef struct velocities {
double vx, vy, vz;
} Velocities;

typedef struct errors {
float prop, deriv, integral;
} Errors;

typedef struct state_error {
errors x, y, z, phi, theta, psi;
} State_Error;

typedef struct control_command {
int thrust, estop; 
double roll_acc, pitch_acc, yaw_acc;
} Control_command;

typedef struct gains {
double kp_theta, kd_theta, kp_phi, kd_phi, kp_psi, kd_psi, kp_x, kd_x, ki_x, kp_y, kd_y, ki_y, kp_z, kd_z, ki_z;
} Gains; 

typedef struct times {
timespec current, old, old_old, delta;
char date_time[80];
} Times; 

typedef struct weights {
float newest, old, old_old;
} Weights;

typedef struct motor_forces {
int motor_1, motor_2, motor_3, motor_4;
} Motor_forces;


typedef struct data_log {
Times time;
float dt;

Vicon vicon_data;
Vicon vicon_vel;

Vicon vicon_data_filt;
Vicon vicon_vel_filt;

State_Error vicon_error;

State imu;
State imu_error;

int read_error;

Motor_forces forces;

Angles desired_angles;

Distances sonar_distances;

RepForces scales;
RepForces repulsive_forces;

SonarTest sonar_test;
} Data_log;

#endif
// __DATA_STRUCTS_H_INCLUDED__
