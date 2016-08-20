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
float theta, phi, psi, psi_contin, psi_magn_raw, psi_magn_continuous, psi_magn_continuous_calibrated, theta_dot, phi_dot, psi_dot;
float psi_cal, psi_contin_cal, theta_dot_cal, phi_dot_cal, psi_dot_cal;
float theta_uncal, phi_uncal, theta_dot_uncal, phi_dot_uncal, psi_dot_uncal;
float psi_gyro_integration, psi_gyro_integration_cal;

float altitude_raw_old, altitude_raw, altitude_calibrated, altitude_deriv, altitude_integral;
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
float desired_value;
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
float kp_altitude, kd_altitude, ki_altitude;
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
float m1_freq, m2_freq, m3_freq, m4_freq;
} Motor_forces;


typedef struct OpticalFlow {
float pix_velocity_x, pix_velocity_y, pix_position_x, pix_position_y, dt;
} OpticalFlow;

typedef struct data_log {
Times time_struct;
float time;
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
float xbee_dt;

OpticalFlow opticalflow;
OpticalFlow opticalflow_vo;

Distances sonar_distances;

RepForces scales;
RepForces repulsive_forces;

SonarTest sonar_test;
int thrust;

float altitude, altitude_desired;
Errors altitude_errors;
float filter_altitude;
Errors filter_altitude_errors;

Errors pos_x_sonar_errors;
float desired_pos_x_dist;

Errors neg_x_sonar_errors;
float desired_neg_x_dist;

Errors pos_y_sonar_errors;
float desired_pos_y_dist;

Errors neg_y_sonar_errors;
float desired_neg_y_dist;

Errors up_sonar_errors;
float desired_up_dist;

Errors down_sonar_errors;
float desired_down_dist;

} Data_log;

#endif
// __DATA_STRUCTS_H_INCLUDED__
