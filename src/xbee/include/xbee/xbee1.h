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

#include <ros/ros.h>
#include "xbee/XbeeData.h"


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

	//position from raw data
	Vicon new_vicon,          old_vicon,          old_old_vicon          = {0};
	Vicon new_filt_vicon,     old_filt_vicon,     old_old_filt_vicon     = {0};

	//velocity from raw data
	Vicon new_vicon_vel,      old_vicon_vel,      old_old_vicon_vel      = {0};
	Vicon new_filt_vicon_vel, old_filt_vicon_vel, old_old_filt_vicon_vel = {0};

	Weights weights = {.7,.2,.1};



        public:
        Xbee(std::string PATH2XBEE, int DATASIZE);
        int open_port(std::string PATH2XBEE, int DATASIZE);

        int   get_xbee_data(void);
	void  unpack_vicon_data(Vicon &vicon, float arr[]);
	Vicon filter_vicon(Vicon& new_vicon, Weights& weights);
	Vicon vicon_velocity(Vicon& current, Vicon& old, float dt);
	void  pushback(Vicon& new_vicon, Vicon& old_vicon, Vicon& old_old_vicon);
	void unpack_vicon_data(Vicon &vicon, uint8_t arr[]);
	State_Error error_vicon(State_Error& error, const Vicon &desired_velocity, const Positions& desired_positions);	
	Vicon getLastVicon(void);
	Vicon getLastFiltVicon(void);
	Vicon getLastViconVel(void);
	Vicon getLastFiltViconVel(void);

	int get_xbee_data(Angles& joystick_des_angles, uint8_t& joystick_thrust, uint8_t& flight_mode);
        int get_xbee_helper(Angles& joystick_des_angles, uint8_t& joystick_thrust, uint8_t& flight_mode);
        int checksum(const uint8_t arr[], int arr_length);
	int checksum_(const uint8_t arr[], int arr_length);
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
