#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H

//=================================
// included dependencies
#include <stdlib.h>
#include <sys/ioctl.h>
#include <iostream>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <stdint.h>
#include <math.h>

#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4

#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE


#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9

#define ALLLED_ON_L 0xFA
#define ALLLED_ON_H 0xFB
#define ALLLED_OFF_L 0xFC
#define ALLLED_OFF_H 0xFD

using namespace std;

class motor
{
private:
       int ensure_valid_force(int force_in);
       int motor_id;
       uint8_t i2c_address;
       uint8_t i2c_handle;
       uint8_t force; //8 bit int
       const static uint8_t max_force = 255;
       const static uint8_t min_force = 0;

       fd_set write_fds;
       struct timeval no_timeout;

public:
	motor(int motor_id, int i2c_address);

	uint8_t     open_motors_i2c(std::string PATH2MOTOR, uint8_t addr, float freq);
	void        send_force_i2c(uint8_t channel, uint16_t on, uint16_t off);
	int         send_motor_data(void);
	void        set_force( int force_in, bool CONTROLLER_RUN );
	int	    get_force( void );
	int         ensure_valid_force(int force_in);
	static void write8(uint8_t addr, uint8_t d, uint8_t handle);
	static uint8_t     read8(uint8_t addr, uint8_t handle);
	

};



#endif
