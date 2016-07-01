//=================================
// include guard
#ifndef MOTOR_H
#define MOTOR_H

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
#include <sys/time.h>
#include <time.h>

#include <utility.h>

using namespace std;

class motor
{
   private:
      int ensure_valid_force(int force_in);
      int motor_id;
      int i2c_address;
      const static uint8_t max_force = 255;
      const static uint8_t min_force = 0;
      uint8_t force = 0;
	
      fd_set write_fds;
      struct timeval no_timeout;

      timespec oldT, newT;
      float calc_dt;
      bool first_call = true;


   public:
   	motor(std::string PATH2MOTOR, int motor_id, int i2c_address);  // This is the constructor
      void set_force( int force_in, bool CONTROLLER_RUN );
      uint8_t  get_force( void );
      int which_motor(void);
      void send_force_i2c(bool CONTROLLER_RUN);
      int send_motor_data(int force, bool CONTROLLER_RUN);
      int open_i2c(std::string PATH2MOTOR);

      float calcDt(timespec& oldT, timespec& newT);
      float getDt(void);
      float timeSinceLastRead(void);

      int i2c_handle;
 
};
 

#endif
// __MOTOR_H_INCLUDED__
