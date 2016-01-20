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

#include <boost/thread/mutex.hpp>

using namespace std;

class motor
{
   private:
      int ensure_valid_force(int force_in);
      int motor_id;
      int i2c_address;
      uint8_t force; //8 bit int
      const static uint8_t max_force = 255;
      const static uint8_t min_force = 0;
      boost::mutex mutex_force;

   public:
   	motor(int motor_id, int i2c_address);  // This is the constructor
      void set_force( int force_in, bool CONTROLLER_RUN );
      uint8_t* get_force( void );
      int which_motor(void);
      void send_force_i2c(void);
      int i2c_handle;
      int open_i2c(void);

 
};
 

#endif
// __MOTOR_H_INCLUDED__
