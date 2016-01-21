#include "motor.h"
//need sudo to run exec

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

#define MOTOR_PATH "/dev/i2c-10"

int motor::open_i2c(void){
    cout << "opening i2c port...";
    int handle = open(MOTOR_PATH,O_RDWR);

    if (handle > 0) {cout << "Done!: file descriptor: " << handle << endl;}
    else            {cout << "Fail to open port" << endl;}

    return handle;
}

// Member functions definitions including constructor
motor::motor(int motor_id, int i2c_address)
{
    cout << "Motor object is being created, motor = " << motor_id << endl;
    this -> motor_id = motor_id;
    this -> i2c_address = i2c_address;
    i2c_handle = open_i2c();
    send_force_i2c();
}

void motor::set_force( int force_in, bool CONTROLLER_RUN )
{ //when setting force, check that ...
    //the motors are allowed to run (CONTROLLER_RUN flag is true)
    //the force is within acceptable bounds

    //obtain lock
   boost::unique_lock<boost::mutex> scoped_lock(mutex_force);
    
    if(CONTROLLER_RUN) force = ensure_valid_force(force_in);
    else force = 0; 

//           if(CONTROLLER_RUN) { 
//        force = ensure_valid_force(force_in);
//        send_force_i2c();
//    }
//    else{ shut_down(); }
}
 
uint8_t* motor::get_force( void )
{  boost::unique_lock<boost::mutex> scoped_lock(mutex_force);
   return &force;
}

int motor::ensure_valid_force(int force_in)
{   //check if requested force of this motor is in the acceptable bounds
    // if not cap it at the max/min
    if(force_in > max_force) {return max_force;}
    if(force_in < min_force) {return min_force;}
    return force_in;
}

int motor::which_motor(void){
	return motor_id;
}

void motor::send_force_i2c(void){
    //Input/Output control: send to i2c_address
    ioctl(i2c_handle,I2C_SLAVE,i2c_address);

    //write(int fd, const void *buf, size_t count)
        //write() writes up to count bytes from the buffer 
        //pointed buf to the file referred to by the file 
        //descriptor fd.
    //uint8_t f = this->get_force();

    int success_write = write(i2c_handle, (this->get_force()), 1);//add &force back

    if(success_write < 0) {
        cout << "Failed to write to motor: " << motor_id << endl;
    }

}


int open_i2c(void){
    cout << "opening i2c port...";
    int handle = open(MOTOR_PATH,O_RDWR);

    if (handle > 0) {cout << "Done!: value is " << handle << endl;}
    else            {cout << "Fail to open i2c port!" << endl;}

    return handle;
}

