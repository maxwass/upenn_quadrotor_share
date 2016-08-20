#include "motor_test_1.h"
//g++ motor_test.cpp -I ../include -std=c++11
//need sudo to run exec
//#define MOTOR_PATH "/dev/i2c-10"  //may have to do this instead of passing in!

motor::motor(int motor_id, int i2c_address)
{
    printf("Motor object is being created, motor = %i \n", motor_id);
    this -> motor_id = motor_id;
    this -> i2c_handle = i2c_address;
}

void motor::write8(uint8_t addr, uint8_t d, uint8_t handle)
{

          uint8_t data[2];
          data[0] = addr;
          data[1] = d;
          if(write(handle,data,2)!= 2)     printf("error using write8 \n");
          else                             printf("success using write8 \n");

}

uint8_t motor::read8(uint8_t addr, uint8_t handle)
{
          uint8_t buf[2];
          buf[1] = addr;
          if(write(handle,buf,1) != 1) printf("error when trying to set read register");

          if(read(handle,buf,1) != 1)  printf("failed to read from i2c bus");
          return buf[1];
}

uint8_t motor::open_motors_i2c(std::string PATH2MOTOR, uint8_t addr, float freq)
{
        printf("opening i2c port...");
        uint8_t handle = open(PATH2MOTOR.c_str(),O_RDWR);

        if (handle > 0) printf("Done!: file descriptor: %i \n",handle);
        else            printf("Fail to open port \n");

        ioctl(handle, I2C_SLAVE, addr); // set slave address    

        //write8(PCA9685_MODE1, 0x0); // this is reset in motordriver.cpp

        uint8_t data[2];
        data[0] = PCA9685_MODE1;
        data[1] = 0x0;
        if(write(handle,data,2)!= 2) printf("er:or in resetting chip \n");
        else                                printf("successi in ressetting chip \n");

        freq *= 0.9;  // Correct overshoot in the frequency setting (see issue #11).
        float prescaleval = 25000000;
        prescaleval /= 4096;
        prescaleval /= freq;
        prescaleval -= 1;
        uint8_t prescale = floor(prescaleval + 0.5);

        uint8_t oldmode = read8(PCA9685_MODE1, handle);
        uint8_t newmode = (oldmode&0x7F) | 0x10; // sleep
        motor::write8(PCA9685_MODE1,     newmode,     handle); // go to sleep
        motor::write8(PCA9685_PRESCALE, prescale,    handle); // set the prescaler
        motor::write8(PCA9685_MODE1,    oldmode,     handle);
        usleep(5000);
        motor::write8(PCA9685_MODE1, oldmode | 0xa1, handle);

        return handle;
}
void motor::send_force_i2c(uint8_t channel, uint16_t on, uint16_t off) {

        //NEED TO USE IDENTIFIER FOR THE MOTOR: channel id's the motor 0-3
        uint8_t buf[5];
        buf[0] = LED0_ON_L + 4*channel;
        buf[1] = on;
        buf[2] = on >> 8;
        buf[3] = off;
        buf[4] = off >> 8;
        write(i2c_handle,buf,5);
}

int motor::send_motor_data(void)
{
    // select returns the number of fd's ready
    FD_ZERO(&write_fds);
    FD_SET(this->i2c_handle, &write_fds);
    no_timeout.tv_sec  = 0;
    no_timeout.tv_usec = 0;

   int num_fds = select(i2c_handle+1, NULL, &write_fds, NULL, &no_timeout);
int returnVal = 0;

 //No data ready to read 
        if(num_fds == 0)
        {
                returnVal= -1;
                return -1;
        }
        //select returned an error 
        else if(num_fds ==-1)
        {
                returnVal= -2;
                return -2;
        }
        else if(num_fds ==1)
        {
                //channel/motor_id tells us which motor should get signal. On/Off is the pwm frequency. Need to map this from origianl (0-255) to new (700-???)
                motor::send_force_i2c(motor_id, 5,100);//on, off);
                returnVal = num_fds;
        }


        return returnVal;
}
void motor::set_force( int force_in, bool CONTROLLER_RUN )
{
        //when setting force, check that ...
        //the motors are allowed to run (CONTROLLER_RUN flag is true)
        //the force is within acceptable bounds

        if(CONTROLLER_RUN) (this->force) = ensure_valid_force(force_in) ;
        else force = 0;
}
int get_force( void )
{
	return this->force;
}

int motor::ensure_valid_force(int force_in)
{
        //check if requested force of this motor is in the acceptable bounds
        // if not cap it at the max/min
        if(force_in > max_force) {return max_force;}
        if(force_in < min_force) {return min_force;}
        return force_in;
}


int main(void)
{
std::string PATH2MOTOR = "/dev/i2c-10";

motor motor(1,2);

}
