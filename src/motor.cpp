#include "motor.h"
//need sudo to run exec
//#define MOTOR_PATH "/dev/i2c-10"

motor::motor(std::string PATH2MOTOR, int motor_id, int i2c_address)
{
    printf("Motor object is being created, motor = %i \n ", motor_id);
    this -> motor_id = motor_id;
    this -> i2c_address = i2c_address;
    
    i2c_handle = open_i2c(PATH2MOTOR);

    ioctl(i2c_handle,I2C_SLAVE,i2c_address);

}

int motor::open_i2c(std::string PATH2MOTOR){
    printf("opening i2c port...");

    int handle = open(PATH2MOTOR.c_str(),O_RDWR);

    if (handle > 0) printf("Done!: file descriptor: %i \n", handle);
    else            printf("Fail to open port \n");

    return handle;
}
 
uint8_t  motor::get_force( void )
{   
	return force;
}

int motor::ensure_valid_force(int force_in)
{   //check if requested force of this motor is in the acceptable bounds
    // if not cap it at the max/min
    if(force_in > max_force) {return max_force;}
    if(force_in < min_force) {return min_force;}
    return force_in;
}

int motor::which_motor(void)
{
	return motor_id;
}

void motor::send_force_i2c(bool CONTROLLER_RUN)
{
	//Input/Output control: send to i2c_address
	ioctl(i2c_handle,I2C_SLAVE,i2c_address);

	 uint8_t f = 0;

	if(CONTROLLER_RUN) f = this->get_force();
	int success_write = write(i2c_handle, &f, 1);

	if(success_write < 0) printf("Failed to write to motor: %i \n", motor_id);  

	
	calcDt(oldT,newT);
}
int motor::send_motor_data(int force_in, bool CONTROLLER_RUN)
{
	// select returns the number of fd's ready
	FD_ZERO(&write_fds);
	FD_SET(i2c_handle, &write_fds);
	no_timeout.tv_sec  = 0;
	no_timeout.tv_usec = 0;


	int num_fds = select(i2c_handle+1, NULL, &write_fds, NULL, &no_timeout);
	int returnval = 0;
	
	if(first_call)
	{
		first_call = false;
		clock_gettime(CLOCK_REALTIME,&oldT);
	}

	//No data ready to read
	if(num_fds == 0)
	{
		returnval= -1;
		//printf("No File_Descriptor Available to write! \n");
		return -1;
	}
	//select returned an error
	else if(num_fds ==-1)
	{
		returnval= -2;
		return -2;
	}
	else if(num_fds ==1)
	{
		this->force = ensure_valid_force(force_in);
    		
		send_force_i2c(CONTROLLER_RUN);
		returnval = num_fds;
	}

	

	return returnval;


}
//timer functions
float motor::calcDt(timespec& oldT, timespec& newT)
{
                //track dt between reads
                clock_gettime(CLOCK_REALTIME, &newT);
                float dt = UTILITY::timespec2float(UTILITY::time_diff(oldT, newT));
                clock_gettime(CLOCK_REALTIME ,&oldT);
                calc_dt = dt;
                return dt;
}

float motor::getDt(void)
{
                return this->calc_dt;
}

float motor::timeSinceLastRead(void)
{

                timespec currentTime;
                clock_gettime(CLOCK_REALTIME,&currentTime);
                //printf("xbee time %f \n",UTILITY::timespec2float(UTILITY::time_diff(this->oldT, currentTime)));
                return UTILITY::timespec2float(UTILITY::time_diff(this->oldT, currentTime));

}

/*
int main(void)
{	
	timespec oldT, newT;
  	float loop_dt;
	clock_gettime(CLOCK_REALTIME,&oldT);
        int returnVal;
	motor motor1("/dev/i2c-10",1, 0x2c);
	motor motor2("/dev/i2c-10",1, 0x29);
	motor motor3("/dev/i2c-10",1, 0x2b);
	motor motor4("/dev/i2c-10",1, 0x2a);

	int i = 10;


	while(1)
	{
		returnVal = motor1.send_motor_data(100,true);
		returnVal = motor2.send_motor_data(50,true);
		returnVal = motor3.send_motor_data(10,true);
		returnVal = motor4.send_motor_data(0,true);


		clock_gettime(CLOCK_REALTIME,&newT);
		loop_dt = UTILITY::calcDt(oldT, newT);
		
		printf("returnVal: %i, loop frequency: %f, write frequency: %f, time since last read: %f  \n\n", returnVal, 1/loop_dt,1/motor1.getDt(), motor1.timeSinceLastRead());

	}

}
*/
