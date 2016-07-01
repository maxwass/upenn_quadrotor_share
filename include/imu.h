//=================================
// include guard
#ifndef IMU
#define IMU

//=================================
// included dependenciesa
#include "data_structs.h"
#include "logger.h"
#include "utility.h"
#include "psi.h"
#include "altitude.h"

#include <iostream>
#include <stdio.h>   /* Standard input/output definitions */
#include <iostream>
#include <stdlib.h>
#include <unistd.h>  /* UNIX standard function definitions */
#include <sys/time.h>
#include <time.h>
#include <math.h>
#include <fcntl.h>   /* File control definitions */
#include <termios.h> /* POSIX terminal control definitions, Unix API for terminal I/O */
#include <string.h>

#include <timer.h>

#define BAUDRATE_IMU B230400

using namespace std;

class Imu {
	int port, data_size;
        bool first_loop;
        float dt;
	//for select()
	fd_set read_fds;
	struct timeval no_timeout;

	//for bias
	bool calibrated = false;
	State bias= {0.0};

	//for Psi
	Psi p;
	Psi gyroEstimate;
	Psi* pg;

	Altitude altitude;

	//for findDt to integrate gyro
	Timer timer;
	
	float scale_factor = 0.065;

  public:
    int get_imu_data(State&);
    int get_imu_calibrated_data(State&);
    void unpack_data(State&, const unsigned char[]);
    Imu(std::string PATH2IMU, int DATASIZE, float dt)
	{
		(this->data_size) = DATASIZE;
		Psi *psiGyro = new Psi(0.00005);
		(this->pg) = psiGyro;
		(this->dt) = dt;
	
			printf("opening usb port for imu...\n");
			int port; /* File descriptor for the port */
			struct termios newtio;
			port = open(PATH2IMU.c_str(), O_RDWR | O_NOCTTY);

			if(port > -1) printf("opened port successfully: %s,  Port number: %i \n", PATH2IMU.c_str(), port);
			else         { cout << "unable to open port: "      << PATH2IMU << endl; }
			//sets the parameters associated with the terminal
			//from the termios structure referred to by newtio.
			tcgetattr(port, &newtio);
			//set input/output baudrate
			cfsetospeed(&newtio, BAUDRATE_IMU);
			cfsetispeed(&newtio, BAUDRATE_IMU);

			//set character size mask.
			//set the number of data bits.

			//CSIZE flag is a mask that specifies the number of bits per byte for both transmission 
			//and reception. This size does not include the parity bit, if any. The values for the 
			//field defined by this mask are CS5, CS6, CS7, and CS8, for 5, 6, 7,and 8 bits per byte, respectively
			newtio.c_cflag &= ~CSIZE;
			newtio.c_cflag |= CS8;     //8 bits/byte

			//set the number of stop bits to 1
			newtio.c_cflag &= ~CSTOPB;

			//Set parity to None
			newtio.c_cflag &=~PARENB;

			//set for non-canonical (raw processing, no echo, etc.)
			newtio.c_iflag = IGNPAR; // ignore parity check close_port(int
			newtio.c_oflag = 0; // raw output
			newtio.c_lflag = 0; // raw input (this puts us in non-canonical mode!)


			//Time-Outs -- won't work with NDELAY option in the call to open
			//Will read until recieved a minimum of DATASIZE bytes, no time limit
			newtio.c_cc[VMIN]  = (this->data_size);// DATASIZE   // block reading until RX x characers. If x = 0, it is non-blocking.
			newtio.c_cc[VTIME] = 0;   // Inter-Character Timer -- i.e. timeout= x*.1 s

			//Set local mode and enable the receiver
			newtio.c_cflag |= (CLOCAL | CREAD);

			//Set the new options for the port...
			//TCSANOW - options go into affect immediately
			int status = tcsetattr(port, TCSANOW, &newtio);
			if (status != 0){ //For error message
			printf("Configuring comport failed\n");
			//return status;
			}
			 if (port > 0) printf("Done!\n");
			 else printf("Fail to open usb port!\n");

			 (this->port) = port;

	} 
    
int calibrate(void)
{

	//calculates gyro biases and psi heading
	//returns 1 if imu is not disturbed during calibration, -1 if not
	State bias = {0.0};

	int disable_motors = 1;
	int i = 0;
	Timer calibrateTimer;

	State imu_data = {0};

	while(calibrateTimer.timeSinceStart() < 5)
	{
		int suc_read = get_imu_data(imu_data);
		//suc_read: -1: first byte wrong or failed read()
			  //-2: no file descriptors ready to read
			  //-3: select() returned an error()
			  // 1: file descriptor ready, successful read        
		if(suc_read<0);// printf("printing old values\n \n \n \n \n \n");
		else if(suc_read == 1)
		{
			if((imu_value_check(imu_data) == -1)) disable_motors = -1;
			bias.theta_dot += imu_data.theta_dot;
			bias.phi_dot   += imu_data.phi_dot;
			bias.psi_dot   += imu_data.psi_dot;
			bias.psi_magn_continuous  += imu_data.psi_magn_continuous;
			bias.altitude_raw	  += imu_data.altitude_raw;
			//printf("bias (averaged) :  theta_dot: %5.3f  phi_dot: %5.3f  psi_dot: %5.3f psi_magn_contin: %5.3f altitude: %5.3f iterations: %i  \n\n", bias.theta_dot/i, bias.phi_dot/i, bias.psi_dot/i,bias.psi_magn_continuous/i, bias.altitude_raw/i, i);
			i++;
			//print_data(imu_data);
		}
	}

	printf("bias (averaged) :  theta_dot: %5.3f  phi_dot: %5.3f  psi_dot: %5.3f psi_magn: %5.3f  iterations: %i  \n\n", bias.theta_dot/i, bias.phi_dot/i, bias.psi_dot/i,bias.psi_magn_continuous/i, i);
	bias.theta_dot   = bias.theta_dot/i;
	bias.phi_dot      = bias.phi_dot/i;
	bias.psi_dot      = bias.psi_dot/i;
	bias.altitude_raw = bias.altitude_raw/i;
	bias.psi_magn_continuous   = bias.psi_magn_continuous/i;

	this->calibrated = true;
	(this->bias) = bias;
	return disable_motors;
} 

int imu_value_check(State& imu_data)
{
	bool crazy_value = false;
	if( (imu_data.theta > 10) || (imu_data.theta <  -10) )              crazy_value = true;
	else if( (imu_data.phi > 10) || (imu_data.phi < -10) )              crazy_value = true;
	else if( (imu_data.psi > 360) || (imu_data.psi < -360) )            crazy_value = true;
	else if( (imu_data.theta_dot > 100) || (imu_data.theta_dot < -100) )crazy_value = true;
	else if( (imu_data.phi_dot > 100) || (imu_data.phi_dot < -100) )    crazy_value = true;
	else if( (imu_data.psi_dot > 100) || (imu_data.psi_dot < -100) )    crazy_value = true;

	if(crazy_value) {
	    printf("\n\n\x1b[31mVALUE OUT OF NORMAL RANGE DURING IMU CALIBRATION:\x1b[0m");
	    //print_data(imu_data);
	    //state2rawBytes(imu_data);
	    printf("\n\x1bPaused for 5 seconds if you want to exit before start: motors will be disabled\x1b[0m");
	    printf("\n\n");
	    usleep(1000000*5);
	    return -1;
	}

	else return 1;

}

void print_data(const State& imu_data)
{ 
	    printf("read freq: %f, theta: %f  phi: %f  theta_dot: %f  phi_dot: %f  psi_dot: %f\n", 1/imu_data.dt, imu_data.theta, imu_data.phi, imu_data.theta_dot, imu_data.phi_dot, imu_data.psi_dot);

	    printf("psi_mag_raw: %f, psi_mag_contin: %f, psi_mag_contin_cal: %f, psi_integ: %f \n", imu_data.psi_magn_raw,imu_data.psi_magn_continuous, imu_data.psi_magn_continuous_calibrated, imu_data.psi_gyro_integration); 
	 
	    printf("Altitude Calibrated: %f, Altitude Raw: %f, Altitude Deriv %f \n\n", imu_data.altitude_calibrated, imu_data.altitude_raw, imu_data.altitude_deriv);
}
void setDt(float dt)
{
	(this-> dt) = dt;
}
State getBias(void)
{
	State bias2return = {0.0};
	bias2return.theta_dot = bias.theta_dot;
	bias2return.phi_dot   = bias.phi_dot;
	bias2return.psi_dot   = bias.psi_dot;
	bias2return.theta_dot = bias.theta_dot;
	bias2return.altitude_raw = bias.altitude_raw;
	
	return bias2return;
}

void findDt(void);
void print_raw_bytes(const unsigned char arr[], int arr_length)
{
	printf("Bytes: \n");
	for(int i = 0; i < arr_length; i++) printf("%i : %04x ,", i, arr[i]);
	printf("\n\n\n");
}
void state2rawBytes(const State& imu_data)
{
	    printf("theta: %04x   phi: %04x  psi: %04x \n  theta_dot: %04x  phi_dot: %04x  psi_dot: %04x \n\n", (unsigned char) imu_data.theta, (unsigned char) imu_data.phi, (unsigned char) imu_data.psi, (unsigned char) imu_data.theta_dot, (unsigned char) imu_data.phi_dot, (unsigned char) imu_data.psi_dot);
}
};

#endif
// __IMU_INCLUDED__
