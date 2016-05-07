#include "xbee1.h"
//g++ xbee1.cpp logger.cpp -I ../include -std=c++11

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
Xbee::Xbee(std::string PATH2XBEE, int DATASIZE)
{
        open_port(PATH2XBEE, DATASIZE);
}
int Xbee::open_port(std::string PATH2XBEE, int DATASIZE)
{
    //port = open(path, O_RDWR | O_NOCTTY)
    //port        - The returned file handle for the device. -1 if an error occurred
    //path        - The path to the serial port (e.g. /dev/ttyS0)
    //"O_RDWR"    - Opens the port for reading and writing
    //"O_NOCTTY"  - The port never becomes the controlling terminal of the process.

    printf("Opening a XBEE USB port...   ");
    struct termios newtio;
    int port; /* File descriptor for the port */
    port = open(PATH2XBEE.c_str(), O_RDWR | O_NOCTTY);

        //gets the parameters (options) associated with the terminal from the termios structure (newtio)
    tcgetattr(port, &newtio);
    cfsetospeed(&newtio, BAUDRATE_XBEE);
    cfsetispeed(&newtio, BAUDRATE_XBEE);

    //set the number of data bits
    newtio.c_cflag &= ~CSIZE;
    newtio.c_cflag |= CS8;     //8 bits/byte

    //set the number of stop bits to 1
    newtio.c_cflag &= ~CSTOPB;

    //Set parity to None
    newtio.c_cflag &=~PARENB;

    //set for non-canonical (raw processing, no echo, etc.)
    newtio.c_iflag = IGNPAR; // ignore parity errors
    newtio.c_oflag = 0; // raw output
    newtio.c_lflag = 0; // raw input 

    //Time-Outs -- won't work with NDELAY option in the call to open
    newtio.c_cc[VMIN]  = DATASIZE;   // block reading until DATASIZE characers. If x = 0, it is non-blocking.
    newtio.c_cc[VTIME] = 0;   // Inter-Character Timer -- i.e. timeout= x*.1 s

     //Set local mode and enable the receiver
     newtio.c_cflag |= (CLOCAL | CREAD);

    //Set the new options for the port...
    //TCSANOW - options go into affect immediately
    int status = tcsetattr(port, TCSANOW, &newtio);
    if (status != 0) printf("Configuring XBEE comport failed\n");

    (this->port) = port;
    (this->PATH2XBEE) = PATH2XBEE;
    (this->DATASIZE) = DATASIZE; //including first and last checkbytes

    if (port < 0)  printf("\n Error opening XBEE USB port: %i !!", port);
    else   	   printf("Done!\n");
    printf("	Port Name: %i \n", port);

    return port;
}
int Xbee::get_xbee_data(Angles& joystick_des_angles, uint8_t& joystick_thrust, uint8_t& flight_mode)
{
    // select returns the number of fd's ready
    FD_ZERO(&read_fds);
    FD_SET(port, &read_fds);
    no_timeout.tv_sec  = 0;
    no_timeout.tv_usec = 0;

    return get_xbee_helper(joystick_des_angles, joystick_thrust, flight_mode);
}
int Xbee::get_xbee_helper(Angles& joystick_des_angles, uint8_t& joystick_thrust, uint8_t& flight_mode)
{
   int num_fds = select(port+1, &read_fds, NULL, NULL, &no_timeout);  
   int returnVal= -10; 
   if(num_fds == -1)      (this->num_fds_n1)++; 
   else if (num_fds == 0) (this->num_fds_0)++; 
   else if (num_fds == 1) (this->num_fds_1)++;  
   else if (num_fds > 1)  (this->num_fds_p)++; 
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
		lseek(port, -(this->DATASIZE), SEEK_END);

		uint8_t  data_received[DATASIZE];
		int result = read(port, &data_received[0], DATASIZE);
		
		int16_t checksum_calc = checksum(data_received, DATASIZE);
		
		//printf("result: %i\n", result);
		//print_raw_bytes(data_received, DATASIZE);

		bool check1_correct =  (  data_received[0] == 253);
 		bool check2_correct =  (  data_received[6] == 173);
 		bool checksum_correct = (checksum_calc > 0); 	

		if(result == 0)
		{
			printf("read 0 bytes \n");
		//	clean();
			returnVal = -3;
		}

		if( (checksum_calc > 0) && (  data_received[0] == 253) && (  data_received[6] == 173) )
		{
			unpack_joystick_data(  joystick_des_angles,  joystick_thrust, flight_mode, data_received);
			calcDt();		 
			returnVal = 1;
		}
		 else
		{
			if (result == -1) printf("get_joystick_data: FAILED read from port \n");
			 //printf("\n\n\x1b[31mFIRST BYTE OR CHECKSUM WRONG:FLUSHED PORT\x1b[0m\n\n");

			tcflush(port, TCIFLUSH);

			if(checksum_calc < 0) 		      returnVal = -4;
			else if(!(  data_received[0] == 253)) returnVal = -5;
			else if(!(  data_received[6] == 173)) returnVal = -6;
			else 				      returnVal = -1;
		}
	}

	joystick_des_angles.succ_read = returnVal;
	return returnVal;
}
void Xbee::unpack_joystick_data(Angles& joystick_des_angles, uint8_t& joystick_thrust, uint8_t& flight_mode, uint8_t arr[])
{
	joystick_thrust             = (int8_t) arr[1] + 75 ; // thrust
	joystick_des_angles.phi     = (int8_t) arr[2]; // roll
	joystick_des_angles.theta   = (int8_t) arr[3]; // pitch 
	joystick_des_angles.psi     = (int8_t) arr[4]; // yaw 
	flight_mode                 = (int8_t) arr[5];

	float attenuation = 3.0;
	joystick_des_angles.phi   = ( (float) joystick_des_angles.phi   /  attenuation); 
	joystick_des_angles.theta = ( (float) joystick_des_angles.theta /  attenuation);
}

int Xbee::checksum(const uint8_t arr[], int arr_length){
         int checksum_calc = 0;
         for(int i = 0; i < arr_length-2; i++) checksum_calc+= (int8_t) arr[i];

         //combine the split 16 bit integer (two bytes: last two slots in array)
         int16_t checksum_recv =  ( (int8_t) arr[arr_length-1] << 8) | ( (int8_t) arr[arr_length-2]);

         // printf("checksum_calc: %i , checksum_recv: %i, equal?:%i\n", checksum_calc, checksum_recv, (checksum_calc == checksum_recv));
         if (checksum_calc == checksum_recv) return 1;
         else return -1;
}
int Xbee::check_start_thrust(void)
{

	if((this->port) < 0)
	{
		  printf("!!!!!NOT RECEIVING ANY JOYSTICK DATA: FAILURE TO OPEN PORT!!!!\n");
                  usleep(100000*2);
                  return -2;

	}


	timespec start_time, current_time;
    	clock_gettime(CLOCK_REALTIME,&start_time);
    	clock_gettime(CLOCK_REALTIME,&current_time);
	float elapsed_time = 0;

	Angles joystick_des_angles = {-10};
	uint8_t joystick_thrust = 0, flight_mode = 0;
	
	//read 250 values from joystick. check if the values are reasonable to start drone. if dont recieve values for 3 seconds, return nregative values
	int i = 0;
	while(i<250)
	{
		int result = this->get_xbee_data(joystick_des_angles, joystick_thrust, flight_mode);
		elapsed_time += calcDt(start_time, current_time); 
		//printf("result: %i \n", result);
	
		if(result > 0)
		{
			i++;
			if(joystick_thrust > 30 )
			{
				printf("!!!!!JOYSTICK HAS TO MUCH INITIAL THRUST (%i) - SYSTEM_RUN SET TO FALSE \n", joystick_thrust);
				this->printData(joystick_des_angles, joystick_thrust, flight_mode);
				usleep(1000000*2);
				return -1;
			}
			else
			{
				if(elapsed_time > 3)
				{
					printf("!!!!!NOT RECEIVING ANY JOYSTICK DATA!!!!\n");
					usleep(100000*2);
					return -2;
				}
			}
		}
		if(elapsed_time > 3)
                {
			printf("!!!!!NOT RECEIVING ANY JOYSTICK DATA!!!!\n");
			usleep(1000000*3);
			return -2;
                }

	
	}

	return 1;

}
void Xbee::print_raw_bytes(const uint8_t arr[], int arr_length){ 
                printf("Bytes: \n"); 
                for(int i = 0; i < arr_length; i++) printf("%i : %04x ,", i, arr[i]);//(arr[i] == 0xbd)); 
                printf("\n  1st byte: %i, should be: 253, equal?:%i  6th byte: %i should be: 173, equal?:%i \n",  arr[0], (arr[0] == 253), arr[6], (arr[6] == 173) ); 
                printf("\n"); 
}
float Xbee::calcDt(void){
                //track dt between reads
                clock_gettime(CLOCK_REALTIME,&newT);
                (this->calc_dt) = UTILITY::timespec2float(UTILITY::time_diff(oldT, newT));
                clock_gettime(CLOCK_REALTIME,&oldT);
		return (this->calc_dt);
}
float Xbee::calcDt(timespec& oldT, timespec& newT){
                //track dt between reads
                clock_gettime(CLOCK_REALTIME,&newT);
                float dt = UTILITY::timespec2float(UTILITY::time_diff(oldT, newT));
                clock_gettime(CLOCK_REALTIME,&oldT);
                return dt;
}
float Xbee::getDt(void)
{
		return this->calc_dt;
}
void Xbee::printStats(void) 
{ 
		printf("Not Ready to Read Count:  %ld, Ready to Read Count: %ld freq (Hz): %f \n\n",  (this->num_fds_0), (this->num_fds_1), 1/(this->calc_dt)); 
}
void Xbee::printData(Angles& joystick_des_angles, uint8_t& joystick_thrust, uint8_t& flight_mode)
{
                printf("Thrust: %i, Phi: %f, Theta: %f, Psi: %f, Flight Mode: %i  \n\n",joystick_thrust, joystick_des_angles.phi, joystick_des_angles.theta, joystick_des_angles.psi, flight_mode);
}
/*
int main(void)
{
		Xbee xbee("/dev/ttyUSB1",9);
		
		Angles joystick_des_angles = {0};
		uint8_t joystick_thrust = 0, flight_mode = 0;
		
		while(1)
		{

			if(xbee.get_xbee_data(joystick_des_angles, joystick_thrust, joystick_thrust) > 0) xbee.printData(joystick_des_angles, joystick_thrust, flight_mode); 
			//else xbee.printStats();
		}



}
*/
