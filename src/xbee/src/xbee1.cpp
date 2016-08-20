#include "xbee1.h"
//g++ xbee1.cpp logger.cpp -I ../include -std=c++11

Xbee::Xbee(std::string path, int DATASIZE)
{
        open_port(path, DATASIZE);
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
    (this->DATASIZE)  = DATASIZE; //including first and last checkbytes

    if (port < 0)  printf("\n Error opening XBEE USB port: %i !!", port);
    else   	   printf("Done!\n");
    printf("	Port Name: %i \n", port);

    return port;
}
int Xbee::get_xbee_data(void)
{
    // select returns the number of fd's ready
    FD_ZERO(&read_fds);
    FD_SET(port, &read_fds);
    no_timeout.tv_sec  = 0;
    no_timeout.tv_usec = 0;

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



		//CHECK BYTES MAY BE DIFFERENT FOR VICON
		bool check1_correct =  (  data_received[0] == 253);
 		//bool check2_correct =  (  data_received[DATASIZE-2] == 173);
 		bool checksum_correct = (checksum_calc > 0); 	




		if(result == 0)
		{
			printf("read 0 bytes \n");
		//	clean();
			returnVal = -3;
		}

		if( (checksum_calc > 0) && ( check1_correct ) )
		{

			calcDt();
			Vicon vicon = {0.0};
			//unpack_vicon_data(vicon,data_received);UNCOMMENT AND FIX!!!

			(this->new_vicon) 	   = vicon;
			(this->new_filt_vicon) 	   = filter_vicon(vicon, this->weights);
			
			(this->new_vicon) 	   = vicon_velocity(this->new_vicon, this->old_vicon, this->getDt()); 
			(this->new_filt_vicon_vel) = vicon_velocity(this->new_filt_vicon, this->old_filt_vicon, this->getDt());

			pushback(this->new_vicon, this->old_vicon, this->old_old_vicon);
			pushback(this->new_filt_vicon, this->old_filt_vicon, this->old_old_filt_vicon);
			pushback(this->new_vicon_vel,this->old_vicon_vel, this->old_old_vicon_vel); 	
			pushback(this->new_filt_vicon_vel, this->old_filt_vicon_vel,  this->old_old_filt_vicon_vel);

					 
			returnVal = 1;
		}
		 else
		{
			if (result == -1) printf("get_xbee_data: FAILED read from port \n");
			 //printf("\n\n\x1b[31mFIRST BYTE OR CHECKSUM WRONG:FLUSHED PORT\x1b[0m\n\n");

			tcflush(port, TCIFLUSH);

			if(checksum_calc < 0) 		      returnVal = -4;
			else if(!(  data_received[0] == 253)) returnVal = -5;  //check bytes??
			else if(!(  data_received[6] == 173)) returnVal = -6;  //check bytes??
			else 				      returnVal = -1;
		}
	}

	return returnVal;

}

Vicon Xbee::filter_vicon(Vicon& new_vicon, Weights& weights)
{
	/*Vicon filtered_vicon = {0.0};

	filtered_vicon.x =     UTILITY::filter(new_vicon.x, this->old_vicon.x, this->old_old_vicon.x, this->weights);
	filtered_vicon.y =     UTILITY::filter(new_vicon.y, this->old_vicon.y, this->old_old_vicon.y, this->weights);
	filtered_vicon.z =     UTILITY::filter(new_vicon.z, this->old_vicon.z, this->old_old_vicon.z, this->weights);
	filtered_vicon.theta = UTILITY::filter(new_vicon.theta, this->old_vicon.theta, this->old_old_vicon.theta, this->weights);
	filtered_vicon.phi =   UTILITY::filter(new_vicon.phi, this->old_vicon.phi, this->old_old_vicon.phi, this->weights);
	filtered_vicon.psi =   UTILITY::filter(new_vicon.psi, this->old_vicon.psi, this->old_old_vicon.psi, this->weights);

	
	return filtered_vicon; 
	*/

}
Vicon Xbee::vicon_velocity(Vicon& current, Vicon& old, float dt){

    Vicon velocity = {0.0};
    velocity.x     = (current.x - old.x)/dt;
    velocity.y     = (current.y - old.y)/dt;
    velocity.z     = (current.z - old.z)/dt;
    velocity.theta = (current.theta - old.theta)/dt;
    velocity.phi   = (current.phi - old.phi)/dt;
    velocity.psi   = (current.psi - old.psi)/dt;

    
    return velocity;
}
void Xbee::pushback(Vicon& new_vicon, Vicon& old_vicon, Vicon& old_old_vicon){
    old_old_vicon = old_vicon;
    old_vicon = new_vicon;
}
State_Error Xbee::error_vicon(State_Error& error, const Vicon &desired_velocity, const Positions& desired_positions)
{ 
    //proportional errors:  desired_positions - filtered_positions 
    error.x.prop = desired_positions.x - this->new_filt_vicon.x; 
    error.y.prop = desired_positions.y - this->new_filt_vicon.y; 
    error.z.prop = desired_positions.z - this->new_filt_vicon.z; 
        
    //derivative errors: desired_velocities - filtered_velocities 
    error.x.deriv = desired_velocity.x - this->new_filt_vicon_vel.x; 
    error.y.deriv = desired_velocity.y - this->new_filt_vicon_vel.y; 
    error.z.deriv = desired_velocity.z - this->new_filt_vicon_vel.z; 
        
    //integral errors: integral error + (proportional error * delta_t) 
    error.x.integral += error.x.prop * (this->getDt()); 
    error.y.integral += error.y.prop * (this->getDt()); 
    error.z.integral += error.z.prop * (this->getDt()); 

}
Vicon Xbee::getLastVicon(void)
{
	Vicon v = {0.0};
	v = this->new_vicon;
	return v;
}
Vicon Xbee::getLastFiltVicon(void) 
{
	Vicon v = {0.0};
        v = this->new_filt_vicon;
        return v;
}
Vicon Xbee::getLastViconVel(void)
{
	Vicon v = {0.0};
        v = this->new_vicon_vel;
        return v;
} 
Vicon Xbee::getLastFiltViconVel(void)
{
	Vicon v = {0.0};
        v = this->new_filt_vicon_vel;
        return v;
}
int Xbee::get_xbee_data(Angles& joystick_des_angles, uint8_t& joystick_thrust, uint8_t& flight_mode)
{
    return get_xbee_helper(joystick_des_angles, joystick_thrust, flight_mode);
}
int Xbee::get_xbee_helper(Angles& joystick_des_angles, uint8_t& joystick_thrust, uint8_t& flight_mode)
{
   int returnVal= -10; 
	uint8_t  data_received[DATASIZE];

	
	int result = read(port, &data_received[0], DATASIZE);
	int16_t checksum_calc = checksum(data_received, DATASIZE);
	
	//printf("result: %i, checksum: %i \n", result, checksum_calc);
	//print_raw_bytes(data_received, DATASIZE);

	bool check1_correct =  (  data_received[0] == 253);
	bool check2_correct =  (  data_received[6] == 173);
	bool checksum_correct = (checksum_calc > 0); 	
	//printf("data_recieved[0]: %i\n",data_received[0]);
	if(result == 0)
	{
		printf("read 0 bytes \n");
	//	clean();
		returnVal = -3;
	}

	if( (checksum_calc > 0) && (  data_received[0] == 253) && (  data_received[6] == 173) )
	{
		//printf("UNPACK \n\n");
		unpack_joystick_data(  joystick_des_angles,  joystick_thrust, flight_mode, data_received);
		calcDt();	
		//printf("checksum: %i \n",checksum_calc);
		returnVal = 1;
	}
	 else
	{
		if (result == -1) printf("get_joystick_data: FAILED read from port \n");
		 //printf("\n\n\x1b[31mFIRST BYTE OR CHECKSUM WRONG:FLUSHED PORT\x1b[0m\n\n");

		//printf("3rd if in unpack \n\n");
		tcflush(port, TCIFLUSH);

		if(checksum_calc < 0) 		      returnVal = -4;
		else if(!(  data_received[0] == 253)) returnVal = -5;
		else if(!(  data_received[6] == 173)) returnVal = -6;
		else 				      returnVal = -1;
	}


	//joystick_des_angles.succ_read = returnVal;
	return returnVal;
}
void Xbee::unpack_joystick_data(Angles& joystick_des_angles, uint8_t& joystick_thrust, uint8_t& flight_mode, uint8_t arr[])
{
	joystick_thrust             = (int8_t) arr[1] + 75; // thrust
	joystick_des_angles.phi     = (int8_t) arr[2]; // roll
	joystick_des_angles.theta   = (int8_t) arr[3]; // pitch 
	joystick_des_angles.psi     = (int8_t) arr[4]; // yaw 
	flight_mode                 = (int8_t) arr[5];

	float attenuation = 3.0;
	joystick_des_angles.phi   = ( (float) joystick_des_angles.phi   /  attenuation); 
	joystick_des_angles.theta = ( (float) joystick_des_angles.theta /  attenuation);
	
}

void Xbee::unpack_vicon_data(Vicon &vicon_data, float arr[])
{
	//      cout << "enter unpack_vicon_data" << endl;
        vicon_data.x =     arr[0];
        vicon_data.y =     arr[1];
        vicon_data.z =     arr[2];
        vicon_data.phi =   arr[3];
        vicon_data.theta = arr[4];
        vicon_data.psi =   arr[5];
//      cout << "exit unpack_vicon_data" << endl;

}
int Xbee::checksum(const uint8_t arr[], int arr_length){
     /*    int checksum_calc = 0;
         for(int i = 1; i < arr_length-2; i++)
        {
                checksum_calc += (uint8_t) arr[i];
	 	printf("checksum_calc %i, new number: %i \n", checksum_calc,(int8_t) arr[i] );
	}
		
         //combine the split 16 bit integer (two bytes: last two slots in array)

        uint16_t checksum_recv =  ( (int8_t) arr[arr_length-2] << 8) | ( (int8_t) arr[arr_length-1]);

         printf("checksum_calc: %i , checksum_recv: %i, equal?:%i\n", checksum_calc, checksum_recv, (checksum_calc == checksum_recv));
         if (checksum_calc == checksum_recv) return 1;
         else return -1;*/
	
	int checksum_calc = 0;
       for(int i = 0; i < arr_length-2; i++){
    	   checksum_calc+= (int8_t) arr[i];
    	   printf("checksum_calc %i, new number: %i \n", checksum_calc,(int8_t) arr[i] );
       }

       //combine the split 16 bit integer (two bytes: last two slots in array)
       int16_t checksum_recv =  ( (int8_t) arr[arr_length-1] << 8) | ( (int8_t) arr[arr_length-2]);

        printf("checksum_calc: %i , checksum_recv: %i, equal?:%i\n", checksum_calc, checksum_recv, (checksum_calc == checksum_recv));
       if (checksum_calc == checksum_recv) return 1;
       else return -1;
}
int Xbee::checksum_(const uint8_t arr[], int arr_length){
         int checksum_calc = 0;
         for(int i = 0; i < arr_length-2; i++)
	{
		checksum_calc+= (uint8_t) arr[i];
		 printf("checksum_calc %i, new number: %i \n", checksum_calc,(uint8_t) arr[i] );
	}
         //combine the split 16 bit integer (two bytes: last two slots in array)
         uint16_t checksum_recv =  ( (uint8_t) arr[arr_length-1] << 8) | ( (uint8_t) arr[arr_length-2]);

	printf("checksum: recieved: %i, calculated: %i \n", checksum_recv, checksum_calc);
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
                for(int i = 0; i < arr_length; i++) printf("%i : %i ,", i, arr[i]);//(arr[i] == 0xbd)); 
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

int main(int argc, char**argv)
{
		Xbee xbee("/dev/ttyUSB1",9);
		Angles joystick_des_angles = {0};
		uint8_t joystick_thrust = 0, flight_mode = 0;
		ros::init(argc,argv,"xbee");
		ros::NodeHandle nh;
		ros::Publisher xbee_pub;
		xbee_pub = nh.advertise<xbee::XbeeData>("xbee/xbee_cmds",1); 
		int suc = 0;
		while(ros::ok())
		{
			suc = xbee.get_xbee_data(joystick_des_angles, joystick_thrust, flight_mode);
			//printf("suc: %i \n",suc);
			if(suc > 0){
			
				//xbee.printData(joystick_des_angles, joystick_thrust, flight_mode); 
				xbee::XbeeData xb_msg;
				//printf("im in here");
				xb_msg.joy_des_angles[0] = joystick_des_angles.phi;
				xb_msg.joy_des_angles[1] = joystick_des_angles.theta;
				xb_msg.joy_des_angles[2] = joystick_des_angles.psi;
				xb_msg.joy_thrust = joystick_thrust;
				xb_msg.flight_mode = flight_mode;
				xb_msg.succ = 1;
				xb_msg.dt = xbee.getDt();
				xbee_pub.publish(xb_msg);
			}
		}



}

