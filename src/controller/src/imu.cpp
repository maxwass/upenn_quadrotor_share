#include "imu.h"

//g++ imu.cpp logger.cpp utility.cpp -I ../include -std=c++11

int Imu::get_imu_calibrated_data_ns(State& imu_data)
{
//identical to get_imu__data, but return calibrated values instead
        //get_raw_data
	imu_data.succ_read = -1;
        imu_data.succ_read = get_imu_data_ns(imu_data);

        if(!(this->calibrated)) //printf( "WARNING: RETURNING CALIBRATED IMU VALUES BEFORE PERFORMING CALIBRATION");
        
	//if successfuly recieved data, subtract off bias
        if((imu_data.succ_read==1) && (this->calibrated))
        {
		imu_data.phi_dot    = imu_data.phi_dot_cal;
                imu_data.theta_dot  = imu_data.theta_dot_cal;
                imu_data.psi_dot    = imu_data.psi_dot_cal;
		imu_data.psi	    = imu_data.psi_contin_cal;
      	}

	return imu_data.succ_read;

}

int Imu::get_imu_data_ns(State& imu_data)
{
	int a = -10;

    	unsigned char sensor_bytes2[(this->data_size)]; //= {0.0};

	lseek(port, -(this->data_size), SEEK_END);

	//read in data_size bytes of data from port to the address in memory &sensor_bytes2
	//result1 indicates success of reading
	int result = read(port, &sensor_bytes2[0], data_size);

	//track dt between reads
	timer.update();

	//check first and last byte
	if((sensor_bytes2[0] == 0xbd) && (sensor_bytes2[data_size-1] == 0xff) )
	{
	    unpack_data(imu_data, sensor_bytes2);
	    a=1;
	}
	else
	{
	    if (result == -1) printf("get_imu_data: FAILED read from port \n");
	    printf("\n\n\x1b[31mCHECK BYTES WRONG:FLUSHED PORT\x1b[0m\n\n");
	    tcflush(port, TCIFLUSH);
	    if(!(sensor_bytes2[0]== 0xbd))       {printf("FIRST BYTE WRONG%04x \x1b[0m\n\n", sensor_bytes2[0]); a=-5;}
	    else if(!(sensor_bytes2[data_size-1] == 0xff)) {printf("LAST BYTE WRONG%04x \x1b[0m\n\n", sensor_bytes2[data_size-1]); a=-6;}
	    else a = -1;
	}

	return a;
}

int Imu::get_imu_calibrated_data(State& imu_data){
//identical to get_imu__data, but return calibrated values instead
        //get_raw_data
	imu_data.succ_read = -1;
        imu_data.succ_read = get_imu_data(imu_data);

        if(!(this->calibrated)) //printf( "WARNING: RETURNING CALIBRATED IMU VALUES BEFORE PERFORMING CALIBRATION");
        
	//if successfuly recieved data, subtract off bias
        if((imu_data.succ_read==1) && (this->calibrated))
        {
		imu_data.phi_dot    = imu_data.phi_dot_cal;
                imu_data.theta_dot  = imu_data.theta_dot_cal;
                imu_data.psi_dot    = imu_data.psi_dot_cal;
		imu_data.psi	    = imu_data.psi_contin_cal;
      	}

	return imu_data.succ_read;

}
int Imu::get_imu_data(State& imu_data)
{
//select: clear read_ds(set of file_descriptors to watch for reading), add port to this set, set time to 0 to returnimmediately,
    // select returns the number of fd's ready
    FD_ZERO(&read_fds);
    FD_SET(port, &read_fds) ;
    no_timeout.tv_sec  = 0;
    no_timeout.tv_usec = 0;
    int num_fds = select(port+1, &read_fds, NULL, NULL, &no_timeout);
	
    //printf("num_fds: %i \n", num_fds);

    int a = -10;
    //No data ready to read
    if(num_fds == 0)  a = -2;

    //select returned an error
    else if(num_fds ==-1)  a = -3;

    //A file descriptor is ready to read: check which one
    else if(FD_ISSET(port , &read_fds))
         {
		unsigned char sensor_bytes2[(this->data_size)]; //= {0.0};

		lseek(port, -(this->data_size), SEEK_END);

		//read in data_size bytes of data from port to the address in memory &sensor_bytes2
		//result1 indicates success of reading
		int result = read(port, &sensor_bytes2[0], data_size);

		//track dt between reads
		timer.update();

                //check first and last byte
                if((sensor_bytes2[0] == 0xbd) && (sensor_bytes2[data_size-1] == 0xff) )
		{
                    unpack_data(imu_data, sensor_bytes2);
		    a=1;
                }
		else
		{
                    if (result == -1) printf("get_imu_data: FAILED read from port \n");
                    printf("\n\n\x1b[31mCHECK BYTES WRONG:FLUSHED PORT\x1b[0m\n\n");
                    tcflush(port, TCIFLUSH);
                    if(!(sensor_bytes2[0]== 0xbd))       {printf("FIRST BYTE WRONG%04x \x1b[0m\n\n", sensor_bytes2[0]); a=-5;}
                    else if(!(sensor_bytes2[data_size-1] == 0xff)) {printf("LAST BYTE WRONG%04x \x1b[0m\n\n", sensor_bytes2[data_size-1]); a=-6;}
                    else a = -1;
                }
         }

	return a;
}
void Imu::unpack_data(State& imu_data, const unsigned char arr[])
{
//distributes data from the input buffer to the imu_data data structure
//we make a char[] to recieve imu data (imu outputs byte by byte)
//the cast to a float pointer takes the first four bytes in the array 'arr',
//thus constructing a float

     //RAW VALUES
        float att_vel[3] = {0.0};
        att_vel[0]         = *(int32_t *)&arr[13]; //printf("att_vel 1: %i \n", att_vel[0]);
        att_vel[1]         = *(int32_t *)&arr[17]; //printf("att_vel 2: %i \n", att_vel[1]);
        att_vel[2]         = *(int32_t *)&arr[21]; //printf("att_vel 3: %i \n", att_vel[2]);

        imu_data.phi_dot   = -att_vel[1]/100*1.5;
        imu_data.theta_dot = -att_vel[0]/100*1.5;
        imu_data.psi_dot   = -att_vel[2]/100*1.5;

	
        imu_data.psi_magn_raw      = *(float *)&arr[1]; //printf("psi 1: %f \n", imu_data.psi);
        imu_data.theta     	   = *(float *)&arr[5]; //printf("theta 1: %f \n", imu_data.theta)state2rawBytes(imu_data);
        imu_data.phi       	   = *(float *)&arr[9]; //printf("phi 1: %f \n", imu_data.phi);
	
	//make psi continuos here
		
	imu_data.numPsiRot = p.getIter();
	imu_data.dt = timer.getDt();//(this->calc_dt);

       	imu_data.psi_magn_continuous  =  p.make_contin(imu_data.psi_magn_raw);


	int32_t acc[3] = {0};

	acc[0] = *(int32_t *)&arr[25];
        acc[1] = *(int32_t *)&arr[29];
        acc[2] = *(int32_t *)&arr[33];

	uint32_t pressure_data = *(uint32_t *)&arr[37];
	imu_data.altitude_raw = 8.31432*303.*log(pressure_data/101325.)/(-9.80665*0.0289644);
	//altitude.update(imu_data.altitude_raw, timer.getDt());
	//imu_data.altitude_deriv   =  altitude.getAltitudeDeriv();

	if((this->calibrated)) 
	{
		//printf("Psi Freq: %f, Cal Psi_dot: %f, Uncal Psi_dot: %f, bias.psi_dot %f, Psi update out: %f, Psi returned: %f \n", 1/timer.getDt(), imu_data.psi_dot_cal, imu_data.psi_dot, bias.psi_dot, gyroEstimate.updatePsi(imu_data.psi_dot_cal), gyroEstimate.getPsi() );

		imu_data.phi_dot_cal    =  imu_data.phi_dot    - bias.phi_dot;
        	imu_data.psi_dot_cal    =  imu_data.psi_dot    - bias.psi_dot;
        	imu_data.theta_dot_cal  =  imu_data.theta_dot  - bias.theta_dot;		
		imu_data.psi_magn_continuous_calibrated =  imu_data.psi_magn_continuous  - bias.psi_magn_continuous;
		
		gyroEstimate.updatePsi(imu_data.psi_dot_cal);
		//printf("Psi dt: %f, Psi freq: %f \n",  gyroEstimate.getDt(),  1/gyroEstimate.getDt());
		//pg->setDt(timer.getDt());
		imu_data.psi_gyro_integration = gyroEstimate.getPsi();


		imu_data.altitude_calibrated = imu_data.altitude_raw - bias.altitude_raw;

	}
}
/*
int main(void){

	std::string path = "/dev/ttyACM0";
	Imu imu(path, 42, .007);
	//Altitude Altitude;

	State imu_data = {0.0};

	int cal = imu.calibrate();

	//logging 
	//std::string log_filename = "file.txt";
	//logger logger(log_filename, 100, true);
	//Data_log d;

	while(1)
	{
		int suc = imu.get_imu_calibrated_data(imu_data);
		if(suc==1)
		{ 
			imu.print_data(imu_data);
		}
	}

}
*/
