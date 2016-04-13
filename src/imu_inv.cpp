#include "imu_inv.h"

// g++ imu_inv.cpp logger.cpp utility.cpp -I ../include

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


State gyro_bias = {0};

fd_set read_fds;
struct timeval no_timeout;
double total_it=0;
double succ=0;
double fail=0;

timespec success_start;
timespec success_current;
double success_seconds, success_sum =0;

timespec total_start;
timespec total_current;
double total_seconds, total_sum=0;

bool calibration_mode = 0;
bool stats_flag = 0;
bool disable_motors = 0;

//for continuos psi
int num_psi_rotations = 0;
bool first_loop = true;
State old_raw_psi;
State new_raw_psi;

//for imu value checking
bool first_loop_check = true;
State old_imu_check;


int open_imu_ivn_port()
{
    //port = open(path, O_RDWR | O_NOCTTY)
    //port        - The returned file handle for the device. -1 if an error occurred
    //path        - The path to the serial port (e.g. /dev/ttyS0)
    //"O_RDWR"    - Opens the port for reading and writing
    //"O_NOCTTY"  - The port never becomes the controlling terminal of the process.
    struct termios newtio;

    int port; /* File descriptor for the port */
    port = open(PATH2IMU, O_RDWR | O_NOCTTY);

    if(port > -1){ cout << "opened port successfully: " << PATH2IMU <<", Port number: " << port << endl; }
    else         { cout << "unable to open port: "      << PATH2IMU << endl; }
    //sets the parameters associated with the terminal
    //from the termios structure referred to by newtio.
    tcgetattr(port, &newtio);
    //set input/output baudrate
    cfsetospeed(&newtio, BAUDRATE_INV);
    cfsetispeed(&newtio, BAUDRATE_INV);

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
    //Will read until recieved a minimum of 26 bytes, no time limit
    newtio.c_cc[VMIN]  = 26; //ivn_imu_data_size;   // block reading until RX x characers. If x = 0, it is non-blocking.
    newtio.c_cc[VTIME] = 0;   // Inter-Character Timer -- i.e. timeout= x*.1 s

    //Set local mode and enable the receiver
    newtio.c_cflag |= (CLOCAL | CREAD);

    //Set the new options for the port...
    //TCSANOW - options go into affect immediately
    int status = tcsetattr(port, TCSANOW, &newtio);
    if (status != 0){ //For error message
        printf("Configuring comport failed\n");
        return status;
    }
    return port;
}

void print_data(const State& imu_data)
{
    printf("theta: %f  phi: %f  psi: %f\n  theta_dot: %f  phi_dot: %f  psi_dot: %f\n\n", imu_data.theta, imu_data.phi, imu_data.psi, imu_data.theta_dot, imu_data.phi_dot, imu_data.psi_dot);

}

int unpack_ivn_data(State& imu_data,  const unsigned char arr[]){
    //distributes data from the input buffer to the imu_data data structure
    //we make a char[] to recieve imu data (imu outputs byte by byte)
    //the cast to a float pointer takes the first four bytes in the array 'arr',
    //thus constructing a float

	float att_vel[3] = {0.0};
        att_vel[0]         = *(int32_t *)&arr[13]; //printf("att_vel 1: %i \n", att_vel[0]);
        att_vel[1]         = *(int32_t *)&arr[17]; //printf("att_vel 2: %i \n", att_vel[1]);
        att_vel[2]         = *(int32_t *)&arr[21]; //printf("att_vel 3: %i \n", att_vel[2]);

	    imu_data.psi       = *(float *)&arr[1]; //printf("psi 1: %f \n", imu_data.psi);
        imu_data.theta     = *(float *)&arr[5]; //printf("theta 1: %f \n", imu_data.theta)state2rawBytes(imu_data);
        imu_data.phi       = *(float *)&arr[9]; //printf("phi 1: %f \n", imu_data.phi);
        
	    imu_data.psi	   = saturate(imu_data.psi);
	    imu_data.theta	   = saturate(imu_data.theta);
	    imu_data.phi	   = saturate(imu_data.phi);
       
        new_raw_psi.psi = imu_data.psi; 
        imu_data.psi = make_psi_contin(); 
       
        if(!calibration_mode)
        {
                imu_data.phi_dot   = saturate(-att_vel[1]/100*1.5) - gyro_bias.phi_dot	;
                imu_data.theta_dot = saturate(-att_vel[0]/100*1.5) - gyro_bias.theta_dot;
                imu_data.psi_dot   = saturate(-att_vel[2]/100*1.5) - gyro_bias.psi_dot; //printf("psi_dot: %f \n", imu_data.psi_dot);
                imu_data.psi       = imu_data.psi - gyro_bias.psi;
	    }
        else
        {
                imu_data.phi_dot   = saturate(-att_vel[1]/100*1.5);
                imu_data.theta_dot = saturate(-att_vel[0]/100*1.5);
                imu_data.psi_dot   = saturate(-att_vel[2]/100*1.5);
                
                //check for crazy values
                imu_value_check(imu_data);
        }
        //cout << imu_data.phi << endl;
        return 1;
}
float make_psi_contin(void){//State& old_raw_psi, State& new_raw_psi){
        
//on first loop through store the first value
if(first_loop)
{ 
         old_raw_psi.psi = new_raw_psi.psi;
         first_loop = !first_loop;
         printf("IN FIRST LOOP, value psi: %f \n", old_raw_psi.psi);
}

//If psi is decreasing
if( (old_raw_psi.psi <= 10.0) && (old_raw_psi.psi >= 0.0)  && (new_raw_psi.psi <= 360.0) && (new_raw_psi.psi >= 350.0))
    {
            num_psi_rotations--;
    }
//Else psi is increasing
else if( (new_raw_psi.psi <= 10.0) && (new_raw_psi.psi >= 0.0) && (old_raw_psi.psi <= 360.0) && (old_raw_psi.psi >= 350.0))
    {
            num_psi_rotations++;
    }

float psi_to_return  = num_psi_rotations*360 + old_raw_psi.psi;
//printf("output psi: %f, new raw psi: %f old_raw_psi: %f , num_rotations: %i\n", psi_to_return, new_raw_psi.psi, old_raw_psi.psi, num_psi_rotations);
       
old_raw_psi.psi = new_raw_psi.psi;

return psi_to_return;

}
void imu_value_check(State& imu_data){
    bool crazy_value = false;
        if( (imu_data.theta > 10) || (imu_data.theta <  -10) )              crazy_value = true;
        else if( (imu_data.phi > 10) || (imu_data.phi < -10) )              crazy_value = true;
        else if( (imu_data.psi > 360) || (imu_data.psi < -360) )            crazy_value = true;
        else if( (imu_data.theta_dot > 100) || (imu_data.theta_dot < -100) )crazy_value = true;
        else if( (imu_data.phi_dot > 100) || (imu_data.phi_dot < -100) )    crazy_value = true;
        else if( (imu_data.psi_dot > 100) || (imu_data.psi_dot < -100) )    crazy_value = true;

    if(crazy_value) {
            printf("\n\n\x1b[31mVALUE OUT OF NORMAL RANGE DURING IMU CALIBRATION:\x1b[0m");
            print_data(imu_data);
            state2rawBytes(imu_data);
            printf("\n\x1bPaused for 5 seconds if you want to exit before start: motors will be disabled\x1b[0m");
            printf("\n\n");
            usleep(1000000*5);
            disable_motors = 1;
    }

}

int imu_check(const State& imu_data){
        int crazy_value = 1;
        int angle_cap = 360;
        int gyro_cap = 1200;
        
        if( (imu_data.theta > angle_cap) || (imu_data.theta <  -angle_cap) )              crazy_value = -8;
        else if( (imu_data.phi > angle_cap) || (imu_data.phi < -angle_cap) )              crazy_value = -9;
        else if( (imu_data.psi > 2000) || (imu_data.psi < -2000) )                        crazy_value = -10;
        else if( (imu_data.theta_dot > gyro_cap) || (imu_data.theta_dot < -gyro_cap) )    crazy_value = -11;
        else if( (imu_data.phi_dot > gyro_cap) || (imu_data.phi_dot < -gyro_cap) )        crazy_value = -12;
        else if( (imu_data.psi_dot > gyro_cap) || (imu_data.psi_dot < -gyro_cap) )        crazy_value = -13;
    
        return crazy_value;
}
/*
int imu_deriv_check(const State& imu_data){
        int crazy_value = 1;
        int angle_deriv_cap = 360;
        int gyro__deriv_cap = 1200;
        
        if(first_loop_check


}
*/
template <typename T>
T saturate(T imu_val) {
//if (imu_val > 400)  imu_val = 0;
//else if (imu_val < -400) imu_val = 0;

return imu_val;
}
void print_raw_bytes(const unsigned char arr[], int arr_length){
        printf("Bytes: \n");
        for(int i = 0; i < arr_length; i++) printf("%i : %04x ,", i, arr[i]);//(arr[i] == 0xbd));
        printf("\n\n\n");
}
void state2rawBytes(const State& imu_data){
    printf("theta: %04x   phi: %04x  psi: %04x \n  theta_dot: %04x  phi_dot: %04x  psi_dot: %04x \n\n", (unsigned char) imu_data.theta, (unsigned char) imu_data.phi, (unsigned char) imu_data.psi, (unsigned char) imu_data.theta_dot, (unsigned char) imu_data.phi_dot, (unsigned char) imu_data.psi_dot);


}

void print_stats(int last_return){
printf("Last Result: ");
switch(last_return){
        case -1: printf("Read Error\n"); break;
        case -2: printf("Select: Not Ready to Read\n"); break;
        case -3: printf("Select: Error\n");break;
        case  1: printf("Select: Ready to Read, Succ Read\n"); break;
        }
//Total Loop Frequency (1/time to perform any full loop through)
//printf("Total Loop Frequency: %f , Average Total Loop Frequency:  %f \n", 1/total_seconds, total_sum/total_it);

//Success Frequency (1/time between each successful read)
printf("Successful Loop Frequency: %f , Average Successful Loop Frequency: %f \n", 1/success_seconds, success_sum/total_it); 

//Raw number of times the loop has completed, number of times port was available to read, rate (port available/total iterations) of successful reading
//printf("Succ: %f , Fail: %f , Succ Rate: %f \n\n", succ, fail, (succ/(succ+fail)));
}


int get_imu_ivn_data(const int port, State& imu_data)
{           
    //select: clear read_ds(set of file_descriptors to watch for reading), add port to this set, set time to 0 to returnimmediately,
    // select returns the number of fd's ready
    FD_ZERO(&read_fds);
    FD_SET(port, &read_fds);
    no_timeout.tv_sec  = 0;
    no_timeout.tv_usec = 0;
    int num_fds = select(port+1, &read_fds, NULL, NULL, &no_timeout);
   
    int a;
    
    if(stats_flag) {
            //reset total start time and increment total counter
            clock_gettime(CLOCK_REALTIME,&total_start);     
            total_it++;
    }

    //No file descriptors ready to read
    if(num_fds == 0) {
            //printf("no data ready: %i \n",num_fds); return -1;}
            if(stats_flag) {
                 //get current time then  -> total_seconds = total_current - total_start
                 //calc ave of all freq's -> total_sum     += new_freq
                 clock_gettime(CLOCK_REALTIME,&total_current);
                 total_seconds = timespec2float(time_diff(total_start, total_current)); 
                 total_sum += 1/total_seconds;
                 fail++;    
            }
        a = -2;
    }
    
    //select returned an error
    else if(num_fds ==-1)  {
           if(stats_flag){
                    //get current time, subtract last start time, reset start time in (beginning)  ==> total
                    clock_gettime(CLOCK_REALTIME,&total_current);
                    total_seconds = timespec2float(time_diff(total_start, total_current));
                    //calc stats
                    fail++;                                                                                                                                
                    total_sum += 1/total_seconds;
                   }
     a = -3;
    }
    //A file descriptor is ready to read: check which one
    else if(FD_ISSET(port , &read_fds)) {
            if(stats_flag) {
                            //get current time, subtract last start time, reset start time in (beginning)  ==> total
                            clock_gettime(CLOCK_REALTIME,&total_current);
                            total_seconds = timespec2float(time_diff(total_start, total_current));
                           
                            //get current time, subtract from last start time, then reset start time => success
                            clock_gettime(CLOCK_REALTIME,&success_current);
                           success_seconds = timespec2float(time_diff(success_start, success_current));
                            clock_gettime(CLOCK_REALTIME,&success_start);
                            
                            //calc stats
                            succ++;
                            success_sum += 1/success_seconds;
                            total_sum += 1/total_seconds;
                            }
            unsigned char sensor_bytes2[ivn_imu_data_size]; //= {0.0};

            //flush input buffer (TCI for input)
            //tcflush(port, TCIFLUSH);

            lseek(port, -26, SEEK_END);
            
            //read in 26 bytes of data from port to the address in memory &sensor_bytes2
            //result1 indicates success of reading
            int result = read(port, &sensor_bytes2[0], ivn_imu_data_size);
            
            //print_raw_bytes(sensor_bytes2, ivn_imu_data_size);
               
                //check first and last byte
                if((sensor_bytes2[0] == 0xbd) && (sensor_bytes2[25] == 0xff) ){
                    unpack_ivn_data(imu_data, sensor_bytes2); 
                    a = imu_check(imu_data);
                }else{
                    if (result == -1) printf("get_imu_data: FAILED read from port \n");
                    printf("\n\n\x1b[31mCHECK BYTES WRONG:FLUSHED PORT\x1b[0m\n\n");
                    tcflush(port, TCIFLUSH);
                    if(!(sensor_bytes2[0]== 0xbd))       {printf("FIRST BYTE WRONG%04x \x1b[0m\n\n", sensor_bytes2[0]); a=-5;} 
                    else if(!(sensor_bytes2[25] == 0xff)) {printf("LAST BYTE WRONG%04x \x1b[0m\n\n", sensor_bytes2[25]); a=-6;}
                    else a = -1;
                }
         }        
    if(stats_flag) print_stats(a);
    //print_data(imu_data);
    imu_data.succ_read = a;
    return a;

}
int find_gyro_bias(const int port){
    calibration_mode = 1;
	int i = 0;

	timespec start;
	timespec current;
	clock_gettime(CLOCK_REALTIME,&start);
	clock_gettime(CLOCK_REALTIME,&current);
    
	double seconds = timespec2float(time_diff(start, current));
    State imu_data = {0};
    
    while(seconds < 5) {
        int suc_read = get_imu_ivn_data(port, imu_data);
        if(stats_flag) printf("suc_read: %i !suc_read: %i \n",suc_read, !suc_read);
        //suc_read: -1: first byte wrong or failed read()
                  //-2: no file descriptors ready to read
                  //-3: select() returned an error()
                  // 1: file descriptor ready, successful read        
                if(suc_read<0) ;//printf("printing old values\n \n \n \n \n \n");
                else{
                        gyro_bias.theta_dot += imu_data.theta_dot;
                        gyro_bias.phi_dot   += imu_data.phi_dot;
                        gyro_bias.psi_dot   += imu_data.psi_dot;
                        gyro_bias.psi       += imu_data.psi;
                        i++;
                        if(stats_flag){ 
                        printf("Seconds: %f\n", seconds);
                        printf("gyro_reading	     :	theta_dot: %5.3f  phi_dot: %5.3f  psi_dot: %5.3f		  \n", imu_data.theta_dot, imu_data.phi_dot, imu_data.psi_dot);
                        printf("gyro_bias (accum)    :  theta_dot: %5.3f  phi_dot: %5.3f  psi_dot: %5.3f   		  \n", gyro_bias.theta_dot, gyro_bias.phi_dot, gyro_bias.psi_dot);
                        printf("gyro_bias (averaged) :  theta_dot: %5.3f  phi_dot: %5.3f  psi_dot: %5.3f  iterations: %i  \n\n", gyro_bias.theta_dot/i, gyro_bias.phi_dot/i, gyro_bias.psi_dot/i, i);
                        printf("yaw (psi)_ave (averaged)_  :  psi:       %5.3f \n", gyro_bias.psi/i); 
                        }
                       //print_data(imu_data);
                
                }
    clock_gettime(CLOCK_REALTIME,&current);
    seconds = timespec2float(time_diff(start, current));
    }

    calibration_mode = 0;

    printf("gyro_bias (averaged) :  theta_dot: %5.3f  phi_dot: %5.3f  psi_dot: %5.3f  iterations: %i  \n\n", gyro_bias.theta_dot/i, gyro_bias.phi_dot/i, gyro_bias.psi_dot/i, i);
    gyro_bias.theta_dot = gyro_bias.theta_dot/i;
    gyro_bias.phi_dot   = gyro_bias.phi_dot/i;
    gyro_bias.psi_dot   = gyro_bias.psi_dot/i;
    gyro_bias.psi       = gyro_bias.psi/i;
    return disable_motors;
}

int main (void)
{
    //logging data
    std::string log_filename = "file.txt";
    logger logger(log_filename, 100, false);
    Data_log d;

    // File descriptor for the port 
    int port = open_imu_ivn_port();
 
    find_gyro_bias(port); 
    //usleep(1000000/2);
   
    //instantiate imu_data ==> scope is over while loop. Same imu_data will be overwritten repeatedly
    State imu_data = {0.0};

    while(1)
    { 
	//pull data from sensor and put into imu_data
	int suc_read = get_imu_ivn_data(port, imu_data);
	if(suc_read<0) ;//printf("No new data: printing old values\n \n \n \n");
    if( suc_read < -7 ) {printf("BAD VALUE!!!!"); usleep(10000000);}
    else
    { 
    print_data(imu_data);
    //imu_value_check(imu_data);
    }

//	d.imu = imu_data;
//	logger.log(d);
	
    }

return 0;


}

