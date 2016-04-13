#include "vicon.h"
//compilation requres -pthread option
//g++ vicon.cpp utility.cpp -I /home/odroid/upenn_quad/include -lpthread
//#include "Xbee.h"
//#include "receiver.h"

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


int usb_xbee_fd;
int select_usb_xbee_fd;
//int num_bytes_per_read=1;
int onesecond_v = 1000000/4;

int stats_flag_v = 0;
fd_set read_fds_v;
struct timeval no_timeout_v;

//stats on reading of joystick data
    //percentage of successful reads/select call
double total_it_v, succ_v, fail_v=0;
    //frequency of successful read       
timespec success_start_v, success_current_v;
double success_seconds_v, success_sum_v =0;
    //total loop frequency
timespec total_start_v, total_current_v;
double total_seconds_v, total_sum_v=0;

int NUM_JOY_BYTES=9;
using namespace std;

void get_vicon_data(int port, Vicon& vicon_data)
{
	//cout << "entering get_vicon_vicon_data" << endl;
	tcflush(usb_xbee_fd,TCIFLUSH); 
	float data_received[6] = {0.0};
	recieve_data(port, data_received,6);
    unpack_vicon_data(vicon_data, data_received);
// cout << "exit get_vicon_data" << endl;
}
void get_joystick_data(int port, Angles& joystick_des_angles, float& joystick_thrust, float& flight_mode)
{
tcflush(usb_xbee_fd,TCIFLUSH);
float data_received[5] = {0.0};
recieve_data(port, data_received,5);
unpack_joystick_data(joystick_des_angles, joystick_thrust, flight_mode, data_received);
}

int select_get_joystick_data(int port, Angles& joystick_des_angles, uint8_t& joystick_thrust, uint8_t& flight_mode)
{
//select: clear read_ds(set of file_descriptors to watch for reading), add port to this set, set time to 0 to returnimmediately,
// select returns the number of fd's ready
FD_ZERO(&read_fds_v);
FD_SET(port, &read_fds_v);
no_timeout_v.tv_sec  = 0;
no_timeout_v.tv_usec = 0;

int num_fds = select(port+1, &read_fds_v, NULL, NULL, &no_timeout_v);
int a;

if(stats_flag_v) {
//reset total start time and increment total counter
clock_gettime(CLOCK_REALTIME,&total_start_v);
total_it_v++;
}    

//1) No file descriptors ready to read
if(num_fds == 0) {
//printf("no data ready: %i \n",num_fds); return -1;}
if(stats_flag_v) {
   //get current time then  -> total_seconds_v = total_current_v - total_start_v
   //calc ave of all freq's -> total_sum_v     += new_freq
   clock_gettime(CLOCK_REALTIME,&total_current_v);
   total_seconds_v = UTILITY::timespec2float(UTILITY::time_diff(total_start_v, total_current_v));
   total_sum_v += 1/total_seconds_v;
   fail_v++;    
  }    
a = -2;  
}       
     
//2) Select returned an error
else if(num_fds ==-1)  {
  if(stats_flag_v){
        //get current time, subtract last start time, reset start time in (beginning)  ==> total
        clock_gettime(CLOCK_REALTIME,&total_current_v);
        total_seconds_v = UTILITY::timespec2float(UTILITY::time_diff(total_start_v, total_current_v));
        //calc stats
        fail_v++;                                                               
        total_sum_v += 1/total_seconds_v;
   }
a = -3;        
}              
//3) A file descriptor is ready to read: check which one
else if(FD_ISSET(port , &read_fds_v)) 
{
 //flush input buffer (TCI for input)
 //tcflush(port, TCIFLUSH);
 
 //adjust pointer to end - NUM_JOY_BYTES
 lseek(port, -NUM_JOY_BYTES, SEEK_END);
 
 //////
 
 uint8_t  data_received[NUM_JOY_BYTES];


 /////



 //read in NUM_JOY_BYTES bytes of data from port to the address in memory &data_received
 //result1 indicates success of reading
 int result = read(port, &data_received[0], NUM_JOY_BYTES);

 //ensure correct communication: check first byte and checksum
 //perform checksum
 int16_t checksum_calc = checksum(data_received, NUM_JOY_BYTES);
 if (stats_flag_v)  s_print_raw_bytes(data_received, NUM_JOY_BYTES);
 
 bool check1_correct =  (  data_received[0] == 253);
 bool check2_correct =  (  data_received[6] == 173);
 bool checksum_correct = (checksum_calc > 0);

          if( (checksum_calc > 0) && (  data_received[0] == 253) && (  data_received[6] == 173) ) 
        {          
                 select_unpack_joystick_data(  joystick_des_angles,  joystick_thrust, flight_mode, data_received);
                 a = 1;
        }
         else
         {
                 if (result == -1) printf("get_joystick_data: FAILED read from port \n");
                 //printf("\n\n\x1b[31mFIRST BYTE OR CHECKSUM WRONG:FLUSHED PORT\x1b[0m\n\n");
                
                 tcflush(port, TCIFLUSH);
                
                 if(checksum_calc < 0)
                 {      
                         a = -4;
                         /*cout << "CHECKSUM_INCORRECT" << endl;
                         printf("CHECKSUM_INCORRECT");
                         s_print_raw_bytes(data_received, NUM_JOY_BYTES);
                         Angles a; int8_t t, f;
                         select_unpack_joystick_data(a, t, f, data_received);
                         print_data_joy(a, t,f);
                         refresh();
                         usleep(100000000);
                */
                 } 
                else if(!(  data_received[0] == 253)) a=-5;
                else if(!(  data_received[6] == 173)) a=-6; 
                else a = -1;       
         }
                if(stats_flag_v) {
                        //get current time, subtract last start time, reset start time in (beginning)  ==> total
                         clock_gettime(CLOCK_REALTIME,&total_current_v);
                         total_seconds_v = UTILITY::timespec2float(UTILITY::time_diff(total_start_v, total_current_v));
                         
                         //get current time, subtract from last start time, then reset start time => success
                         clock_gettime(CLOCK_REALTIME,&success_current_v);
                         success_seconds_v = UTILITY::timespec2float(UTILITY::time_diff(success_start_v, success_current_v));
                         clock_gettime(CLOCK_REALTIME,&success_start_v);
                         
                         //calc stats
                         succ_v++;
                         success_sum_v += 1/success_seconds_v;
                         total_sum_v += 1/total_seconds_v;
                  }
    }
    //if(stats_flag_v) print_stats_joy(a);
    joystick_des_angles.succ_read = a; 
    return a;
}
int checksum(const uint8_t arr[], int arr_length){
         int checksum_calc = 0;
         for(int i = 0; i < arr_length-2; i++) checksum_calc+= (int8_t) arr[i];

         //combine the split 16 bit integer (two bytes: last two slots in array)
         int16_t checksum_recv =  ( (int8_t) arr[arr_length-1] << 8) | ( (int8_t) arr[arr_length-2]);
         
         // printf("checksum_calc: %i , checksum_recv: %i, equal?:%i\n", checksum_calc, checksum_recv, (checksum_calc == checksum_recv));
         if (checksum_calc == checksum_recv) return 1;
         else return -1;
}
int checksum(const int8_t arr[], int arr_length){
         int checksum_calc = 0;
         for(int i = 0; i < arr_length-2; i++) checksum_calc+= arr[i];

         //combine the split 16 bit integer (two bytes: last two slots in array)
         int16_t checksum_recv =  (arr[arr_length-1] << 8) | (arr[arr_length-2]);
         
        // printf("checksum_calc: %i , checksum_recv: %i, equal?:%i\n", checksum_calc, checksum_recv, (checksum_calc == checksum_recv));
         if (checksum_calc == checksum_recv) return 1;
         else return -1;
}

void s_print_raw_bytes(const uint8_t arr[], int arr_length){
                printf("Bytes: \n");
                for(int i = 0; i < arr_length; i++) printf("%i : %04x ,", i, arr[i]);//(arr[i] == 0xbd));
                printf("\n  1st byte: %i, should be: 253, equal?:%i  6th byte: %i should be: 173, equal?:%i \n",  arr[0], (arr[0] == 253), arr[6], (arr[6] == 173) );
                printf("\n");
}
void s_print_raw_bytes(const int8_t arr[], int arr_length){
                printf("Bytes: \n");
                for(int i = 0; i < arr_length; i++) printf("%i : %04x ,", i, arr[i]);//(arr[i] == 0xbd));
                printf("\n  1st byte: %i, should be: 253, equal?:%i  6th byte: %i should be: 173, equal?:%i \n",  arr[0], (arr[0] == 253), arr[6], (arr[6] == 173) );
                printf("\n");
}
void print_data_joy(const Angles& joystick_des_angles, int joystick_thrust, int flight_mode)
{   
printf("theta: %f  phi: %f  psi: %f\n" , joystick_des_angles.theta, joystick_des_angles.phi, joystick_des_angles.psi);
printf("joystick_thrust: %i  flight_mode: %i \n", joystick_thrust, flight_mode);
}

void print_stats_joy(int a){
printf("Last Result: ");
switch(a){ 
         case -1: printf("Read Error\n"); break;
         case -2: printf("Select: Not Ready to Read\n"); break;
         case -3: printf("Select: Error\n");break;                                                                                                                                                           
         case  1: printf("Select: Ready to Read, Succ Read\n"); break;
         }
 //Total Loop Frequency (1/time to perform any full loop through)
 //printf("Total Loop Frequency: %f , Average Total Loop Frequency:  %f \n", 1/total_seconds_v, total_sum_v/total_it_v);

 //Success Frequency (1/time between each successful read)
 printf("Successful Loop Frequency: %f , Average Successful Loop Frequency: %f \n", 1/success_seconds_v, success_sum_v/total_it_v); 

 //Raw number of times the loop has completed, number of times port was available to read, rate (port available/total iterations) of successful reading
 printf("Succ: %f , Fail: %f , Succ Rate: %f \n\n", succ_v, fail_v, (succ_v/(succ_v+fail_v)));
}

void unpack_vicon_data(Vicon& vicon_data, float arr[]){
    //distributes data from the input buffer to the imu_data data structure
    //we make a char[] to recieve imu data (imu outputs byte by byte)
    //the cast to a float pointer takes the first four bytes in the array 'arr',
    //thus constructing a float
    
//	cout << "enter unpack_vicon_data" << endl;
	vicon_data.x =     arr[0];
        vicon_data.y =     arr[1];
        vicon_data.z =     arr[2];
        vicon_data.phi =   arr[3];
        vicon_data.theta = arr[4];
        vicon_data.psi =   arr[5];
//	cout << "exit unpack_vicon_data" << endl;
}
void unpack_joystick_data(Angles& joystick_des_angles, float& joystick_thrust,float& flight_mode, float arr[]){
	joystick_thrust	          = arr[0]; // thrust  0 - 150
	joystick_des_angles.theta = arr[1]; // roll -75 - 75
	joystick_des_angles.phi   = arr[2]; // pitch -75 - 75
	joystick_des_angles.psi   = arr[3]; // yaw -75 - 75
	flight_mode     	  = arr[4]; 	    

}
void select_unpack_joystick_data(Angles& joystick_des_angles, int8_t& joystick_thrust, int8_t& flight_mode, int8_t arr[]){
	float scale = 3;
        
    joystick_thrust	            = arr[1] + 75 ; // thrust (-75 - 75)+ 75
	joystick_des_angles.phi     = arr[2]/scale; // roll -127 - 127
	joystick_des_angles.theta   = arr[3]/scale; // pitch 
	joystick_des_angles.psi     = arr[4]/scale; // yaw 
	flight_mode     	        = arr[5];
     //printf("phi before: %f   ", joystick_des_angles.phi);
     //printf("phi after: %f \n", ( (float) joystick_des_angles.phi /  4.0));

    //refresh();
    //joystick_des_angles.phi   = ( (float) joystick_des_angles.phi /  4.0); /// DELETE THIS AFTER RESCALING
    //joystick_des_angles.theta = ( (float) joystick_des_angles.theta /  4.0);
}
void select_unpack_joystick_data(Angles& joystick_des_angles, uint8_t& joystick_thrust, uint8_t& flight_mode, uint8_t arr[]){
	joystick_thrust	            = (int8_t) arr[1] + 75 ; // thrust
	joystick_des_angles.phi     = (int8_t) arr[2]; // roll
	joystick_des_angles.theta   = (int8_t) arr[3]; // pitch 
	joystick_des_angles.psi     = (int8_t) arr[4]; // yaw 
	flight_mode     	        = (int8_t) arr[5]; 	    
    
    //printf("phi before: %f   ", joystick_des_angles.phi);
    //printf("phi after: %f \n", ( (float) joystick_des_angles.phi /  4.0));

    //refresh();
    joystick_des_angles.phi   = ( (float) joystick_des_angles.phi /  4.0); /// DELETE THIS AFTER RESCALING
    joystick_des_angles.theta = ( (float) joystick_des_angles.theta /  4.0);
}
Vicon filter_vicon_data(Vicon& new_vicon, Vicon& old_vicon, Vicon& old_old_vicon ,Weights& weights){
    //cout << "filtering vicon data" << endl;

    Vicon filt_vicon = {0.0};

    //cout << "inputs: " << endl;
    //cout << "new_vicon.x: " << new_vicon.x << endl;
    //cout << "old_vicon.x: " << old_vicon.x << endl;
    //cout << "old_old_vicon.x: " << old_old_vicon.x << endl;

    //filt_vicon.x = (weights.newest * new_vicon.x) + (weights.old * old_vicon.x) + (weights.old_old * old_old_vicon.x);
    filt_vicon.x = filt(new_vicon.x, old_vicon.x, old_old_vicon.x, weights);
    //cout << "outputs: " << endl;
    //cout << "filt_vicon.x: " << filt_vicon.x << endl;
    filt_vicon.y = filt(new_vicon.y, old_vicon.y, old_old_vicon.y, weights);
    filt_vicon.z = filt(new_vicon.z, old_vicon.z, old_old_vicon.z, weights);
    filt_vicon.theta = filt(new_vicon.theta, old_vicon.theta, old_old_vicon.theta, weights);
    filt_vicon.phi = filt(new_vicon.phi, old_vicon.phi, old_old_vicon.phi, weights);
    filt_vicon.psi = filt(new_vicon.psi, old_vicon.psi, old_old_vicon.psi, weights);
    
    /*
    cout << "new_vicon.x: " << new_vicon.x << endl;
    cout << "old_vicon.x: " << old_vicon.x << endl;
    cout << "old_old_vicon.x: " << old_old_vicon.x << endl;
    cout << "filt_vicon.x: " << filt_vicon.x << endl;
    
    cout << endl;
     
    cout << "new_vicon.theta: " << new_vicon.theta << endl;
    cout << "old_vicon.theta: " << old_vicon.theta << endl;
    cout << "old_old_vicon.theta: " << old_old_vicon.theta << endl;
    cout << "filt_vicon.theta: " << filt_vicon.theta << endl;

    
    cout << endl;
    */
    return filt_vicon;
}
float filt(float new_data, float old_data, float old_old_data, Weights& weights){
    float f = (weights.newest * new_data) + (weights.old * old_data) + (weights.old_old * old_old_data);
    return f;
}
void pushback(Vicon& new_vicon, Vicon& old_vicon, Vicon& old_old_vicon){
    old_old_vicon = old_vicon;
    old_vicon = new_vicon;
}
void display_vicon_data(const Vicon& vicon_data) {
        printf("<==========================================>\n");   	
       	printf("    VICON DATA    \n");
        printf("phi: %.2f         x: %.2f\n",    vicon_data.phi  ,  vicon_data.x);
        printf("theta: %.2f       y: %.2f\n",    vicon_data.theta,  vicon_data.y);
        printf("psi: %.2f         z: %.2f\n\n\n",vicon_data.psi  ,  vicon_data.z);
}
void display_joystick_data(const Angles& joystick_data, const float& thrust, const float& flight_mode) {
        printf("<==========================================>\n");
        printf("    JOYSTICK DATA    \n");
        printf("phi: %.2f         theta: %.2f      psi: %.2f \n",  joystick_data.phi,  joystick_data.theta, joystick_data.psi);
        printf("thrust:         %.2f \n",    thrust);
        printf("flight_mode:    %.2f\n\n\n",flight_mode);
}
int open_xbee_port()
{
    //port = open(path, O_RDWR | O_NOCTTY)
    //port        - The returned file handle for the device. -1 if an error occurred
    //path        - The path to the serial port (e.g. /dev/ttyS0)
    //"O_RDWR"    - Opens the port for reading and writing
    //"O_NOCTTY"  - The port never becomes the controlling terminal of the process.

  struct termios newtio;
  int port; /* File descriptor for the port */
  string MY_PATH = "/dev/ttyUSB0";

  port = open(MY_PATH.c_str(), O_RDWR | O_NOCTTY);
  printf("Opening an USB port...   ");//Opens the usb Port

    //open_usbport  -- this is in reciever.h
    //port = open_usbport();  //this opens port and only waits until it gets 2 bytes!!!!
    if(port > -1){ cout << "opened port successfully: " << MY_PATH << endl; }
    else         { cout << "unable to open port: "      << MY_PATH << endl; }

    //gets the parameters (options) associated with the terminal from the termios structure (newtio)
    tcgetattr(port, &newtio);

    //set input/output baudrate
    cfsetospeed(&newtio, BAUDRATE);
    cfsetispeed(&newtio, BAUDRATE);

    //set character size mask.
    //set the number of data bits.
   
    //CSIZE flag is a mask that specifies the number of bits per byte for both transmission 
    //and reception. This size does not include the parity bit, if any. The values for the 
    //field defined by this mask are CS5, CS6, CS7, and CS8, for 5, 6, 7,and 8 bits per byte, respectively
    newtio.c_cflag &= ~CSIZE; 
    newtio.c_cflag |= CS8;     //8 bits/byte

    //no parity bits
    newtio.c_cflag &= ~CSTOPB;
    newtio.c_cflag &=~PARENB;

    //set for non-canonical (raw processing, no echo, etc.)
    newtio.c_iflag = IGNPAR; // ignore parity errors
    newtio.c_oflag = 0; // raw output
    newtio.c_lflag = 0; // raw input 

    //Time-Outs -- won't work with NDELAY option in the call to open
    //Will read until recieved a minimum of num_bytes_per_read bytes, no time limit
    newtio.c_cc[VMIN]  = num_bytes_per_read;   // block reading until RX x characers. If x = 0, it is non-blocking.
    newtio.c_cc[VTIME] = 0;   // Inter-Character Timer -- i.e. timeout= x*.1 s

     //Set local mode and enable the receiver
     newtio.c_cflag |= (CLOCAL | CREAD);

    //Set the new options for the port...
    //TCSANOW - options go into affect immediately
    int status = tcsetattr(port, TCSANOW, &newtio);

    if (status != 0){ //For error message
        printf("Configuring XBee comport failed\n");
        return status;
    }

    return port;
}
int select_open_xbee_port(void)
{
    //port = open(path, O_RDWR | O_NOCTTY)
    //port        - The returned file handle for the device. -1 if an error occurred
    //path        - The path to the serial port (e.g. /dev/ttyS0)
    //"O_RDWR"    - Opens the port for reading and writing
    //"O_NOCTTY"  - The port never becomes the controlling terminal of the process.

  struct termios newtio;
  int port; /* File descriptor for the port */
  string MY_PATH = "/dev/ttyUSB0";

  port = open(MY_PATH.c_str(), O_RDWR | O_NOCTTY);
  printf("Opening an USB port...   ");//Opens the usb Port

    //open_usbport  -- this is in reciever.h
    //port = open_usbport();  //this opens port and only waits until it gets 2 bytes!!!!
    if(port > -1){ cout << "opened port successfully: " << MY_PATH << endl; }
    else         { cout << "unable to open port: "      << MY_PATH << endl; }

    //gets the parameters (options) associated with the terminal from the termios structure (newtio)
    tcgetattr(port, &newtio);

    //set input/output baudrate
    cfsetospeed(&newtio, BAUDRATE);
    cfsetispeed(&newtio, BAUDRATE);

    //set character size mask.
    //set the number of data bits.
   
    //CSIZE flag is a mask that specifies the number of bits per byte for both transmission 
    //and reception. This size does not include the parity bit, if any. The values for the 
    //field defined by this mask are CS5, CS6, CS7, and CS8, for 5, 6, 7,and 8 bits per byte, respectively
    newtio.c_cflag &= ~CSIZE; 
    newtio.c_cflag |= CS8;     //8 bits/byte

    //no parity bits
    newtio.c_cflag &= ~CSTOPB;
    newtio.c_cflag &=~PARENB;

    //set for non-canonical (raw processing, no echo, etc.)
    newtio.c_iflag = IGNPAR; // ignore parity errors
    newtio.c_oflag = 0; // raw output
    newtio.c_lflag = 0; // raw input 

    //Time-Outs -- won't work with NDELAY option in the call to open
    //Will read until recieved a minimum of num_bytes_per_read bytes, no time limit
    newtio.c_cc[VMIN]  = NUM_JOY_BYTES;   // block reading until RX x characers. If x = 0, it is non-blocking.
    newtio.c_cc[VTIME] = 0;   // Inter-Character Timer -- i.e. timeout= x*.1 s

     //Set local mode and enable the receiver
     newtio.c_cflag |= (CLOCAL | CREAD);

    //Set the new options for the port...
    //TCSANOW - options go into affect immediately
    int status = tcsetattr(port, TCSANOW, &newtio);

    if (status != 0){ //For error message
        printf("Configuring XBee comport failed\n");
        return status;
    }

    return port;
}
int recieve_data(int fd_xbee, float data_received[], int data_size)
{

    int bytes_received, attempts, i;
    int MAXATTEMPTS=100;
    int8_t recv_byte;
    int8_t recv_buffer[sizeof(float)*data_size+3]; // 1 start byte + data + 2 checksum
    fd_set readfs;
    int16_t checksum_recv, checksum_calc=0;

    FD_ZERO(&readfs);
    FD_SET(fd_xbee, &readfs);


    // wait for data
    select(fd_xbee+1, &readfs, NULL, NULL, NULL);

    // check the start byte
    read(fd_xbee,&recv_byte,1);

    if(recv_byte==XBEE_START_BYTE)
    {
        // correct start byte

        bytes_received=0;
        attempts=0;

        recv_buffer[bytes_received]=recv_byte;
        bytes_received+=1;

        // receive data
        while(bytes_received < (sizeof(float)*data_size)+3 && attempts++ < MAXATTEMPTS)
        {
            if(read(fd_xbee,&recv_byte,1)==1)
            {
                recv_buffer[bytes_received]=recv_byte;
                bytes_received+=1;
            }
            else
            {
                usleep(1000);
            }

        }

        // Checksum: sum of data without start byte (two bytes: rolls over at 65536)
        for(i=1;i<sizeof(float)*data_size+1;i++)
        {
            checksum_calc+=recv_buffer[i];
        }
        checksum_recv=recv_buffer[sizeof(float)*data_size+1] << 8 | recv_buffer[sizeof(float)*data_size+2]; // high << 8 | low

        if(checksum_recv == checksum_calc)
        {
         // correct checksum
	// convert data to float
        for(i=0;i<data_size;i++) { data_received[i]=*(float *)&recv_buffer[sizeof(float)*i+1]; }
            return bytes_received;
        }
        else
        {
            // incorrect checksum
            printf("checksum_calc:%d, checksum_recv:%d\n",checksum_calc,checksum_recv);
            return -2;
       	}
    }
   else
    {  	// incorrect start byte
        return -1;
    }

 tcflush(fd_xbee,TCIOFLUSH);

}

void xbee_init(void){
    
    //usb_xbee_fd = open_xbee_port();
    select_usb_xbee_fd = select_open_xbee_port();
}

/*
int main(void){

   xbee_init();

   Vicon vicon_data;
   Angles joystick_des_angles;
   //float joystick_thrust, flight_mode = 0;
   uint8_t joystick_thrust, flight_mode = 0; 
   
   int8_t a,b;
   int8_t c,d;

   while(1){
            //get_vicon_data(usb_xbee_fd, vicon_data);
            //display_vicon_data(vicon_data);
	    //get_joystick_data(usb_xbee_fd, joystick_des_angles, joystick_thrust, flight_mode);
	    //display_joystick_data(joystick_des_angles, joystick_thrust, flight_mode);
                int suc_read = select_get_joystick_data(select_usb_xbee_fd, joystick_des_angles, joystick_thrust, flight_mode);
                if(suc_read<0) ;//printf("No new data: Succ Read: %i \n \n \n \n", suc_read);
                else{ print_data_joy(joystick_des_angles, joystick_thrust, flight_mode);printf("\n\n\n");}
        // usleep(10000);
          // int8_t data_received[NUM_JOY_BYTES];
          // int result = read(select_usb_xbee_fd, &data_received[0], NUM_JOY_BYTES);
          //  printf("result of read: %i\n", result);   
        //a++;
        //printf("a %i casted to unsigned: %i cast back to signed %i \n",a, (uint8_t) a, (int8_t) (uint8_t) a);

        }

    return 0;

}
*/
