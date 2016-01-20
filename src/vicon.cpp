#include "vicon.h"
//compilation requres -pthread option
//g++ vicon.cpp -I /home/odroid/upenn_quad/include -lpthread

/*
Copyright (c) <2015>, <University of Pennsylvania>
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
   * Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
   * Neither the name of the University of Pennsylvania nor the names of its contributors may be used to endorse or promote  derived from this software without specific prior written permission.
  
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL University of Pennsylvania BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

int usb_xbee_fd;
//int num_bytes_per_read=1;
int onesecond_v = 1000000/4;

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

int recieve_data(int fd_xbee, float data_received[], int data_size)
{

    int bytes_received, attempts, i;
    int MAXATTEMPTS=100;
    uint8_t recv_byte;
    uint8_t recv_buffer[sizeof(float)*data_size+3]; // 1 start byte + data + 2 checksum
    fd_set readfs;
    uint16_t checksum_recv, checksum_calc=0;

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
    
    usb_xbee_fd = open_xbee_port();
    //usb_xbee_fd = open_usbport_twobytes(); //Daewons functions from Xbee.h
}
/*
int main(void){

   xbee_init();

   Vicon vicon_data;
   Angles joystick_des_angles;
   float joystick_thrust, flight_mode = 0;
    while(1){
  	    cout << "before get_xbee_data" << endl;
            //get_vicon_data(usb_xbee_fd, vicon_data);
            //display_vicon_data(vicon_data);
	    get_joystick_data(usb_xbee_fd, joystick_des_angles, joystick_thrust, flight_mode);
	    display_joystick_data(joystick_des_angles, joystick_thrust, flight_mode);
           // usleep(10000);           
    }

    return 0;

}
*/
