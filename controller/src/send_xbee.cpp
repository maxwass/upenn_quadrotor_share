// g++ send_xbee.cpp

#include <string.h>  
#include <stdint.h>  
#include <stdio.h>
#include <termios.h>
#include <iostream>  
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */

using namespace std;
#define BAUDRATE B115200
int NUM_JOY_BYTES = 9;

int open_xbee_port()                                                                                                                                                                                 
 {
      struct termios newtio;
      int port;
        
      //port = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
      port = open("/dev/cu.usbserial-DA01LOG7", O_RDWR | O_NOCTTY);
      
      tcgetattr(port, &newtio);
      cfsetospeed(&newtio, BAUDRATE);
      cfsetispeed(&newtio, BAUDRATE);
  
         //set the number of data bits.
         newtio.c_cflag &= ~CSIZE;  // Mask the character size bits
         newtio.c_cflag |= CS8;
  
         //set the number of stop bits to 1
         newtio.c_cflag &= ~CSTOPB;
  
         //Set parity to None
         newtio.c_cflag &=~PARENB;
  
         //set for non-canonical (raw processing, no echo, etc.)
         newtio.c_iflag = IGNPAR; // ignore parity check close_port(int
         newtio.c_oflag = 0; // raw output
         newtio.c_lflag = 0; // raw input
  
         //Time-Outs -- won't work with NDELAY option in the call to open
         newtio.c_cc[VMIN]  = NUM_JOY_BYTES;   // block reading until RX x characers. If x = 0, it is non-blocking.
         newtio.c_cc[VTIME] = 0;   // Inter-Character Timer -- i.e. timeout= x*.1 s
      
         //Set local mode and enable the receiver
         newtio.c_cflag |= (CLOCAL | CREAD);
        
         //tcflush(port, TCIFLUSH);
      //Set the new options for the port...
      int status=tcsetattr(port, TCSANOW, &newtio);
           
      if (status != 0){ //For error message
          cout<< "Configuring comport failed\n";
          return status;
      }
      else cout << "open port success\n";
  
      return port;
}                

int main(void){

                cout << "entering main" << endl;

                int fd_xbee = open_xbee_port();

                int8_t send_buf[NUM_JOY_BYTES];
 
                // int8_t f = 244;
                // printf("f %i, f cast to unsigned int %i", f,(uint8_t)f);
                //  usleep(1000000);

                int i = 1;
                while(1)
                {       
                        i++;
                        send_buf[0] = 253; //  253  oxfd
                        send_buf[1] = 10;    // thrust
                        send_buf[2] = 0;    // roll
                        send_buf[3] = 0;    // pitch
                        send_buf[4] = -5;    // yaw
                        send_buf[5] = 20;  //estop
                        send_buf[6] = 173;// 0xad
                        int16_t sum = send_buf[0] + send_buf[1] + send_buf[2] + send_buf[3] + send_buf[4] + send_buf[5] + send_buf[6];
                        int8_t low  = sum & 0xFF;
                        int8_t high = (sum>>8);

                        send_buf[7] = low;
                        send_buf[8] = high;
                        
                        //this is where we cast: must do this for reading to work. unsure why
                          int8_t u_send_buf[NUM_JOY_BYTES];
                          for(int j = 0; j < NUM_JOY_BYTES; j++){
                                u_send_buf[j] = (uint8_t) send_buf[j];
                          }

                        tcflush(fd_xbee,TCOFLUSH);
                        int bytes_sent = write(fd_xbee,u_send_buf,NUM_JOY_BYTES);
                        if (bytes_sent < 0) cout << "error writing!!!!\n\n";
                        else cout << "sent: " << (int)bytes_sent << " bytes\n\n";
                        usleep(1000);

                }
        }