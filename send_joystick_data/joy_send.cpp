#include <string.h>  
#include <stdint.h>  
#include <stdio.h>
#include <termios.h>
#include <iostream>  
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <linux/types.h>
#include <linux/input.h>
#include <linux/hidraw.h>
#include <errno.h>
#include <sys/time.h>
 
 
using namespace std;
#define BAUDRATE B115200


 #ifndef HIDIOCSFEATURE
 #warning Please have your distro update the userspace kernel headers
 #define HIDIOCSFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x06, len)
 #define HIDIOCGFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x07, len)
 #endif
 struct timeval t_init, t_now;
 double t, del_t, del_t_;
 double t_prev = 0.0;
double t_prev_ = 0.0;
 const char *bus_str(int bus);
 
 int saturation_value(float input, float ceiling, float floor)
 {
 	if (input > ceiling) input = ceiling;
 	if (input < floor) input = floor;
 	
 	return input;

 }

int NUM_JOY_BYTES = 9;

int open_xbee_port()                                                                                                                                                                                 
 {
      struct termios newtio;
      int port;
        
      port = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
      //port = open("/dev/cu.usbserial-DA01LOG7", O_RDWR | O_NOCTTY);
      
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

int main(int argc, char **argv){
 	 int8_t send_buf_[5];
 	 double t_temp = 13000;
 	 gettimeofday(&t_init,NULL);
 	 
 	 int thrust_h = 53;
 	 int thrust_l = -52;
 	 
 	 int roll_h = 48; //left end
 	 int roll_l = -41; //right end
 	 
 	 int pitch_h = 52;
 	 int pitch_l = -55;
 	 
 	 int yaw_h = 46; //left end
 	 int yaw_l = -42; //right end
 	 
 	 
 	 printf("Opening an USB port...   ");//Opens the usb Port
	//int usb0 = open_port();
	//if (usb0 <0)
       // 	printf("\n Error opening an USB0 port!!\n");
        //else
        //	printf("Done!\n");
        //usleep(14000);
 	
         int fd;
         int res, desc_size = 0;
         char buf[8];
         struct hidraw_report_descriptor rpt_desc;
         struct hidraw_devinfo info;
         char *device = "/dev/hidraw1";
 
         if (argc > 1)
                 device = argv[1];
 
         /* Open the Device with non-blocking reads. In real life,
            don't use a hard coded path; use libudev instead. */
         fd = open(device, O_RDWR|O_NONBLOCK);
 
         if (fd < 0) {
                 perror("Unable to open device");
                 return 1;
         }
 
         memset(&rpt_desc, 0x0, sizeof(rpt_desc));
         memset(&info, 0x0, sizeof(info));
         memset(buf, 0x0, sizeof(buf));
 
 
 
                cout << "entering main" << endl;

                int fd_xbee = open_xbee_port();

                int8_t send_buf[NUM_JOY_BYTES];

                int i = 1;
                while(1)
                {       
                
                gettimeofday(&t_now,NULL);
         t = (t_now.tv_sec - t_init.tv_sec) ;
         t += (t_now.tv_usec - t_init.tv_usec)/1000000.;
        
         del_t=t-t_prev;
         t_prev=t;
         //printf("rate: %e\n",1/del_t);
        
         res = read(fd, buf, 8);
         if (res < 0) {
                 perror("read");
         } else {
                 //printf("read() read %d bytes:\n\t", res);
                 //for (i = 0; i < res; i++)
                         //printf("%hhx ", buf[i]);
                //         printf("%d \n", buf[i]);
                // puts("\n");
                 usleep(1000);
         }
         
         t_temp -= del_t*1000000;
         if(t_temp < 0)
         {
         	t_temp = 13000;	
         	del_t_=t-t_prev_;
         	t_prev_=t;
         	printf("rate: %e\n",1/del_t_);
         	 for (i = 0; i < res; i++)
                         //printf("%hhx ", buf[i]);
                         printf("%d ", buf[i]);
                         puts("\n");
                //if(buf[3] > 0) send_buf[0] = 256 + thrust_l - buf[3];
                //else send_buf[0] = thrust_l - buf[3];
                
                if(buf[3] > 0) send_buf[0] = 127 - buf[3];
                else send_buf[0] = -128 - buf[3];  //range: -76 - 74, on receiving side, add 76
                
                
                if(buf[1] > 0) send_buf[1] = buf[1] - 127;
                else send_buf[1] = buf[1] + 128;
                
                if(buf[2] > 0) send_buf[2] = buf[2] - 127;
                else send_buf[2] = buf[2] + 128;
                send_buf[2] =  send_buf[2];
                
                if(buf[5] > 0) send_buf[3] = buf[5] - 127;
                else send_buf[3] = buf[5] + 128;
                
                send_buf_[0] = saturation_value(send_buf[0], 75, -75); // thrust
                send_buf_[1] = saturation_value(send_buf[1], 75, -75); // roll
                send_buf_[2] = saturation_value(send_buf[2], 75, -75)-1; // pitch
                send_buf_[3] = saturation_value(send_buf[3], 75, -75)-1; // yaw
                send_buf_[4] = buf[7];
                
                
                        i++;
                        send_buf[0] = 0xFD; //  253 
                        send_buf[1] = send_buf_[0];    // thrust
                        send_buf[2] = send_buf_[1];    // roll
                        send_buf[3] = send_buf_[2];    // pitch
                        send_buf[4] = send_buf_[3];    // yaw
                        send_buf[5] = send_buf_[4];  //estop
                        send_buf[6] = 0xAD;
                        uint16_t sum = send_buf[0] + send_buf[1] + send_buf[2] + send_buf[3] + send_buf[4] + send_buf[5] + send_buf[6];
                        uint8_t low  = sum & 0xFF;
                        uint8_t high = (sum>>8);
                         printf("thrust: %d, %d, %d, %d, %d\n",  send_buf[1], send_buf[2], send_buf[3], send_buf[4],send_buf[5]);
                        

                        send_buf[7] = low;
                        send_buf[8] = high;
                        
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
        }
