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



//CODE USED TO SEND JOYSTICK DATA

/*
  2  * Hidraw Userspace Example
  3  *
  4  * Copyright (c) 2010 Alan Ott <alan@signal11.us>
  5  * Copyright (c) 2010 Signal 11 Software
  6  *
  7  * The code may be used by anyone for any purpose,
  8  * and can serve as a starting point for developing
  9  * applications using hidraw.
 10  */
 #include <linux/types.h>
 #include <linux/input.h>
 #include <linux/hidraw.h>
 #include <sys/ioctl.h>
 #include <sys/types.h>
 #include <sys/stat.h>
 #include <fcntl.h>
 #include <unistd.h>
 #include <termios.h>
 #include <stdio.h>
 #include <string.h>
 #include <stdlib.h>
 #include <errno.h>
 #include <sys/time.h>
 #include "joystick_send.h"
 #include "Xbee.h"
  /*
 18  * Ugly hack to work around failing compilation on systems that don't
 19  * yet populate new version of hidraw.h to userspace.
 20  */
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
 
 
 int main(int argc, char **argv)
 {
 	 float send_buf[5];
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
	int usb0 = open_port();
	if (usb0 <0)
        	printf("\n Error opening an USB0 port!!\n");
        else
        	printf("Done!\n");
        usleep(14000);
 	
         int fd;
         int i, res, desc_size = 0;
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
                if(buf[3] > 0) send_buf[0] = 256 + thrust_l - buf[3];
                else send_buf[0] = thrust_l - buf[3];
                
                
                if(buf[1] > 0) send_buf[1] = buf[1] - 127;
                else send_buf[1] = buf[1] + 128;
                
                if(buf[2] > 0) send_buf[2] = buf[2] - 127;
                else send_buf[2] = buf[2] + 128;
                send_buf[2] =  send_buf[2];
                
                if(buf[5] > 0) send_buf[3] = buf[5] - 127;
                else send_buf[3] = buf[5] + 128;
                
                send_buf[0] = saturation_value(send_buf[0], 150, 0); // thrust
                send_buf[1] = saturation_value(send_buf[1], 75, -75); // roll
                send_buf[2] = saturation_value(send_buf[2], 75, -75)-1; // pitch
                send_buf[3] = saturation_value(send_buf[3], 75, -75)-1; // yaw
                send_buf[4] = buf[7];
                
                int fd_send = XBee_send_float(usb0,send_buf,5);
	
		tcflush(usb0,TCOFLUSH);
		if (fd_send < 0)
		printf("sent: %d\n",fd_send);
                
                printf("thrust: %e, %e, %e, %e, %e\n", send_buf[0], send_buf[1], send_buf[2], send_buf[3], send_buf[4]);
         	
         }
         
         
         }
         close(fd);
         close(usb0);
         return 0;
 }
 
 const char *
 bus_str(int bus)
 {
         switch (bus) {
         case BUS_USB:
                 return "USB";
                 break;
         case BUS_HIL:
                 return "HIL";
                 break;
         case BUS_BLUETOOTH:
                 return "Bluetooth";
                 break;
         case BUS_VIRTUAL:
                 return "Virtual";
                 break;
         default:
                 return "Other";
                 break;
         }
 }
 
