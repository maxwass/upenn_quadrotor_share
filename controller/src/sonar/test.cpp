#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <math.h>
#include <fcntl.h>
#include <termios.h>

#include "sonar_receiver.h"

//#define BAUDRATE B115200

int usb0, res, range;
unsigned char sensor_bytes2[24];
bool SYSTEM_RUN = true;
char range_array[5];
struct timeval t_init, t_now;
double t_prev = 0.0;
double t, del_t;
char sprintf_buffer[10000000];
int sprintf_buffer_loc = 0;

int main()
{
	gettimeofday(&t_init,NULL);
	
	printf("Opening an USB port...   ");//Opens the I2C Port
	usb0 = open_usbport();
	if (usb0 < 0) printf("\n Error opening an USB0 port!!\n");
        else	     printf("Done!\n");
        	
        FILE *file;
 	file = fopen("data.txt","w");
        
	usleep(100000);
    
	while(t < 20.)
	{
		
		gettimeofday(&t_before,NULL);
		gettimeofday(&t_now,NULL);
		t = (t_now.tv_sec - t_init.tv_sec) ;
		t += (t_now.tv_usec - t_init.tv_usec)/1000000.;
		
		printf("%e\n",t);
		
		del_t=t-t_prev;
		t_prev=t;
        
        
		tcflush(usb0, TCIFLUSH);
		usleep(10000);
		res = read(usb0,&sensor_bytes2[0],1);
		

			
		if (sensor_bytes2[0] == 82 )
		{
			//range = (sensor_bytes2[1] << 12 | sensor_bytes2[2] << 8 | sensor_bytes2[3] << 4 | sensor_bytes2[4]);
		        res = read(usb0, &sensor_bytes2[1],4);
			range_array[0] = sensor_bytes2[1]; //
			range_array[1] = sensor_bytes2[2];
			range_array[2] = sensor_bytes2[3];
			range_array[3] = sensor_bytes2[4];
			range_array[4] = '\0';
			
			printf("%c%c%c\n",sensor_bytes2[1], sensor_bytes2[2], sensor_bytes2[3] );
			sscanf(range_array, "%d", &range);
			printf("%d \n",range); //4 bytes make up this 32 bit integer, distance in cm
			for (int i = 0;  i<6; i++){
				printf("%d, ",sensor_bytes2[i]);
			}
			printf("\n\n");
		}
		
	//	printf("%d \n", range);
		
	//	printf("%d, %d, %d, %d, %d \n", sensor_bytes2[0], sensor_bytes2[1], sensor_bytes2[2], sensor_bytes2[3], sensor_bytes2[4]);
		usleep(1000);
		
		if(sprintf_buffer_loc < sizeof(sprintf_buffer))
        {
           int sprintf_size = sprintf(sprintf_buffer+sprintf_buffer_loc,"%E %d\n", t, range);

            sprintf_buffer_loc+=sprintf_size;
        }
       else if(sprintf_buffer_loc >= sizeof(sprintf_buffer))
        {
            printf("Warning: sprintf_buffer is full! \n");
        }

	}
	
	printf("Opening text file.....  ");
    printf("Saving buffer to a file.....  ");
    fwrite(sprintf_buffer, 1, sprintf_buffer_loc,file);
    printf("free buffer file.....  ");
    fclose(file);
    

}
