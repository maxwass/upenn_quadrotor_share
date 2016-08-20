#include <pthread.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include "imu.h"
#include "receiver.h"
#include <math.h>
#include "serial1.h"
#include "Xbee.h"

#define NUM_THREADS 3
#define PI 3.14159265359

#define LIDAR_LITE_ADRS 0x62
#define MEASURE_VAL 0x04
#define MEASURE_REG 0x00
#define STATUS_REG  0x47
#define DISTANCE_REG_HI 0x0f
#define DISTANCE_REG_LO 0x10  
#define VERSION_REG 0x41
#define ERROR_READ -1
// Status Bits
#define STAT_BUSY               0x01
#define STAT_REF_OVER           0x02
#define STAT_SIG_OVER           0x04
#define STAT_PIN                0x08
#define STAT_SECOND_PEAK        0x10
#define STAT_TIME               0x20
#define STAT_INVALID            0x40
#define STAT_EYE                0x80
#define DISTANCE_CONT		0x8f

pthread_mutex_t data_acq_mutex; 
pthread_cond_t data_acq_cv;

struct timeval t_init, t_now, t_now_v, t_before;
double t_v, del_t_v, t_landing, t_prev_v = 0.0;
double t_prev = 0.0;
double t, del_t;
bool SYSTEM_RUN = true;
bool CONTROLLER_RUN = false;
bool DISPLAY_RUN = true;
bool CTRL_LANDING = false;

 int i2cHandle, usb_imu, usb_xbee, res1;
 int motor_out[4] = {10, 10, 10, 10};
 int F_init = 30;
float phi_d = 0.0;
float theta_d = 0.0;\
float psi_d = 0.0;
float phi, theta, psi;
float e_phi, e_phi_dot, e_theta, e_theta_dot, e_psi, e_psi_dot, psi_dot;
int U_trim[4], ff[4], U_thrust;
float U[4];
float psi_init;
int  motor_trim[4] = {0,0,0,0};
float kp_phi = 22.3;
float kd_phi = 0.35;

float kp_theta = 22.3;
float kd_theta = 0.35;

float kp_psi = 18.0;
float kd_psi = 2.41;

char sprintf_buffer[10000000];
int sprintf_buffer_loc = 0;

float pos[3], vel[3], psi_v, phi_v, theta_v, pos_filtered[3], vel_filtered[3];
float xd, yd, zd, ex, ey, ez, ex_i, ey_i, ez_i;
float delta_t = 0.01;
float kp_g = 19.5;//16.5;
float ki_g = 0.05;
float kd_g = 2.7;//2.3;

int Get_Range_Values(){
	char range_read_lo[2] = { 0 };
	char range_read_hi[2] = { 0 };
	unsigned char sensor_read_hi;
	unsigned char sensor_read_lo;
	unsigned char write_buffer[2];
	unsigned char write_buffer2[2];

	write_buffer[0] = MEASURE_REG;
	write_buffer[1] = MEASURE_VAL;
	write_buffer2[0] = DISTANCE_REG_LO;
	write_buffer2[1] = DISTANCE_REG_HI;


	int hiVal, loVal, i=0, res_range, LoVal;
	res_range = write(i2cHandle, &write_buffer[0], 2);
	if (res_range < 0) printf("Error writing REG\n");
	usleep(50);
	
	
	res_range = write(i2cHandle, &write_buffer2[0] , 1);
	if (res_range < 0) printf("Error writing Lo\n");
	usleep(50);
	res_range = read(i2cHandle, range_read_lo,1);
	if (res_range < 0) printf("Error reading Lo1\n");
	usleep(50);
	
	res_range = write(i2cHandle, &write_buffer2[1] , 1);
	if (res_range < 0) printf("Error writing hi\n");
	usleep(50);
	res_range = read(i2cHandle, range_read_hi,1);
	if (res_range < 0) printf("Error reading hi1\n");
	usleep(50);

	return( (range_read_hi[0] << 8) + range_read_lo[0]);
}

int getch(void)
{
    struct termios oldattr, newattr;
    int ch;
    tcgetattr( STDIN_FILENO, &oldattr );
    newattr = oldattr;
    newattr.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newattr );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldattr );
    return ch;
}


void *command_input(void *thread_id)
{
 unsigned char command_key;
 
    U_trim[3] = 0;
    motor_trim[1] = 2;
    motor_trim[3] = -2;
    
    int stick_input = 10;
    
    motor_trim[0] = 1;
    motor_trim[2] = -1;
    while(SYSTEM_RUN == true) {
    command_key = getch();
	if (command_key == 't' || command_key == 'T')
	{
		if(CONTROLLER_RUN == false)
			{printf("Controller ON!!\n");
			CONTROLLER_RUN = true;}
		else if(CONTROLLER_RUN == true)
			{printf("Controller OFF!!\n");
			U_trim[0] = 0;
			SYSTEM_RUN = false;
		CONTROLLER_RUN = false;}
	}
	else if (command_key == 'q' || command_key == 'Q')
	{printf("Motor Stop!\n");
	motor_out[0] = 0;
	motor_out[1] = 0;
	motor_out[2] = 0;
	motor_out[3] = 0;
	U_trim[0] = 0;
	CONTROLLER_RUN = false;
	SYSTEM_RUN = false;
	
	}

	else if (command_key == 'p' || command_key == 'P')
	{
	if(DISPLAY_RUN == false)
	{DISPLAY_RUN = true;}
	else if(DISPLAY_RUN == true)
	{DISPLAY_RUN = false;}
	}

	else if (command_key == 'i' || command_key == 'I')
	{
	U_trim[0] = U_trim[0] + 10;
	if(U_trim[0] > 460) U_trim[0] = 460;
	printf("Increase Thrust to %d\n", U_trim[0]);
	}
	else if (command_key == 'k' || command_key == 'K')
	{
	U_trim[0] = U_trim[0] - 10;
	printf("Decrease Thrust to %d\n", U_trim[0]);
	}


	else if (command_key == 'y' || command_key == 'Y')
	{
	kp_g = kp_g + 0.3;
	//kp_phi = kp_phi + 5;
	//kp_theta = kp_theta + 5;
	
	printf("P gain up: %e\n", kp_g);
	//printf("P gain up: %e\n", kp_g);
	}
	else if (command_key == 'h' || command_key == 'H')
	{
	kp_g = kp_g - 0.3;
	//kp_phi = kp_phi - 5;
	//kp_theta = kp_theta - 5;
	//printf("P gain down: %e\n", kp_phi);
	printf("P gain down: %e\n", kp_g);
	}
	
	else if (command_key == 'u' || command_key == 'U')
	{
	kd_g = kd_g + 0.1;
	//kd_phi = kd_phi + 0.2;
	//kd_theta = kd_theta + 0.2;
	//printf("D gain up: %e\n", kd_phi);
	printf("D gain up: %e\n", kd_g);
	}
	else if (command_key == 'j' || command_key == 'J')
	{
	kd_g = kd_g - 0.1;
	//kd_phi = kd_phi - 0.2;
	//kd_theta = kd_theta - 0.2;
	//printf("D gain down: %e\n", kd_phi);
	printf("D gain down: %e\n", kd_g);
	}
	
	
	else if (command_key == 'o' || command_key == 'O')
	{
	xd = pos[0];
	yd = pos[1];
	zd = pos[2];
	printf("Set Desired Position\n");
	}
	
	
	else if (command_key == ' ' || command_key == '\n')
	{
		t_landing = t;
		CTRL_LANDING = true;
	}
	
	
	else if (command_key == 'w' || command_key == 'W')
	{
		xd = xd + 0.1;
	}
	else if (command_key == 's' || command_key == 'S')
	{
		xd = xd - 0.1;
	}
	else if (command_key == 'a' || command_key == 'A')
	{ 
		yd = yd - 0.1;
	}
	else if (command_key == 'd' || command_key == 'D')
	{
		yd = yd + 0.1;
	}
	else if (command_key == 'r' || command_key == 'R')
	{ 
		zd = zd - 0.1;
	}
	else if (command_key == 'f' || command_key == 'F')
	{
		zd = zd + 0.1;
	}

    }
    pthread_exit(NULL);
}






void *control_stabilizer(void *thread_id)
{

 int i;
 int address[4] = {0x2b, 0x2a, 0x2c, 0x29}; //The address of i2c
 int motorspeed, res_mtr_out;
 unsigned char sensor_bytes2[24];
 double Ct=0.013257116418667*10;
 double d=0.169;
 float psi_ = 0.0, phi_ = 0.0, theta_ = 0.0;
 int range_res;
 float range_out;
 
 FILE *file;
 file = fopen("data.txt","w");
    
    while(SYSTEM_RUN == true) {

	gettimeofday(&t_before,NULL);
        gettimeofday(&t_now,NULL);
        t = (t_now.tv_sec - t_init.tv_sec) ;
        t += (t_now.tv_usec - t_init.tv_usec)/1000000.;
        
        del_t=t-t_prev;
        t_prev=t;
	ioctl(i2cHandle,I2C_SLAVE,LIDAR_LITE_ADRS);
	range_res = Get_Range_Values();

	SImu_data imu_data;
	tcflush(usb_imu, TCIFLUSH);
	res1 = read(usb_imu,&sensor_bytes2[0],24);

	imu_data.theta = *(float *)&sensor_bytes2[4];
	imu_data.phi = *(float *)&sensor_bytes2[8];
	imu_data.psi = *(float *)&sensor_bytes2[0];
	imu_data.phi_dot = *(float *)&sensor_bytes2[12];
	imu_data.theta_dot = *(float *)&sensor_bytes2[16];
	imu_data.psi_dot = *(float *)&sensor_bytes2[20];
	//printf("psi: %e\n",imu_data.psi);
	//if(imu_data.psi > 350.0)
	//imu_data.psi = imu_data.psi - 360.0;
	
	psi_dot = ( psi_v - psi_)/delta_t;
	psi_ = psi_v;
	//imu_data.phi = phi_v * 180/PI;
	//imu_data.theta = theta_v * 180/PI;
	
	phi = phi_v * 180/PI;
	theta = theta_v * 180/PI;
	//phi = imu_data.phi;
	//theta = imu_data.theta;
	
	range_out = range_res * cos(phi_v) * cos(theta_v) / 100;
	//printf("%3.3f m  %d Hz: %e\n", range_out*100, range_res, 1/del_t);
	//imu_data.phi_dot = (imu_data.phi - phi_)/delta_t;
	//imu_data.theta_dot = (imu_data.theta - theta_)/delta_t;
	//phi_ = imu_data.phi;
	//theta_ = imu_data.theta;

	//printf("phi, theta, psi, phi_dot, theta_dot, psi_dot: %E, %E, %E, %E, %E, %E \n", imu_data.phi, imu_data.theta, imu_data.psi, imu_data.phi_dot, imu_data.theta_dot, imu_data.psi_dot);
	tcflush(usb_imu, TCIFLUSH);

	e_phi = (-phi + phi_d) * PI/180;
	e_theta = (-theta + theta_d) * PI/180;
	e_psi = -psi_v + psi_d * PI/180;
	e_phi_dot = (-imu_data.phi_dot) * PI/180;
	e_theta_dot = (-imu_data.theta_dot) * PI/180;
	e_psi_dot = -psi_dot;
	
	if (CTRL_LANDING == true)
	{
		if (t - t_landing < 1.5)
		{
			U_trim[0] = U_trim[0] - 15;
		}
		else
		{
			U_trim[0] = 0;
		}
			
	}

	U[0] = 30 + U_trim[0] + U_thrust;
	U[1] = kp_phi * e_phi + kd_phi * e_phi_dot;
	U[2] = kp_theta * e_theta + kd_theta * e_theta_dot;
	U[3] = kp_psi * e_psi + kd_psi * e_psi_dot;
	
    
    	 //printf("Moment: %E %E %E\n",M[0],M[1],M[2]);
   	 //Calculating the forces of each motor
    
    	ff[0] = (U[0]/4-(U[3]/(4*Ct))+(U[2]/(2*d))) + motor_trim[0];
   	ff[1] = (U[0]/4+(U[3]/(4*Ct))-(U[1]/(2*d))) + motor_trim[1];
   	ff[2] = (U[0]/4-(U[3]/(4*Ct))-(U[2]/(2*d))) + motor_trim[2];
   	ff[3] = (U[0]/4+(U[3]/(4*Ct))+(U[1]/(2*d))) + motor_trim[3];

   	int i;
	for(i=0;i<4;i++)
		{
		if(ff[i] < 0.0) ff[i] = 0.0;

		if(CONTROLLER_RUN == true)
    		{
			motor_out[i] = ff[i];
		}
		else
			motor_out[i] = 0.0;

		if(motor_out[i] > 240.0) motor_out[i]=240.0;
		else if(motor_out[i] < 0.0)	motor_out[i]=0.0;
		}	
		
	for(i=0;i<4;i++)
	{
	 ioctl(i2cHandle,I2C_SLAVE,address[i]);
	 motorspeed = (int)motor_out[i];
	 res_mtr_out = write(i2cHandle, &motorspeed,1);
	}

	if(DISPLAY_RUN == true)
	{
		printf("<==========================================>\n");
		if(CONTROLLER_RUN == true) printf("Controller ON \n");
		else if (CONTROLLER_RUN == false) printf("Controller OFF \n");
		printf("time: %e\n", t);
		printf("	IMU DATA	\n");
		printf("phi: %.2f         phi dot: %.2f\n",imu_data.phi, imu_data.phi_dot);
		printf("theta: %.2f         theta dot: %.2f\n",imu_data.theta, imu_data.theta_dot);
		printf("psi: %.2f         psi dot: %.2f\n\n\n",imu_data.psi, imu_data.psi_dot);

		printf("	GAINS		\n");
		printf("kp_phi: %f	kd_phi: %f\n",kp_phi, kd_phi);
		printf("kp_theta: %f	kd_theta: %f\n",kp_theta, kd_theta);
		printf("kp_psi: %f	kd_psi: %f\n\n\n",kp_psi, kd_psi);

		printf("	MOTOR_OUT	\n");
		printf("Motor #1: %d,      raw: %d,	trim: %d\n", motor_out[0], ff[0], motor_trim[0]);
		printf("Motor #2: %d,      raw: %d	trim: %d\n", motor_out[1], ff[1], motor_trim[1]);		
		printf("Motor #3: %d,      raw: %d	trim: %d\n", motor_out[2], ff[2], motor_trim[2]);
		printf("Motor #4: %d,      raw: %d	trim: %d\n\n\n", motor_out[3], ff[3], motor_trim[3]);
		
		printf("	Errors		\n");
		printf("e_phi: %f,	e_theta: %f,	e_psi: %f\n\n\n", e_phi*180/PI, e_theta*180/PI, e_psi*180/PI);
	}

	if(sprintf_buffer_loc < sizeof(sprintf_buffer))
        {
           int sprintf_size = sprintf(sprintf_buffer+sprintf_buffer_loc,"%E %E %E %E %E  %E %E %E %E %E   %e %e %e %e %E   %E %E %E %E %E   %E %E %E %E %E   %E %E %E %E %E   %E %E %E %E %d\n", t, imu_data.phi, imu_data.theta, psi_v, imu_data.phi_dot, imu_data.theta_dot, imu_data.psi_dot, e_phi*180/PI, e_theta*180/PI, e_psi*180/PI, U[0], U[1], U[2], U[3], pos[0], pos[1], pos[2], xd, yd, zd, vel[0], vel[1], vel[2], phi_d, theta_d, pos_filtered[0], pos_filtered[1], pos_filtered[2], vel_filtered[0], vel_filtered[1], vel_filtered[2], phi_v, theta_v, range_out, range_res/100);

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
    pthread_exit(NULL);
}







void *vicon(void *thread_id)
{

float pos_1[3] = {0.0};
float pos_2[3] = {0.0};
float pos_filtered_1[3] = {0.0};
float vel_deriv[3] = {0.0};
float vel_1[3] = {0.0};
float vel_2[3] = {0.0};
float data_received[6];
int bytes_received, i;

while(SYSTEM_RUN == true) {

	tcflush(usb_xbee,TCIFLUSH);
	bytes_received=XBee_receive_float(usb_xbee,data_received,6);
	usleep(14000);
	
	pos[0] = data_received[0];
        pos[1] = data_received[1];
        pos[2] = data_received[2];
        phi_v = data_received[3];
        theta_v = data_received[4];
        psi_v = data_received[5];
        tcflush(usb_xbee,TCIOFLUSH);
        
        for(i=0;i<3;i++){
        	pos_filtered[i] = 0.7*pos[i] + 0.2*pos_1[i] + 0.1*pos_2[i];
        	pos_2[i] = pos_1[i];
        	pos_1[i] = pos_filtered[i];
        }
        
        for(i=0;i<3;i++){
        	vel[i] = (pos_filtered[i] - pos_filtered_1[i])/delta_t;
        	pos_filtered_1[i] = pos_filtered[i];
        }
        
        for(i=0;i<3;i++){
        	vel_filtered[i] = 0.7*vel[i] + 0.2*vel_1[i] + 0.1*vel_2[i];
        	vel_2[i] = vel_1[i];
        	vel_1[i] = vel_filtered[i];
        }
        
    /*    
        delt = 0.012;
vel_deriv_1 = zeros(1,3);
vel_deriv_2 = zeros(1,3);
a = 0.2;
b = 0.3;
c = 0.5;
for i = 3:size(data,1)
    pos_filt(i,:) = 0.4*pos(i,:)+0.4*pos(i-1,:)+0.2*pos(i-2,:);
    vel_deriv(i,:) = (pos_filt(i,:) - pos_filt(i-1,:))/delt;
    vel_filt(i,:) = a*vel_deriv(i,:) + b*vel_deriv_1 + c*vel_deriv_2;
    vel_deriv_2 = vel_deriv_1;
    vel_deriv_1 = vel_filt(i,:);
end
       */ 
        
        ex = xd - pos_filtered[0];
        ey = yd - pos_filtered[1];
        ez = zd - pos_filtered[2];
        
        for(int i=0;i<3;i++)
        {
        	if(vel_filtered[i] > 5.0) vel_filtered[i]=5.0;
        	else if(vel_filtered[i] < -5.0) vel_filtered[i] = -5.0;
        }
		
        
        if(CONTROLLER_RUN == true){
        ex_i = ex_i + ex * del_t;
        ey_i = ey_i + ey * del_t;
        ez_i = ez_i + ez * del_t;
        U_thrust = (int)( -12.0*ez - 5.0*ez_i - 5.0*vel_filtered[2]);
        }
        else{
        ex_i = 0;
        ey_i = 0;
        ez_i = 0;
        U_thrust = 0;
        }
        
        phi_d =   kp_g*ey + ki_g*ey_i - kd_g*vel_filtered[1];
        theta_d =  - kp_g*ex - ki_g*ex_i + kd_g*vel_filtered[0];
        
        if (U_thrust > 300.0)
        U_thrust = 300.0;
        //15 degrees
	double sat_val = 15.0;
        if (phi_d > sat_val)
        phi_d = sat_val;
        else if (phi_d < -sat_val)
        phi_d = -sat_val;
        
        if (theta_d > sat_val)
        theta_d = sat_val;
        else if (theta_d < -sat_val)
        theta_d = -sat_val;
        
        }
	
    pthread_exit(NULL);
}



int main(void)
{

 pthread_t threads[3];
 pthread_attr_t attr;
 struct sched_param	param;
 int fifo_max_prio, fifo_min_prio;

 system("clear");
 gettimeofday(&t_init,NULL);
   
 printf("opening i2c port...\n");
 i2cHandle = open("/dev/i2c-4",O_RDWR);

	if (i2cHandle > 0)
		printf("Done!\n");
	else
		printf("Fail to open i2c port!\n");
 usleep(10000);

 printf("Opening an USB port...   ");//Opens the usb Port
 usb_xbee = open_usbport();
	if (usb_xbee <0)
        	printf("\n Error opening an USB0 port!!\n");
        else
        	printf("Done!\n");
 usleep(10000);

 printf("opening usb port for imu...\n");
 usb_imu = open_port();
	if (usb_imu > 0)
		printf("Done!\n");
	else
		printf("Fail to open usb port!\n");
 usleep(100000);

  // Initialize mutex and condition variables
    pthread_mutex_init(&data_acq_mutex, NULL);
    
    // Set thread attributes
    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    fifo_max_prio = sched_get_priority_max(SCHED_FIFO);
    fifo_min_prio = sched_get_priority_min(SCHED_FIFO);
    
    // Create threads
    // Higher priority for filter
    param.sched_priority = fifo_max_prio;
    pthread_attr_setschedparam(&attr, &param);
    pthread_create(&threads[0], &attr, control_stabilizer, (void *) 0);
    
    // Medium priority for vicon
    param.sched_priority = (fifo_max_prio+fifo_min_prio)/2;
    pthread_attr_setschedparam(&attr, &param);
    pthread_create(&threads[1], &attr, vicon, (void *) 1);
    
    // Lower priority for vicon
    param.sched_priority = (fifo_max_prio+fifo_min_prio)/2;
    pthread_attr_setschedparam(&attr, &param);
    pthread_create(&threads[2], &attr, command_input, (void *) 2);
    
    // Wait for all threads to complete
    for (int i = 0; i < NUM_THREADS; i++)
    {
        pthread_join(threads[i], NULL);
    }
    
    close(i2cHandle);
    close(usb_xbee);
    close(usb_imu);
    
    pthread_attr_destroy(&attr);
    pthread_mutex_destroy(&data_acq_mutex);
    return 0;
    

}

