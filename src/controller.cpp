 #include "controller.h"
//g++ controller.cpp imu.cpp sonar.cpp xbee1.cpp vicon.cpp motor.cpp logger.cpp utility.cpp -I ../include -lpthread -lncurses -lboost_system -std=c++11
//initialize process-scoped data-structures

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

Times times;
Times time_m;
Times times_display;

Positions init_positions = {0.0};
Positions desired_positions = {0.0};
Gains gains= {0.0};
Control_command U_trim = {0};

bool MAGN = false;
bool SYSTEM_RUN = true;
bool CONTROLLER_RUN = false;
bool ESTOP = true;
bool XConfig = true;
bool AUTO_HEIGHT = false;
bool DISPLAY_RUN = true;
bool LOG_DATA = false;
int VICON_OR_JOY = 0; // 1 = VICON, 0 = JOYSTICK
int i2cHandle, usb_imu_ivn, usb_xbee;
uint16_t display_count=0;
bool SONAR_BUBBLE = true;
int repulsion_factor = 15;
int minDist = 500;
int maxDist = 2000;

int aa = 0;
timespec s,f;
timespec dd,tt;
int port;

std::string log_filename = "file.log";
logger logger(log_filename, 10, LOG_DATA);

//create our motor objects - accesible from all threads
motor motor_1(1, 0x29); //0x2f
motor motor_2(2, 0x2c); //0x2d
motor motor_3(3, 0x2a); //0x30
motor motor_4(4, 0x2b); //0x2e

SonarTest x_pos ={0}, x_neg ={0}, y_pos ={0}, y_neg ={0}; 

Sonar sonar_x_pos("/dev/ttyUSB1");
Sonar sonar_x_neg("/dev/ttyUSB2");
Sonar sonar_y_pos("/dev/ttyUSB0");
Sonar sonar_y_neg("/dev/ttyUSB4");

std::string path = "/dev/ttyACM0";
Imu imu(path, 26, .007);

Xbee joystick("/dev/ttyUSB3",9);

#define ncurse 1
#if ncurse ==1
	#define printf(...) printw(__VA_ARGS__)
#endif

void *control_stabilizer(void *thread_id){

printf("in control stabilizer \n");
    State imu_data;
    int range_sonar_1, range_sonar_2, range_sonar_3, range_sonar_4; 
    
    Distances sonar_distances;
    Distances repulsive_forces;
    //weights is used for filter: current, one value ago, 2 values ago
    Weights weights = {.7,.2,.1};

    //for error calculations PID: stores the actual errors, not gains
    State_Error vicon_error = {0.0};
    
    //position from raw data
    Vicon new_vicon,          old_vicon,          old_old_vicon          = {0};  
    Vicon new_filt_vicon,     old_filt_vicon,     old_old_filt_vicon     = {0};

    //velocity from raw data
    Vicon new_vicon_vel,      old_vicon_vel,      old_old_vicon_vel      = {0};
    Vicon new_filt_vicon_vel, old_filt_vicon_vel, old_old_filt_vicon_vel = {0};

//for display
    State imu_error = {0};
    Control_command U = {0};
    Angles desired_angles = {0};
    uint8_t joystick_thrust , flight_mode = 0;
    int succ_read;
    int new_xbee_data, new_imu_data, new_sonar_data_x_pos, new_sonar_data_x_neg, new_sonar_data_y_pos, new_sonar_data_y_neg;
    times.delta.tv_nsec = delta_time; //500000;

	clock_gettime(CLOCK_REALTIME,&s);
	clock_gettime(CLOCK_REALTIME,&f);
    	clock_gettime(CLOCK_REALTIME,&dd);
        clock_gettime(CLOCK_REALTIME,&tt);
	
while(SYSTEM_RUN) {
	//calc new times and delta
	float dt = UTILITY::calcDt(dd,tt);    
	//time_calc(times_display);

	new_sonar_data_x_pos = sonar_x_pos.get_sonar_data(x_pos);
	new_sonar_data_x_neg = sonar_x_neg.get_sonar_data(x_neg);
	new_sonar_data_y_pos = sonar_y_pos.get_sonar_data(y_pos);
	new_sonar_data_y_neg = sonar_y_neg.get_sonar_data(y_neg);

	//reads input from imu (in degrees), distributes into fields of imu_data
	//get_imu_data returns 1 if read successful,< 0 if not. Reuse old values if not successful
	int new_imu_data = imu.get_imu_calibrated_data(imu_data);
	

	//can switch between psi estimators
	if(!MAGN) imu_data.psi = imu_data.psi_gyro_integration;

	if (VICON_OR_JOY == 1)
	{
		printf("in vicon part");
		//get_vicon_data(usb_xbee, new_vicon);

		new_filt_vicon = filter_vicon_data(new_vicon, old_vicon, old_old_vicon, weights);

		//calc velocities from vicon
		new_vicon_vel = vicon_velocity(new_filt_vicon, old_filt_vicon);
		//filter velocities
		new_filt_vicon_vel = filter_vicon_data(new_vicon_vel, old_vicon_vel, old_old_vicon_vel, weights);       
		//set old_old data to old_data, and old_data to new data
		//vicon data
		pushback(new_vicon,      old_vicon,      old_old_vicon);
		pushback(new_filt_vicon, old_filt_vicon, old_old_filt_vicon);

		//calculated vicon velocities
		pushback(new_vicon_vel,      old_vicon_vel,      old_old_vicon_vel);
		pushback(new_filt_vicon_vel, old_filt_vicon_vel, old_old_filt_vicon_vel);

		//calculate error from vicon
		error_vicon(vicon_error, new_filt_vicon, new_filt_vicon_vel, desired_positions,times);	

		//calculate desired attitude (phi theta phi) in desired_angles
		desired_angles_calc(desired_angles, vicon_error, gains);
	}	
	else
	{
		Angles old_desired_angles = desired_angles;
		//get joystick data => desired angles
		//new_xbee_data = select_get_joystick_data(usb_xbee, desired_angles, joystick_thrust, flight_mode);
		new_xbee_data = joystick.get_xbee_data(desired_angles,joystick_thrust,flight_mode);


		if(new_xbee_data < 0) ; //printf("joystick not ready to read: old data");
		//check flight mode
		    if(ESTOP)
		    {
			    if(flight_mode < 11.0) 
				{
					//printf("FLIGHT MODE: OFF %i \n", flight_mode); 
					CONTROLLER_RUN = false;
				}
			    else if (flight_mode > 15.0 && flight_mode < 17.0) AUTO_HEIGHT = false;
			    else if (flight_mode > 17.0) AUTO_HEIGHT = true;
		    }
	}

	//calculate error from imu (in radians) between desired and measured state
	State imu_error = error_imu(imu_data, desired_angles);

	//calculate thrust and desired acceleration
	U = thrust(imu_error,vicon_error, U_trim, joystick_thrust, gains);

	//calculate the forces of each motor and change force on motor objects and send via i2c 
	set_forces(U,Ct,d);

	//printf("BOOOOL: %i\n", (LOG_DATA && ( (new_xbee_data>0) || (new_imu_data>0) || (new_sonar_data_1>0) || (new_sonar_data_2>0) || (new_sonar_data_3>0) || (new_sonar_data_4>0) ) ) );
	if (LOG_DATA && ( (new_xbee_data>0) || (new_imu_data>0) || (new_sonar_data_x_pos>0) || (new_sonar_data_x_neg>0) || (new_sonar_data_y_pos>0) || (new_sonar_data_y_neg>0) ) )   
	//if(true)
	{
	 clock_gettime(CLOCK_REALTIME,&f);
	 //printf("freq new data is available: %f \n", 1/joystick.calcDt(s,f));
	 clock_gettime(CLOCK_REALTIME,&s);
	 //printf("number of times new data is available: %i \n", aa++);
	log_data(sonar_distances, dt, new_vicon, new_vicon_vel, new_filt_vicon, new_filt_vicon_vel, vicon_error, imu_data, imu_error, desired_angles);       
	}
	if (DISPLAY_RUN) 
	{ 
	display_info(sonar_distances, new_xbee_data,  imu_data, vicon_error, imu_error, U, new_vicon, new_filt_vicon, new_vicon_vel, new_filt_vicon_vel, desired_angles,joystick_thrust, flight_mode, times_display, time_m); 
	}

}
 
    printf("EXIT CONTROL_STABILIZER\n");
    pthread_exit(NULL);

}

void desired_angles_calc(Angles& desired_angles, const State_Error& error, const Gains& gains){

    desired_angles.psi     = 0;
    desired_angles.phi     =  gains.kp_y*error.y.prop - gains.kd_y*error.y.deriv + gains.ki_y*error.y.integral; //5.y_pos_sensor - 5*y_neg_sensor
    desired_angles.theta   = -gains.kp_x*error.x.prop + gains.kd_x*error.x.deriv - gains.ki_x*error.x.integral; //5.x_pos_sensor - 5*x_neg_sensor

}
State_Error error_vicon(State_Error& error, const Vicon& pos_filt, const Vicon& vel_filt, const Positions& desired_positions, const Times& times){
       
    //proportional errors:  desired_positions - filtered_positions
    error.x.prop = desired_positions.x - pos_filt.x;
    error.y.prop = desired_positions.y - pos_filt.y;
    error.z.prop = desired_positions.z - pos_filt.z;
       
    //derivative errors: desired_velocities - filtered_velocities
    error.x.deriv = 0 - vel_filt.x;
    error.y.deriv = 0 - vel_filt.y;
    error.z.deriv = 0 - vel_filt.z;
       
    //integral errors: integral error + (proportional error * delta_t)
    error.x.integral = error.x.integral + (error.x.prop * tv2float(times.delta));
    error.y.integral = error.y.integral + (error.y.prop * tv2float(times.delta));
    error.z.integral = error.z.integral + (error.z.prop * tv2float(times.delta));

}

void *motor_signal(void *thread_id){

   //printf("INSIDE MOTOR_SIGNAL\n");
   set_initial_times(time_m);

      while(SYSTEM_RUN){
	//printf("in motor loop");
	motor_1.send_force_i2c();
	motor_2.send_force_i2c();
	motor_3.send_force_i2c();
	motor_4.send_force_i2c();

    	time_calc(time_m);
	
	//sets frequency of motor_signal
	usleep(10);
	}

   printf("EXIT MOTOR_SIGNAL\n");

   pthread_exit(NULL);
}


void start_motors(void){
    //set speed to 30 out of 255
    printf("Starting Motors ...\n");
    motor_1.set_force(30, CONTROLLER_RUN);
    motor_2.set_force(30, CONTROLLER_RUN);
    motor_3.set_force(30, CONTROLLER_RUN);
    motor_4.set_force(30, CONTROLLER_RUN);
}
void stop_motors(void){
    printf("Stopping Motors ...\n");
    motor_1.set_force(0, false);
    motor_2.set_force(0, false);
    motor_3.set_force(0, false);
    motor_4.set_force(0, false);
}

void controller_on_off(bool &CONTROLLER_RUN){
    if(CONTROLLER_RUN == false){
        printf("Controller ON!!\n");
        CONTROLLER_RUN = true;
        }
    else{
        printf("Controller OFF!!\n");
        CONTROLLER_RUN = false;
        }
}
void display_on_off(bool& DISPLAY_RUN){
    if(DISPLAY_RUN == false){
        printf("DISPLAY ON!!\n");
        DISPLAY_RUN = true;
    }
    else if(DISPLAY_RUN == true){
        printf("DISPLAY OFF!!\n");
        DISPLAY_RUN = false;
    }
}
State error_imu(const State& imu_data, const Angles& desired_angles){
    //calculate error in RADIANS
    //  xxx_d is xxx_desired.  imu outputs  degrees, we convert to radians with factor PI/180
    // 
    State error;
    error.phi    =  (desired_angles.phi   - imu_data.phi)      * PI/180;
    error.theta  =  (desired_angles.theta - imu_data.theta)   * PI/180;

    if (SONAR_BUBBLE)
    {
	 error.phi   += repulsion_factor*(UTILITY::dist2ScaleInv(sonar_y_neg.returnLastDistance(), minDist, maxDist) - UTILITY::dist2ScaleInv(sonar_y_pos.returnLastDistance(), minDist, maxDist))  * PI/180;
         error.theta += repulsion_factor*(UTILITY::dist2ScaleInv(sonar_x_pos.returnLastDistance(), minDist, maxDist) - UTILITY::dist2ScaleInv(sonar_x_neg.returnLastDistance(), minDist, maxDist))  * PI/180;
    }

    error.psi       =     (-imu_data.psi  + desired_angles.psi) * PI/180;
    /*if(ncurse)clear();
    printf("error.psi(desired_psi - imu_psi) , %3.3f  imu_data.psi %3.3f, desired_angles.psi %3.3f\n", error.psi, imu_data.psi, desired_angles.psi);
    printf("error.theta(desired_psi - imu_theta) , %3.3f  imu_data.theta %3.3f, desired_angles.theta %3.3f\n", error.theta, imu_data.theta, desired_angles.theta);
    if(ncurse)refresh();
    */
    error.phi_dot   =                           (-imu_data.phi_dot) * PI/180;
    error.theta_dot =                         (-imu_data.theta_dot) * PI/180;
    error.psi_dot   =                           (-imu_data.psi_dot) * PI/180;
    return error;
}
Control_command thrust(const State& imu_error, const State_Error& vicon_error, const Control_command& U_trim, const int joystick_thrust, const Gains& gains){
    //calculate thrust and acceleration
    Control_command U = {0};
    
    if(VICON_OR_JOY == 1){
    	 int calc_thrust = (int) (-(gains.kp_z * vicon_error.z.prop)  -  (gains.kd_z * vicon_error.z.deriv) - (gains.ki_z * vicon_error.z.integral));
    	 U.thrust        =  calc_thrust + U_trim.thrust; }
    else{
	//U.thrust        =  floor(joystick_thrust * (4) ) + U_trim.thrust; //thrust from joystick
	U.thrust        = 4 * joystick_thrust  + U_trim.thrust;
    }
 	
    U.roll_acc  =  (gains.kp_phi   * imu_error.phi  )  +  (gains.kd_phi   * imu_error.phi_dot  )  + U_trim.roll_acc;
    U.pitch_acc =  (gains.kp_theta * imu_error.theta)  +  (gains.kd_theta * imu_error.theta_dot)  + U_trim.pitch_acc;
    U.yaw_acc   =  (gains.kp_psi   * imu_error.psi  )  +  (gains.kd_psi   * imu_error.psi_dot  )  + U_trim.yaw_acc;
   /*printf("U.yaw_acc %3.3f  =  (gains.kp_psi %3.3f   * imu_error.psi %3.3f  )  +  (gains.kd_psi %3.3f  * imu_error.psi_dot %3.3f  ) \n", 
                        U.yaw_acc, gains.kp_psi, imu_error.psi, gains.kd_psi, imu_error.psi_dot);
   if(ncurse)refresh(); 
   */
   if (U.thrust <= 0) U.thrust = 0;
   
    return U;
}

void set_forces(const Control_command& U, double Ct, double d){
      //calculate forces from thrusts and accelerations
    
  if(!XConfig)
    {//this is Plus Configuration
        double force_1 = (U.thrust/4 - (U.yaw_acc /(4*Ct)) + (U.pitch_acc /  (2*d)));
        double force_2 = (U.thrust/4 + (U.yaw_acc /(4*Ct)) - (U.roll_acc  /  (2*d)));
        double force_3 = (U.thrust/4 - (U.yaw_acc /(4*Ct)) - (U.pitch_acc /  (2*d)));
        double force_4 = (U.thrust/4 + (U.yaw_acc /(4*Ct)) + (U.roll_acc  /  (2*d)));
        motor_1.set_force( round(force_1), CONTROLLER_RUN );
        motor_2.set_force( round(force_2), CONTROLLER_RUN );
        motor_3.set_force( round(force_3), CONTROLLER_RUN );
        motor_4.set_force( round(force_4), CONTROLLER_RUN );
    }
      else
    {  
       //     U[1] = phi, U[2] = theta .... theta  = pitch, phi = roll, psi = yaw
    //round forces to be integers
        double force_1 = (U.thrust/4 + (U.pitch_acc/(2*d)));
        double force_2 = (U.thrust/4 - (U.roll_acc/(2*d)))  ;
        double force_3 = (U.thrust/4 - (U.pitch_acc/(2*d))); 
        double force_4 = (U.thrust/4 + (U.roll_acc/(2*d)))  ; 
        
        double x_param = sqrt(2)/2;
        
        double x1 = x_param * (force_1 + force_2)-(U.yaw_acc/(4*Ct));
        double x2 = x_param * (force_2 + force_3)+(U.yaw_acc/(4*Ct));
        double x3 = x_param * (force_3 + force_4)-(U.yaw_acc/(4*Ct));
        double x4 = x_param * (force_4 + force_1)+(U.yaw_acc/(4*Ct));
            
        motor_1.set_force( round(x1), CONTROLLER_RUN );
        motor_2.set_force( round(x2), CONTROLLER_RUN );
        motor_3.set_force( round(x3), CONTROLLER_RUN );
        motor_4.set_force( round(x4), CONTROLLER_RUN );
    }

}

Vicon vicon_velocity(Vicon& current, Vicon& old){
    
    Vicon velocity = {0.0};
    
    velocity.x     = (current.x - old.x)/tv2float(times.delta);
    velocity.y     = (current.y - old.y)/tv2float(times.delta);
    velocity.z     = (current.z - old.z)/tv2float(times.delta);
    velocity.theta = (current.theta - old.theta)/tv2float(times.delta);
    velocity.phi   = (current.phi - old.phi)/tv2float(times.delta);
    velocity.psi   = (current.psi - old.psi)/tv2float(times.delta);

    return velocity;    
}

void log_data(const Distances& sonar_distances, const float& dt, const Vicon& new_vicon, const Vicon& new_vicon_vel, const Vicon& new_filt_vicon, const Vicon& new_filt_vicon_vel, const State_Error& vicon_error, const State& imu_data, const State& imu_error, const Angles& desired_angles){

    Data_log d;
    d.dt   = dt;
    d.vicon_data      = new_vicon;
    d.vicon_vel       = new_vicon_vel;
    d.vicon_data_filt = new_filt_vicon;
    d.vicon_vel_filt  = new_filt_vicon_vel;
    d.vicon_error     = vicon_error;
    d.imu             = imu_data;
    d.imu_error       = imu_error;
    d.forces.motor_1  = *(motor_1.get_force());
    d.forces.motor_2  = *(motor_2.get_force());
    d.forces.motor_3  = *(motor_3.get_force());
    d.forces.motor_4  = *(motor_4.get_force());
    d.desired_angles  = desired_angles;
    	Distances sonarDist;
	
	sonarDist.x_pos = sonar_x_pos.returnLastDistance();
	sonarDist.x_neg = sonar_x_neg.returnLastDistance();
	sonarDist.y_pos = sonar_y_pos.returnLastDistance();
	sonarDist.y_neg = sonar_y_neg.returnLastDistance();
    	//printf(" x_pos) %i, x_neg) %i, y_pos) %i y_neg) %i \n",sonarDist.x_pos, sonarDist.x_neg, sonarDist.y_pos, sonarDist.y_neg);
	d.sonar_distances = sonarDist;

	RepForces scales;
	scales.x_pos = repulsion_factor*UTILITY::dist2ScaleInv(sonar_x_pos.returnLastDistance(), minDist, maxDist);
	scales.x_neg = repulsion_factor*UTILITY::dist2ScaleInv(sonar_x_neg.returnLastDistance(), minDist, maxDist);
	scales.y_pos = repulsion_factor*UTILITY::dist2ScaleInv(sonar_y_pos.returnLastDistance(), minDist, maxDist);
	scales.y_neg = repulsion_factor*UTILITY::dist2ScaleInv(sonar_y_neg.returnLastDistance(), minDist, maxDist);
	d.scales = scales;

	
    logger.log(d);
}

void display_info(const Distances& sonar_distances, const int succ_read,  const State& imu_data, const State_Error& vicon_error, const State& imu_error, const Control_command& U, const Vicon& vicon, const Vicon& vicon_filt, const Vicon& vicon_vel, const Vicon& vicon_vel_filt, const Angles& desired_angles,const int  joystick_thrust, const int  flight_mode, const Times& times, const Times& time_m){
    
    display_count++;
    if(! ( (display_count % 200) == 0) ) return;
 
    if(ncurse)clear();//function in curses library  
   
    printf("<==========================================>\n");   	
        printf("        System Flags    \n");
	printf("CONTROLLER_RUN = "); printf(CONTROLLER_RUN ? "true\n" : "false\n");
	printf("SYSTEM_RUN = "); printf(SYSTEM_RUN ? "true\n" : "false\n");
	printf("\n\n");

        printf("        IMU DATA (degrees)    \n");
        printf("Last read success? %i \n", imu_data.succ_read);
        printf("phi:   %7.2f         phi dot:   %7.2f \n",imu_data.phi, imu_data.phi_dot);//, imu_data.phi, imu_data.phi_dot);
        printf("theta: %7.2f         theta dot: %7.2f\n",imu_data.theta, imu_data.theta_dot);
        printf("psi:   %7.2f         psi dot:   %7.2f    psi uncal:   %7.2f\n\n",imu_data.psi, imu_data.psi_dot, imu_data.psi_contin);
    
/*
   
	printf("        VICON DATA                                                          FILTERED VICON DATA   \n");
        printf("phi:      %10.2f       x: %10.2f                           phi:   %10.2f       x: %10.2f\n", vicon.phi, vicon.x, vicon_filt.phi, vicon_filt.x);
        printf("theta:    %10.2f       y: %10.2f                           theta: %10.2f       y: %10.2f\n",vicon.theta, vicon.y, vicon_filt.theta, vicon_filt.y);
        printf("psi:      %10.2f       z: %10.2f                           psi:   %10.2f       z: %10.2f\n\n",vicon.psi, vicon.z, vicon_filt.psi, vicon_filt.z);
      
    printf("        VICON VELOCITY                                                      FILTERED VICON VELOCITY  \n");
printf("phi_dot:  %10.2f       x_dot: %10.2f                  phi_dot:   %10.2f      x_dot: %10.2f\n", vicon_vel.phi, vicon_vel.x,  vicon_vel_filt.phi, vicon_vel_filt.x);
printf("theta_dot:%10.2f       y_dot: %10.2f                  theta_dot: %10.2f      y_dot: %10.2f\n",vicon_vel.theta, vicon_vel.y, vicon_vel_filt.theta, vicon_vel_filt.y);
printf("psi_dot:  %10.2f       z_dot: %10.2f                  psi_dot:   %10.2f      z_dot: %10.2f\n\n",vicon_vel.psi, vicon_vel.z ,vicon_vel_filt.psi, vicon_vel_filt.z);

    printf("        VICON ERRORS (meters)      \n"), joystick_thrust, U_trim.thrust;
        printf("x_prop:   %10.2f      y_prop:  %10.2f       z_prop:  %10.2f\n", vicon_error.x.prop, vicon_error.y.prop, vicon_error.z.prop);
        printf("x_deriv:  %10.2f      y_deriv: %10.2f       z_deriv: %10.2f\n", vicon_error.x.deriv, vicon_error.y.deriv, vicon_error.z.deriv);
        printf("x_integ:  %10.2f      y_integ: %10.2f       z_integ: %10.2f\n\n", vicon_error.x.integral, vicon_error.y.integral, vicon_error.z.integral);

    printf("        DESIRED ANGLES = f(vicon_error, gains)      \n");
        printf("phi:      %10.2f\n", desired_angles.phi);
        printf("theta:    %10.2f\n", desired_angles.theta);
        printf("psi:      %10.2f\n\n", desired_angles.psi);
        
*/  
      
     printf("        JOYSTICK DATA\n ");
	if(flight_mode < 11) printf("Flight Mode: OFF %i \n", flight_mode);
        printf("Last read success? %i  freq: %f \n", succ_read, 1/joystick.getDt());
        printf("phi: %.2f         theta: %.2f      psi: %.2f \n",  desired_angles.phi, desired_angles.theta, desired_angles.psi);
        printf("flight_mode: %i  joystick_thrust:  %i \n\n",flight_mode, joystick_thrust);

/*
    printf("        THRUST \n");
        printf("U_trim: %i, joystick: %i \n\n", U_trim.thrust, joystick_thrust);   
      

    printf("        IMU ERRORS = f(imu_data, desired_angles) (radians)      \n");
        printf("phi:   %7.2f         phi dot:   %7.2f \n",imu_error.phi, imu_error.phi_dot);//, imu_data.phi, imu_data.phi_dot);
        printf("theta: %7.2f         theta dot: %7.2f\n",imu_error.theta, imu_error.theta_dot);
        printf("psi:   %7.2f         psi dot:   %7.2f\n\n",imu_error.psi, imu_error.psi_dot);
        printf("psi:   %7.2f         psi dot:   %7.2f\n\n",imu_error.psi*180/3.14, imu_error.psi_dot*180/3.14);
        

     printf("        DESIRED ACCELERATION = f(imu_error, gains)      \n");
        printf("roll  (phi)  :      %f\n",   U.roll_acc);
        printf("pitch (theta):      %f\n",   U.pitch_acc);
        printf("yaw   (psi)  :      %f\n\n", U.yaw_acc);    
     //   printf("U.yaw_acc %3.3f  =  (gains.kp_psi %3.3f   * imu_error.psi %3.3f  )  +  (gains.kd_psi %3.3f  * imu_error.psi_dot %3.3f  ) \n", 
       //                 U.yaw_acc, gains.kp_psi, imu_error.psi, gains.kd_psi, imu_error.psi_dot);
     
    printf("          GAINS \n");
        printf("Theta: KP %10.2f KD: %10.2f \n", gains.kp_theta, gains.kd_theta);
        printf("Phi: KP %10.2f KD: %10.2f \n", gains.kp_phi, gains.kd_phi);
        printf("Psi: KP %10.2f KD: %10.2f \n\n", gains.kp_psi, gains.kd_psi);


	printf("        THRUST(0-255): Current mode is");
	printf(VICON_OR_JOY ? " VICON: Thrust = Calc_Thrust + Trim_Thrust\n" : " JOYSTICK: Thrust = joystick_thrust from Joystick\n");    
	printf("U.thrust: %i, joystick_thrust %i, U.trim %i  \n\n", U.thrust, joystick_thrust, U_trim.thrust);

*/


 
    printf("        FORCES (0-255)     \n");
        printf("motor_1: %i  ", *(motor_1.get_force()));
        printf("motor_2: %i  ", *(motor_2.get_force()));
        printf("motor_3: %i  ", *(motor_3.get_force()));
        printf("motor_4: %i \n\n", *(motor_4.get_force()));

/*
    printf("        TIME INFO     \n");
        printf("Controller timestep (s)       :     %7.4f\n", tv2float(times.delta));
        printf("Controller loop frequency (Hz):     %5.3f\n", 1/tv2float(times.delta));
        printf("Motor loop frequency (Hz)     :    %5.3f\n", 1/tv2float(time_m.delta));
       // printf("%lld.%.9ld", (long long)times.current.tv_sec, times.current.tv_nsec);
        printf("Time Date                     :    %s\n ", times.date_time);
*/


	printf("        Sonar: Distance (300 mm -5000 mm) and Frequency (Hz)     \n");
	printf("X+) succ_read: %i, freq: %f, distance: %i, repulsive_scale [0,1] %f \n", x_pos.succ_read, 1/sonar_x_pos.getDt(), sonar_x_pos.returnLastDistance(), repulsion_factor*UTILITY::dist2ScaleInv(sonar_x_pos.returnLastDistance(), minDist, maxDist));  
	printf("X-) succ_read: %i, freq: %f, distance: %i, repulsive_scale [0,1] %f \n", x_neg.succ_read, 1/sonar_x_neg.getDt(), sonar_x_neg.returnLastDistance(), repulsion_factor*UTILITY::dist2ScaleInv(sonar_x_neg.returnLastDistance(), minDist, maxDist));
	printf("Y+) succ_read: %i, freq: %f, distance: %i, repulsive_scale [0,1] %f \n", y_pos.succ_read, 1/sonar_y_pos.getDt(), sonar_y_pos.returnLastDistance(), repulsion_factor*UTILITY::dist2ScaleInv(sonar_y_pos.returnLastDistance(), minDist, maxDist));
	printf("Y-) succ_read: %i, freq: %f, distance: %i, repulsive_scale [0,1] %f  \n", y_neg.succ_read, 1/sonar_y_neg.getDt(),sonar_y_neg.returnLastDistance(), repulsion_factor*UTILITY::dist2ScaleInv(sonar_y_neg.returnLastDistance(), minDist, maxDist));
	if(ncurse)refresh();//refreshes shell console to output this text



}

//executes input from host computer on motors, controller gains, displays, and controller
void *command_input(void *thread_id){
   
    //printf("INSIDE COMMAND_INPUT");
    unsigned char command;
    string input;

    while(SYSTEM_RUN) {
	    printf("    please give input for command_input: ");
        //command = getchar(); 
        //getline(cin, input);
        command = getch();
        printf("input: %i \n", command);
        switch (command) {
            case '1':

            case '4':
                start_motors();
                break;
                
            case '5':
            case 'q':
            case 'Q':
                printf("Motor Stop!\n");
                stop_motors();
                CONTROLLER_RUN = false;//change to System Run!
                endwin();
                break;
                
            case ' ':
            case 'z':
            case 'Z':
            case '\n':
                controller_on_off(CONTROLLER_RUN);
                break;

// control of desired positions
            case 'w':
            case 'W':
	          printf("Increase X desired_positions\n");
              desired_positions.x = desired_positions.x + delta_position;
                break; 
    
            case 's':
            case 'S':
	          printf("Decrease X desired_positions\n");
              desired_positions.x = desired_positions.x - delta_position;
                break; 
            
            case 'd':
            case 'D':
              printf("Increase Y desired_positions\n");
              desired_positions.y = desired_positions.y + delta_position;
              break;

            case 'a':
            case 'A':
              printf("Decrease Y desired_positions\n");
              desired_positions.y = desired_positions.y - delta_position;
                break;

            case 'r':
            case 'R':
              printf("Decrease Z desired_positions\n");
              desired_positions.z = desired_positions.z - delta_position;
                break;

            case 'f':
            case 'F':
              printf("Increase Z desired_positions\n");
              desired_positions.z = desired_positions.z + delta_position;
                 break;

// control of desired positions

            case 'b':
            case 'B':
                display_on_off(DISPLAY_RUN);
               // system("clear");
               //if(ncurse)clear(); //function in curses library
                break;
                
            case 'c':
            case 'C':
                if((U_trim.thrust + delta_thrust) >=  max_thrust) {
                  printf("Maximum Thrust Reached: Cannot Increase Thrust\n");
                  U_trim.thrust = max_thrust;}
                else {U_trim.thrust += delta_thrust; 
                     printf("Increase Thrust: %i\n", U_trim.thrust);}
              break;
                
            case 'v':
            case 'V':
                if((U_trim.thrust-=delta_thrust) <= 0) {
                printf("Thrust is 0. Cannot decrease further\n");
                U_trim.thrust = 0;}
                else {printf("Decrease Thrust: %i\n", U_trim.thrust);}
              break;
               
            case 'i':
            case 'I':
              printf("Reset Desired_positions to initial values\n");
              desired_positions.x = init_positions.x;
              desired_positions.y = init_positions.y;
              desired_positions.z = init_positions.z;
              break;
            
            case 'j':
            case 'J':
                printf("Reset desired_positions to current position (raw vicon)\n");
                Vicon current_position;
                get_vicon_data(usb_xbee, current_position);
                desired_positions.x = current_position.x;
                desired_positions.y = current_position.y;
                desired_positions.z = current_position.z;
                break;
    //START GAINS         
            case 'k':
            case 'K':
                printf("Increase kp_phi and kp_theta\n");
                gains.kp_phi   = gains.kp_phi   + 0.1;
                gains.kp_theta = gains.kp_theta + 0.1;
                break;
            
            case 'm':
            case 'M':
                printf("Decrease kp_phi and kp_theta\n");
                if((gains.kp_phi <= 0) || (gains.kp_phi <= 0)) {printf("kp_phi/theta already 0"); break;}
                gains.kp_phi   = gains.kp_phi   - 0.1;
                gains.kp_theta = gains.kp_theta - 0.1;    
                break;
     
            case 'g':
            case 'G':
                printf("Increase kd_phi and kd_theta\n");
                gains.kd_phi   = gains.kd_phi   + 0.05;
                gains.kd_theta = gains.kd_theta + 0.05;
                break;
                
            case 'h':
            case 'H':
                printf("Decrease kd_phi and kd_theta\n");
                if((gains.kd_phi <= 0) || (gains.kd_phi <= 0)) {printf("kd_phi/theta already 0"); break;}
                gains.kd_phi   = gains.kd_phi   - 0.05;
                gains.kd_theta = gains.kd_theta - 0.05;
                break;
            
            case 'n':
            case 'N':
                printf("Decrease kp_psi\n");
                if(gains.kp_psi <= 0) {printf("kp_psi already 0"); break;}
                gains.kp_psi   = gains.kp_psi   - 0.1;
                break;

            case 'o':
            case 'O':
                printf("Increase kp_psi\n");
                gains.kp_psi   = gains.kp_psi   + 0.1;
                break;
            
            case 'p':
            case 'P':
                printf("Decrease kd_psi\n");
                if(gains.kd_psi <= 0) {printf("kd_psi already 0"); break;} 
                gains.kd_psi   = gains.kd_psi   - 0.05;
                break;

            case 't':
            case 'T':
                printf("Increase kd_psi\n");
                gains.kd_psi   = gains.kd_psi   + 0.05;
                break;
      //END GAINS      
            case 'L':
                printf("Start/Stop Logging\n");
                LOG_DATA = ~LOG_DATA;
                break;
             
            case '`':
                if(ncurse)clear();
                if(ncurse)refresh();
                break;
                
              //used after j: s,w,r,f,v
              
              //  printf("Control Landing: Not Implemented\n");
              //t_landing = t;
              //CTRL_LANDING = true;

        }

    }
    pthread_exit(NULL);
}
void configure_threads(void){
    //pthread_t - is an abstract datatype that is used as a handle to reference the thread
     //threads[0] = control_stabilizer
     //threads[1] = motor_signal
     //threads[2] = command_input

    pthread_t threads[NUM_THREADS];
    //Special Attribute for starting thread
    pthread_attr_t attr;
    //sched_param is a structure that maintains the scheduling parameters
        //sched_param.sched_priority  - an integer value, the higher the value the higher a thread's proiority for scheduling
    struct sched_param param;
    int fifo_max_prio, fifo_min_prio;
    
    // system("clear");
     //printf("INSIDE CONFIGURE_THREADS\n");
     	//if(ncurse)refresh();
     // Set thread attributes: FIFO scheduling, Joinable
     // the sched_param.sched_priorirty is an int that must be in [min,max] for a certain schedule policy, in this case, SCHED_FIFO
     pthread_attr_init(&attr);
     pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
     pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
     fifo_max_prio = sched_get_priority_max(SCHED_FIFO);
     fifo_min_prio = sched_get_priority_min(SCHED_FIFO); 

     	// create threads
     
     printf("=> creating control_stabilizer thread\n");
     //	if(ncurse)refresh();
     // Higher priority for filter
     param.sched_priority = fifo_max_prio;
     pthread_attr_setschedparam(&attr, &param);
     pthread_create(&threads[0], &attr, control_stabilizer, (void *) 0);
     usleep(onesecond);
  
     printf("=> creating motor_signal thread\n");
     	//if(ncurse)refresh();
     // Medium priority for motor_signal
     param.sched_priority = (fifo_max_prio+fifo_min_prio)/2;
     pthread_attr_setschedparam(&attr, &param);
     pthread_create(&threads[1], &attr, motor_signal, (void *) 1);
//	usleep(onesecond);
//	printf("final usleep done");

     printf("=> creating command_input thread\n");
	//if(ncurse)refresh();
     // Lower priority for commmand input
     param.sched_priority = (fifo_max_prio+fifo_min_prio)/2;
     pthread_attr_setschedparam(&attr, &param);
     pthread_create(&threads[2], &attr, command_input, (void *) 2);
    

     // Wait for all threads to complete
       for (int i = 0; i < NUM_THREADS; i++)  {  
	   //calling join will block this main thread until every thread exits
         pthread_join(threads[i], NULL);
        }

     printf("EXITING CONFIGURE_THREADS\n");
     close(usb_imu_ivn);
    
     pthread_attr_destroy(&attr);
}
void init(void)
{
    //ncurses
    if(ncurse) initscr();

   //finds bias in gyro and checks for out of range imu values. returns 1 if bad imu values.
   printf("finding gyro bias...\n");  
   //if(ncurse)refresh();
   if(imu.calibrate() == 1) ;
   else CONTROLLER_RUN = false;
   //if(ncurse)refresh();
	
printf("joystick.check_start_thrust() %i \n", joystick.check_start_thrust());
/*
   if(!VICON_OR_JOY)
   {
	if(joystick.check_start_thrust() > 0) CONTROLLER_RUN = true;
	else				      CONTROLLER_RUN = false;

	printf("CONTROLLER_RUN: %i\n\n", CONTROLLER_RUN);
	//if(ncurse)refresh();
   }
*/
   set_gains(gains);
   set_initial_times(times);
   set_initial_times(times_display);
}

int main(void)
{
	//intialize desired angles, gains, U_trim, & open port ot xbee and imu
	init();
	
	//if(ncurse)clear();
	printf("in main \n"); //if(ncurse)refresh();
	
	start_motors();

	configure_threads();

	usleep(onesecond*2);
	//if(ncurse)clear();
	if(ncurse) endwin();

	return 0;

}

