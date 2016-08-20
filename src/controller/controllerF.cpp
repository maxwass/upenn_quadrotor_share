#include "controller.h"
//g++ controller.cpp imu.cpp sonar.cpp xbee1.cpp vicon.cpp motor.cpp logger.cpp utility.cpp -I ../include -lpthread -lncurses -lboost_system -std=c++11
//initialize process-scoped data-structures
Times times;
Times time_m;
Times times_display;

Positions init_positions = {0.0};
Positions desired_positions = {0.0};

Gains gains= {0.0};
Control_command U_trim = {0};
double Ct=0.013257116418667*10;
double d=0.169;
bool MAGN = false;
bool SYSTEM_RUN = true;
bool CONTROLLER_RUN = true;
bool ESTOP = true;
bool XConfig = true;
bool AUTO_HEIGHT = false;
bool VISION_CONTROL = false;
bool DISPLAY_RUN =false;
bool LOG_DATA = true;
bool END_CALIBRATION = false;
bool AUTO_TAKEOFF = false;
bool ALREADY_TOOK_OFF = false;

int i2cHandle, usb_imu_ivn, usb_xbee;
uint16_t display_count=0;
bool SONAR_BUBBLE_SIDES = false;
bool SONAR_BUBBLE_DOWN =  false;
bool SONAR_BUBBLE_UP   =  false;
bool REPULSE_OR_DISTANCE = false;
float desiredDist = 1000;

int repulsion_factor = 10;
int repulsion_factor_down = 20;
int repulsion_factor_up = 20;
int minDist = 500;
int maxDist = 1500;

int minDistDown = 300;
int maxDistDown = 1000;
int minDistUp = 300;
int maxDistUp = 1000;

float upLim = 1;
float lowLim = 0;
float totalTime = 2.00;
		
float display_thrust  = 0.0;
int port;

std::string log_filename = "file.log";
logger logger(log_filename, 0, LOG_DATA);

//create our motor objects - accesible from all threads
//on this particular quadrotor the Body Frame is defined as follows:
//+x is between motors (0x2b , 0x29), +y is between (0x29 , 0x2c), 
//and by the right hand rule, +z is down.

/*
#define hardwareI2C 1
#if hardwareI2C ==1
        std::string PATH2MOTOR = "/dev/i2c-1";
#else
        std::string PATH2MOTOR = "/dev/i2c-10";
#endif
*/
std::string PATH2MOTOR = "/dev/i2c-10";

motor motor_1(PATH2MOTOR,1, 0x2c); //motor motor_1(1, 0x29); //0x2f
motor motor_2(PATH2MOTOR,2, 0x29);//motor motor_2(2, 0x2c); //0x2d
motor motor_3(PATH2MOTOR,3, 0x2b);//motor motor_3(3, 0x2a); //0x30
motor motor_4(PATH2MOTOR,4, 0x2a);//motor motor_4(4, 0x2b); //0x2e

SonarTest x_pos ={0}, x_neg ={0}, y_pos ={0}, y_neg ={0}, down = {0}, up = {0}; 

PIDController altitudeObj(gains.kp_altitude, gains.kd_altitude, gains.ki_altitude, 0.00, 0.00, 5.00);
Sonar sonar_x_pos("/dev/ttyUSB4", minDist, maxDist);
Sonar sonar_x_neg("/dev/ttyUSB3", minDist, maxDist);
Sonar sonar_y_pos("/dev/ttyUSB0", minDist, maxDist);
Sonar sonar_y_neg("/dev/ttyUSB2", minDist, maxDist);
Sonar sonar_down("/dev/ttyUSB6",  minDistDown, maxDistDown);
Sonar sonar_up("/dev/ttyUSB5",    minDistDown, maxDistDown);

int VICON_OR_JOY = 0; // 1 = VICON, 0 = JOYSTICK, 2 = both

float timeLimitImu, timeLimitXbee, timeLimitM1, timeLimitM2, timeLimitM3, timeLimitM4 = 0.0;
int incLimXbee, incLimImu = 0;
int iterUntilImuRead = 13;
int iterUntilXbeeRead = 3;
//Angles desired_angles = {0};

Timer timerController;
Timer timerXbee;
Timer timerImu;
Timer timerTakeOff;


#define ncurse 0
#if ncurse == 1
	#define printf(...) printw(__VA_ARGS__)
#endif

double force_m1, force_m2, force_m3, force_m4 = 0.0;

//optical flow parameters 
bool new_vo = false;
double xd, yd, zd;
double pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, vel_x_filt, vel_y_filt, vel_z_filt, pos_x_, pos_y_, pos_z_, vel_x_, vel_y_;
double pos_x_temp = 0.;
double pos_y_temp = 0.; 
double pos_z_temp = 0.;
double vel_x_filt_ = 0.;
double vel_x_filt__ = 0.;
double vel_y_filt_ = 0.;
double vel_y_filt__ = 0.;
double vel_z_filt_ = 0.;
double vel_z_filt__ = 0.;
double vel_x_vo, vel_y_vo, pos_x_vo, pos_y_vo, vision_dt = 0.00;
double x_vel_d = 0.;
double y_vel_d = 0.;

State imu_data;
int8_t new_xbee_data_global = -1;
uint8_t joystick_thrust , flight_mode = 0;
Angles desired_angles = {0};
float xbee_dt = 0;

float gyro_phi_sync[12] = {0.0};
float gyro_theta_sync[12] = {0.0};

int nominal_thrust = 0;

void *control_stabilizer(void *thread_id)
{

	printf("in control stabilizer \n");
	

	Distances sonar_distances;
	Distances repulsive_forces;
	//weights is used for filter: current, one value ago, 2 values ago
	//Weights weights = {.7,.2,.1};

	//for error calculations PID: stores the actual errors, not gains
	State_Error vicon_error = {0.0};
	State_Error trans_error = {0.0};
	Vicon desired_velocity = {0.0}; 

	State imu_error = {0};
	Control_command U = {0};
	//Angles desired_angles = {0};
	
	//uint8_t joystick_thrust , flight_mode = 0;
	int succ_read;
	int new_xbee_data, new_imu_data, new_motor_write, new_sonar_data_x_pos, new_sonar_data_x_neg, new_sonar_data_y_pos, new_sonar_data_y_neg, new_sonar_data_down, new_sonar_data_up = 0;
	times.delta.tv_nsec = delta_time; //500000;
	bool first_loop = true;
	
	ros::NodeHandle nh;
	ros::Subscriber optical_flow_subscriber = nh.subscribe("/optical_flow", 1, optical_flow_callback);
	ros::Subscriber imu_sub = nh.subscribe<controller::ImuData>("imu/imu_data",1,imuCallback);
	ros::Subscriber xbee_sub = nh.subscribe<controller::XbeeData>("xbee/xbee_cmds",1,xbeeCallback); 
	//double force_m1, force_m2, force_m3, force_m4 = 0.0;

while(SYSTEM_RUN) 
  {
	
	new_xbee_data, new_imu_data, new_motor_write, new_sonar_data_x_pos, new_sonar_data_x_neg, new_sonar_data_y_pos, new_sonar_data_y_neg, new_sonar_data_down, new_sonar_data_up = -1;
	
	
	if(!ros::ok()) (SYSTEM_RUN = false);
	
	//usleep(100);
	//calc new times and delta
	float dt = timerController.update();  


/*
	new_sonar_data_x_pos = sonar_x_pos.get_sonar_data(x_pos);
	new_sonar_data_x_neg = sonar_x_neg.get_sonar_data(x_neg);
	new_sonar_data_y_pos = sonar_y_pos.get_sonar_data(y_pos);
	new_sonar_data_y_neg = sonar_y_neg.get_sonar_data(y_neg);
	new_sonar_data_down  = sonar_down.get_sonar_data(down);
	new_sonar_data_up    = sonar_up.get_sonar_data(up);
*/

	//reads input from imu (in degrees), distributes into fields of imu_data
	//get_imu_data returns 1 if read successful,< 0 if not. Reuse old values if not successful
	new_xbee_data = -2;
	new_xbee_data_global = -2;

	timeLimitImu += dt;
	timeLimitXbee+= dt;
	timeLimitM1  += dt;
	timeLimitM2  += dt;
	timeLimitM3  += dt;
	timeLimitM4  += dt;
	
	//printf("IMU TIMER: %f  ", timeLimitImu);
	imu_data.succ_read = false;
	new_xbee_data_global = false;
	new_vo = false;
	
	ros::spinOnce(); // get messages from published topics 
	new_imu_data = imu_data.succ_read;
	new_xbee_data = new_xbee_data_global;

	//optical flow calculations
	if(new_vo){

		vel_x = vel_x_vo/100.;
		vel_y = vel_y_vo/100.;
		
		//printf("vel_x_vo: %f, vel_y_vo: %f, vel_x: %f, vel_y: %f \n", vel_x_vo, vel_y_vo, vel_x, vel_y);

		vel_x -= gyro_theta_sync[2]/140000.;
		vel_y -= -gyro_phi_sync[2]/140000.;
		
		//printf("vel_x -= gyro_theta_sync[2]/140000: %f, gyro_theta_sync[2]: %f \n", vel_x, gyro_theta_sync[2]);
		//printf("vel_y -= gyro_phi_sync[2]/140000: %f, gyro_phi_sync[2]: %f \n", vel_y, gyro_phi_sync[2]);

		
		vel_x *= (altitudeObj.getFiltValue() + 0.05)/1.5;
	        vel_y *= (altitudeObj.getFiltValue() + 0.05)/1.5;
		
		//printf("vel_x *= (altitudeObj.getFiltValue() + 0.05)/1.5: %f, altitudeObj.getValue(): %f \n", vel_x, altitudeObj.getValue());
		//printf("vel_y *= (altitudeObj.getFiltValue() + 0.05)/1.5: %f, altitudeObj.getValue(): %f \n", vel_y, altitudeObj.getValue()); 


		//if(abs(vel_x) < 0.000001) printf("		VEL_X IS ZERO^^  \n\n\n\n");
		//if(abs(vel_y) < 0.000001) printf("           VEL_Y IS ZERO^^  \n\n\n\n");

		if(abs(vel_x) > 0.02 || abs(vel_y) > 0.02){
			vel_x = vel_x_;
			vel_y = vel_y_;
		}
		
		//printf("Absolute value range check: vel_x: %f, vel_y: %f \n\n", vel_x, vel_y);

		vel_x_ = vel_x;
		vel_y_ = vel_y;
		
		pos_x += vel_x;
		pos_y += vel_y;
	/*
		    result = pfilt_vo_velx.filter(vel_x, vel_x_filt);
	            if (result < 0){
	                printf("Filter Failed!!\n");
	            }
	            
	            result = pfilt_vo_vely.filter(vel_y, vel_y_filt);
	            if (result < 0){
	                printf("Filter Failed!!\n");
	            }
	*/
		trans_error.x.prop = xd - pos_x;
		trans_error.y.prop = yd - pos_y;
		trans_error.x.deriv = x_vel_d - vel_x;
		trans_error.y.deriv = y_vel_d - vel_y;
		
		trans_error.x.integral += trans_error.x.prop;
		trans_error.y.integral += trans_error.y.prop;


		//printf("BEGIN: vel_x_vo: %f \n", vel_x_vo);

		//new_vo = false;
			
		}


	//can switch between psi estimators
	if(!MAGN) imu_data.psi = imu_data.psi_gyro_integration;
	else 	  imu_data.psi = imu_data.psi_magn_continuous_calibrated;

	if (VICON_OR_JOY == 1)
	{
		printf("in vicon part");
		//new_xbee_data = xbee.get_xbee_data();

		/*if(new_xbee_data)
		{
			//calculate error from vicon
			xbee.error_vicon(vicon_error, desired_velocity, desired_positions);	
			//calculate desired attitude (phi theta phi) in desired_angles
			desired_angles_calc(desired_angles, vicon_error, gains);
		}*/
	}	
	else
	{
		Angles old_desired_angles = desired_angles;
	
		if (VISION_CONTROL == true)
		{
			desired_angles_calc(desired_angles,trans_error,gains);
		}
		
		if(AUTO_TAKEOFF == true)
		{
	
			float t = timerTakeOff.timeSinceStart();
			float altDesired = ((upLim - lowLim)/totalTime)*t - lowLim;
			if(altDesired > 1) 
			{
					AUTO_HEIGHT = true;
					VISION_CONTROL = true;
					AUTO_TAKEOFF = false;
					altDesired = 1;
			}
			altitudeObj.setDesiredValue(altDesired);
		
			if(altitudeObj.getFiltValue() > 0.500) VISION_CONTROL = true;

		}


		if(new_xbee_data < 0) ; //printf("joystick not ready to read: old data");
		//check flight mode
		    if(ESTOP)
		    {
			    if(flight_mode < 11.0) 
			    {
				CONTROLLER_RUN     = false;
				SONAR_BUBBLE_SIDES = false;
				SONAR_BUBBLE_DOWN  = false;
				SONAR_BUBBLE_UP    = false;   
				sonar_x_pos.distanceControlOff();
	
				VISION_CONTROL = false;
				AUTO_HEIGHT = false;
			
				altitudeObj.autoControlOff();
			    }
				//16 only controller
			    else if (flight_mode > 15.5 && flight_mode < 16.5) 
			    {
				SONAR_BUBBLE_SIDES = false;
                                SONAR_BUBBLE_DOWN = false;
                                SONAR_BUBBLE_UP   =  false;
				//if(sonar_x_pos.isDistControlOn()) sonar_x_pos.distanceControlOff();
				VISION_CONTROL = false;
				AUTO_HEIGHT = false;
				altitudeObj.autoControlOff();
		            }
			    //17 Bubble
			    else if(flight_mode > 16.5 && flight_mode < 17.5)
			    {
				SONAR_BUBBLE_SIDES = true;
                                //SONAR_BUBBLE_DOWN = true;
                                //SONAR_BUBBLE_UP   =  true;
				//if(!sonar_x_pos.isDistControlOn()) sonar_x_pos.distanceControlOn(desiredDist);
				VISION_CONTROL = false;
				AUTO_HEIGHT = false;
				altitudeObj.autoControlOff();
			    }
			    

			    //18 AUTO_HEIGHT
			    else if (flight_mode > 17.5 && flight_mode < 18.5)
			    {
				SONAR_BUBBLE_SIDES = false;
				SONAR_BUBBLE_DOWN =  false;
				SONAR_BUBBLE_UP   =  false;
				//if(sonar_x_pos.isDistControlOn()) sonar_x_pos.distanceControlOff();
			
				if(ALREADY_TOOK_OFF == false) AUTO_TAKEOFF = true;
				timerTakeOff.reset();
				AUTO_HEIGHT = true;
				VISION_CONTROL = true;
			        ALREADY_TOOK_OFF = true;
			     }

			    //19 BOTH
			    else if (flight_mode > 18.5 && flight_mode < 19.5)
			    {
				SONAR_BUBBLE_SIDES = true;
				//SONAR_BUBBLE_DOWN =  true;
				//SONAR_BUBBLE_UP   =  true;
				if(!sonar_x_pos.isDistControlOn()) sonar_x_pos.distanceControlOn(desiredDist);			
				if(!altitudeObj.isAutoControlOn()) altitudeObj.autoControlOn(imu_data.altitude_calibrated);
				//AUTO_HEIGHT = true;
			    }


		    }
	}

	//calculate error from imu (in radians) between desired and measured state
	State imu_error = error_imu(imu_data, new_imu_data, desired_angles);
	//calculate thrust and desired acceleration
	U = thrust(imu_error,vicon_error, U_trim, joystick_thrust, gains);
	//calculate the forces of each motor and change force on motor objects and send via i2c 
	distribute_forces(U, Ct, d, force_m1, force_m2, force_m3, force_m4);
	

	//new_motor_write = send_forces(force_m1, force_m2, force_m3, force_m4);

	//printf("new_vo: %i, new_imu_data: %i \n", new_vo, new_imu_data);
	//printf("new_vo || new_imu_data: %i \n",  new_vo || new_imu_data); 

	//printf("BOOOOL: %i\n", (LOG_DATA && ( (new_xbee_data>0) || (new_imu_data>0) || (new_sonar_data_1>0) || (new_sonar_data_2>0) || (new_sonar_data_3>0) || (new_sonar_data_4>0) ) ) );
	if (LOG_DATA && (new_vo || new_imu_data))
//( (new_xbee_data>0) || (new_imu_data>0) || (new_sonar_data_x_pos>0) || (new_sonar_data_x_neg>0) || (new_sonar_data_y_pos>0) || (new_sonar_data_y_neg>0) ) || (new_sonar_data_down>0) || (new_vo > 0) )   
	{
		//printf("Before Log Data  -  vel_x_vo: %f \n", vel_x_vo);
		log_data(timerController.timeSinceStart(), sonar_distances, dt,  vicon_error, imu_data, imu_error, desired_angles, U.thrust);       
		//printf("After  Log Data  -  vel_x_vo: %f \n\n", vel_x_vo);
	}
	if (DISPLAY_RUN) 
	{ 
		display_info(sonar_distances, new_xbee_data,  imu_data,  imu_error, vicon_error, U, desired_angles,joystick_thrust, flight_mode, times_display, time_m); 
	}



	//new_vo = false;
	//new_imu_data = false;
	//new_xbee_data = false;
	

 }
    printf("EXIT CONTROL_STABILIZER\n");
    pthread_exit(NULL);

}

void desired_angles_calc(Angles& desired_angles, const State_Error& error, const Gains& gains)
{

    desired_angles.psi     = 0;
    desired_angles.phi     =  gains.kp_y*error.y.prop + gains.kd_y*error.y.deriv + gains.ki_y*error.y.integral; //5.y_pos_sensor - 5*y_neg_sensor
    desired_angles.theta   = -gains.kp_x*error.x.prop - gains.kd_x*error.x.deriv - gains.ki_x*error.x.integral; //5.x_pos_sensor - 5*x_neg_sensor

    if(desired_angles.phi > 10.00) desired_angles.phi = 10.00;
    else if(desired_angles.phi < -10.00) desired_angles.phi = -10.00;

    if(desired_angles.theta > 10.00) desired_angles.theta = 10.00;
    else if(desired_angles.theta < -10.00) desired_angles.theta = -10.00;

}

void stop_motors(void){
    printf("Stopping Motors ...\n");
    motor_1.send_motor_data(0, false);
    motor_2.send_motor_data(0, false);
    motor_3.send_motor_data(0, false);
    motor_4.send_motor_data(0, false);
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
State error_imu(const State& imu_data, const int new_imu_data, const Angles& desired_angles){
    //calculate error in RADIANS
    //  xxx_d is xxx_desired.  imu outputs  degrees, we convert to radians with factor PI/180
    
    State error;
    error.phi    =  (desired_angles.phi   - imu_data.phi)      * PI/180;
    error.theta  =  (desired_angles.theta - imu_data.theta)    * PI/180;

    if (SONAR_BUBBLE_SIDES)
    {
	if(REPULSE_OR_DISTANCE)
	{
		 error.phi   += repulsion_factor*(UTILITY::dist2ScaleInv(sonar_y_neg.returnLastDistance(), minDist, maxDist) - UTILITY::dist2ScaleInv(sonar_y_pos.returnLastDistance(), minDist, maxDist))  * PI/180;
         	error.theta += repulsion_factor*(UTILITY::dist2ScaleInv(sonar_x_pos.returnLastDistance(), minDist, maxDist) - UTILITY::dist2ScaleInv(sonar_x_neg.returnLastDistance(), minDist, maxDist))  * PI/180;
	}
	else
	{
		 //error.phi   += sonar_y_pos.distance_control_output(sonar_y_pos.returnLastDistance()); 		       
		 //error.theta += sonar_x_pos.distance_control_output(sonar_x_pos.returnLastDistance());

	}
	
    }



    error.psi       =     (-imu_data.psi  + desired_angles.psi) * PI/180;
    error.phi_dot   =                           (-imu_data.phi_dot) * PI/180;
    error.theta_dot =                         (-imu_data.theta_dot) * PI/180;
    error.psi_dot   =                           (-imu_data.psi_dot) * PI/180;
   
    if(new_imu_data) 
    {
	altitudeObj.update(imu_data.altitude_calibrated, 0.00625);
       	////altitudeObj.getFiltValue();

     }



    error.altitude_calibrated     =     altitudeObj.getFiltPropError();
    error.altitude_deriv 	  =     altitudeObj.getFiltDerivError();
    error.altitude_integral 	  =     altitudeObj.getIntegralError();


    return error;
}
Control_command thrust(const State& imu_error, const State_Error& vicon_error, const Control_command& U_trim, const int joystick_thrust, const Gains& gains){
    //calculate thrust and acceleration
    Control_command U = {0};
    
    if(VICON_OR_JOY == 1) //VICON
    {
    	 int calc_thrust = (int) (-(gains.kp_z * vicon_error.z.prop)  -  (gains.kd_z * vicon_error.z.deriv) - (gains.ki_z * vicon_error.z.integral));
    	 U.thrust        =  calc_thrust + U_trim.thrust; 
    }
    else if(VICON_OR_JOY != 1 && AUTO_HEIGHT == false) //JOYSTICK
    {
	U.thrust        = 4 * joystick_thrust  + U_trim.thrust;
	nominal_thrust = U.thrust;
	/*xd = pos_x; 
	  yd = pos_y;*/
    }
    else if(AUTO_TAKEOFF == true && AUTO_HEIGHT == true) //AUTOTAKEOFF
    {
	//used saved trim value
        //U.thrust = 380 + gains.kp_altitude*altitudeObj.getFiltPropError() + gains.kd_altitude*altitudeObj.getFiltDerivError();

	//used trim value from joystick
	U.thrust = nominal_thrust + gains.kp_altitude*altitudeObj.getFiltPropError() + gains.kd_altitude*altitudeObj.getFiltDerivError();

    }
    else if(AUTO_HEIGHT == true) //AUTOHEIGHT
    {
        U.thrust = 380 + gains.kp_altitude*altitudeObj.getFiltPropError() + gains.kd_altitude*altitudeObj.getFiltDerivError();
        
    }
    
    if(VISION_CONTROL == false) {
      xd = pos_x;
      yd = pos_y;
    }
    if(SONAR_BUBBLE_DOWN)
    {
	U.thrust += repulsion_factor_down*UTILITY::dist2ScaleInv(sonar_down.returnLastDistance(), minDistDown, maxDistDown);
    }

    if(SONAR_BUBBLE_UP)
    {
        U.thrust -= repulsion_factor_up*UTILITY::dist2ScaleInv(sonar_up.returnLastDistance(), minDistUp, maxDistUp); 
    }

    
 	
    U.roll_acc  =  (gains.kp_phi   * imu_error.phi  )  +  (gains.kd_phi   * imu_error.phi_dot  )  + U_trim.roll_acc;
    U.pitch_acc =  (gains.kp_theta * imu_error.theta)  +  (gains.kd_theta * imu_error.theta_dot)  + U_trim.pitch_acc;
    U.yaw_acc   =  (gains.kp_psi   * imu_error.psi  )  +  (gains.kd_psi   * imu_error.psi_dot  )  + U_trim.yaw_acc;
   /*printf("U.yaw_acc %3.3f  =  (gains.kp_psi %3.3f   * imu_error.psi %3.3f  )  +  (gains.kd_psi %3.3f  * imu_error.psi_dot %3.3f  ) \n", 
                        U.yaw_acc, gains.kp_psi, imu_error.psi, gains.kd_psi, imu_error.psi_dot);
   if(ncurse)refresh(); 
   */
   if (U.thrust <= 0) U.thrust = 0;
   display_thrust = U.thrust;
    return U;
}

void distribute_forces(const Control_command& U, double Ct, double d, double &force_m1, double &force_m2, double &force_m3, double &force_m4){
      //calculate forces from thrusts and accelerations

  //printf("thrust: %i, U.trim: %i, yaw_acc: %f, pitch_acc: %f, Ct: %f, d: %f \n", U.thrust,U_trim.thrust, U.yaw_acc, U.pitch_acc, Ct, d);
  if(!XConfig)
    {//this is Plus Configuration
	
        force_m1 = (U.thrust/4 - (U.yaw_acc /(4*Ct)) + (U.pitch_acc /  (2*d)));
        force_m2 = (U.thrust/4 + (U.yaw_acc /(4*Ct)) - (U.roll_acc  /  (2*d)));
        force_m3 = (U.thrust/4 - (U.yaw_acc /(4*Ct)) - (U.pitch_acc /  (2*d)));
        force_m4 = (U.thrust/4 + (U.yaw_acc /(4*Ct)) + (U.roll_acc  /  (2*d)));
        
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
        
        force_m1 = x_param * (force_1 + force_2)-(U.yaw_acc/(4*Ct));
        force_m2 = x_param * (force_2 + force_3)+(U.yaw_acc/(4*Ct));
        force_m3 = x_param * (force_3 + force_4)-(U.yaw_acc/(4*Ct));
        force_m4 = x_param * (force_4 + force_1)+(U.yaw_acc/(4*Ct));
       
    }
	//printf("Dist: force_m1: %f, force_m2: %f, force_m3: %f, force_m4: %f \n", force_m1, force_m2, force_m3, force_m4);
}

int send_forces(double force_m1, double force_m2, double force_m3, double force_m4)
{

	int m1 = 0;
	int m2 = 0;
	int m3 = 0;
	int m4 = 0;
	
	if(timeLimitM1 > 0.004)
	{
		m1 = motor_1.send_motor_data( round(force_m1), CONTROLLER_RUN);
		if(m1 > 0) timeLimitM1 = 0.000;
	}
	if(timeLimitM2 > 0.004)
        {
		m2 = motor_2.send_motor_data( round(force_m2), CONTROLLER_RUN);
		if(m2 > 0) timeLimitM2 = 0.000;
        }
	if(timeLimitM3 > 0.004)
        {
		
		m3 = motor_3.send_motor_data( round(force_m3), CONTROLLER_RUN);
                if(m3 > 0) timeLimitM3 = 0.000;
        }
	if(timeLimitM4 > 0.004)
        {
		m4 = motor_4.send_motor_data( round(force_m4), CONTROLLER_RUN);
                if(m4 > 0) timeLimitM4 = 0.000;
        }


	//printf("Send: force_m1: %i, force_m2: %i, force_m3: %i, force_m4: %i \n", motor_1.get_force(), motor_2.get_force(), motor_3.get_force(), motor_4.get_force());


	if (m1 || m2 || m3 || m4) return 1;
	else			  return 0;
}
void *motor_signal(void *thread_id){

   //printf("INSIDE MOTOR_SIGNAL\n");
   set_initial_times(time_m);

      while(SYSTEM_RUN){
        //printf("in motor loop");
        motor_1.send_motor_data_ns( round(force_m1), CONTROLLER_RUN);
        motor_2.send_motor_data_ns( round(force_m2), CONTROLLER_RUN);
        motor_3.send_motor_data_ns( round(force_m3), CONTROLLER_RUN);
        motor_4.send_motor_data_ns( round(force_m4), CONTROLLER_RUN);

        //sets frequency of motor_signal
        usleep(1000);
        }

   printf("EXIT MOTOR_SIGNAL\n");

   pthread_exit(NULL);
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

void log_data(const float time, const Distances& sonar_distances, const float& dt, const State_Error& vicon_error, const State& imu_data, const State& imu_error, const Angles& , const int thrust){

	Data_log d;
	d.time = time;
	d.dt   = dt;

	if(VICON_OR_JOY > 0)
	{
	    /*d.vicon_data      = xbee.getLastVicon();
	    d.vicon_vel       = xbee.getLastViconVel();
	    d.vicon_data_filt = xbee.getLastFiltVicon();
	    d.vicon_vel_filt  = xbee.getLastViconVel();*/
	}

	d.vicon_error     = vicon_error;
	d.imu             = imu_data;
	d.imu_error       = imu_error;
	d.forces.motor_1  = motor_1.get_force();//round(force_m1);
	d.forces.motor_2  = motor_2.get_force();//round(force_m2);
	d.forces.motor_3  = motor_3.get_force();//round(force_m3);
	d.forces.motor_4  = motor_4.get_force();//round(force_m4);
	d.forces.m1_freq  = 1/motor_1.getDt();
	d.forces.m2_freq  = 1/motor_2.getDt();
	d.forces.m3_freq  = 1/motor_3.getDt();
	d.forces.m4_freq  = 1/motor_4.getDt();

	d.thrust	  = thrust;
	
	OpticalFlow o;
	o.pix_velocity_x =  vel_x;
	o.pix_velocity_y =  vel_y;
	o.pix_position_x =  pos_x;
	o.pix_position_y =  pos_y;
	o.dt 		 =  vision_dt;
	d.opticalflow = o;

	OpticalFlow o_vo;
        o_vo.pix_velocity_x =  vel_x_vo;
	//printf("IN Log Data: o_vo.pix_velocity_x: %f \n", o_vo.pix_velocity_x);
        o_vo.pix_velocity_y =  vel_y_vo;
        o_vo.pix_position_x =  pos_x_vo;
        o_vo.pix_position_y =  pos_y_vo;
        o_vo.dt             =  vision_dt;
      

//printf("pvx: %f, pvy: %f, px: %f, py: %f \n\n", o_vo.pix_velocity_x, o_vo.pix_velocity_y, o_vo.pix_position_x, o_vo.pix_position_y);

	d.opticalflow_vo = o_vo;


	d.desired_angles  = desired_angles;
	d.xbee_dt	  = xbee_dt;
	
	
	Distances sonarDist;
	sonarDist.x_pos = sonar_x_pos.returnLastDistance();
	sonarDist.x_neg = sonar_x_neg.returnLastDistance();
	sonarDist.y_pos = sonar_y_pos.returnLastDistance();
	sonarDist.y_neg = sonar_y_neg.returnLastDistance();
	sonarDist.down = sonar_down.returnLastDistance();
	sonarDist.up = sonar_up.returnLastDistance();
	//printf(" x_pos) %i, x_neg) %i, y_pos) %i y_neg) %i \n",sonarDist.x_pos, sonarDist.x_neg, sonarDist.y_pos, sonarDist.y_neg);
	d.sonar_distances = sonarDist;

	RepForces scales;
	scales.x_pos = repulsion_factor*UTILITY::dist2ScaleInv(sonar_x_pos.returnLastDistance(), minDist, maxDist);
	scales.x_neg = repulsion_factor*UTILITY::dist2ScaleInv(sonar_x_neg.returnLastDistance(), minDist, maxDist);
	scales.y_pos = repulsion_factor*UTILITY::dist2ScaleInv(sonar_y_pos.returnLastDistance(), minDist, maxDist);
	scales.y_neg = repulsion_factor*UTILITY::dist2ScaleInv(sonar_y_neg.returnLastDistance(), minDist, maxDist);
	scales.down  = repulsion_factor_down*UTILITY::dist2ScaleInv(sonar_down.returnLastDistance(), minDistDown, maxDistDown);
	scales.up    = repulsion_factor_up  *UTILITY::dist2ScaleInv(sonar_up.returnLastDistance(), minDistUp, maxDistUp);
	d.scales = scales;

	Errors altitude_errors;
	altitude_errors.prop     = altitudeObj.getPropError();
	altitude_errors.deriv    = altitudeObj.getDerivError();
	altitude_errors.integral = altitudeObj.getIntegralError();
	d.altitude 	         = altitudeObj.getValue();
	d.altitude_desired       = altitudeObj.getDesiredValue();
	d.altitude_errors        = altitude_errors;

	Errors filter_altitude_errors;
	filter_altitude_errors.prop     	  = altitudeObj.getFiltPropError();
	filter_altitude_errors.deriv    	  = altitudeObj.getFiltDerivError();
	filter_altitude_errors.integral 	  = altitudeObj.getFiltIntegralError();
	d.filter_altitude 	          = altitudeObj.getFiltValue();
	d.filter_altitude_errors          = filter_altitude_errors;


	Errors pos_x_sonar_errors;
	sonar_x_pos.getErrors(pos_x_sonar_errors);
	d.pos_x_sonar_errors   = pos_x_sonar_errors; 
	d.desired_pos_x_dist   = pos_x_sonar_errors.desired_value;
	
	Errors neg_x_sonar_errors;
	sonar_x_neg.getErrors(neg_x_sonar_errors);
	d.neg_x_sonar_errors   = neg_x_sonar_errors; 
	d.desired_neg_x_dist   = neg_x_sonar_errors.desired_value;
	
	Errors pos_y_sonar_errors;
	sonar_y_pos.getErrors(pos_y_sonar_errors);
	d.pos_y_sonar_errors   = pos_y_sonar_errors; 
	d.desired_pos_y_dist   = pos_y_sonar_errors.desired_value;

	Errors neg_y_sonar_errors;
	sonar_y_neg.getErrors(neg_y_sonar_errors);
	d.neg_y_sonar_errors   = neg_y_sonar_errors; 
	d.desired_neg_y_dist   = neg_y_sonar_errors.desired_value;
	
	Errors up_sonar_errors;
	sonar_up.getErrors(up_sonar_errors);
	d.up_sonar_errors   = up_sonar_errors; 
	d.desired_up_dist   = up_sonar_errors.desired_value;
	
	Errors down_sonar_errors;
	sonar_down.getErrors(up_sonar_errors);
	d.down_sonar_errors   = down_sonar_errors; 
	d.desired_down_dist   = down_sonar_errors.desired_value;
	
    	logger.log(d);
}

void display_info(const Distances& sonar_distances, const int succ_read,  const State& imu_data,  const State& imu_error, const State_Error& vicon_error, const Control_command& U, const Angles& desired_angles,const int  joystick_thrust, const int  flight_mode, const Times& times, const Times& time_m){
    
    display_count++;
    if(! ( (display_count % 10000) == 0) ) return;
 
    if(ncurse)clear();//function in curses library  
   
    printf("<==========================================>\n");   	
        printf("        System Flags    \n");
	printf("CONTROLLER_RUN = "); printf(CONTROLLER_RUN ? "true\n" : "false\n");
	printf("SYSTEM_RUN = "); printf(SYSTEM_RUN ? "true\n" : "false\n");
	printf("BUBBLE_SIDES = "); printf(SONAR_BUBBLE_SIDES ? "true\n" : "false\n");
	printf("BUBBLE_UP = "); printf(SONAR_BUBBLE_UP ? "true\n" : "false\n");
	printf("BUBBLE_DOWN = "); printf(SONAR_BUBBLE_DOWN ? "true\n" : "false\n");
	printf("AUTO_HEIGHT = "); printf(AUTO_HEIGHT ? "true\n" : "false\n");
	printf("VISION_CONTROL = "); printf(VISION_CONTROL ? "true\n" : "false\n");
	printf("gains.kp_x = %f      gains.kp_y = %f     gains.kd_x = %f      gains.kd_y = %f \n", gains.kp_x, gains.kp_y, gains.kd_x, gains.kd_y);
	printf("\n\n");

	
	printf("AUTO_TAKE_OFF = ");  printf(AUTO_TAKEOFF ? "true\n" : "false\n");
	printf("ALREADY_TOOK_OFF = ");  printf(ALREADY_TOOK_OFF ? "true\n" : "false\n");
	printf("Timer: %7.2f \n", timerTakeOff.timeSinceStart());
	//printf("Nominal Thrust: %f /n", nominal_thrust);
	printf("Altitude: %7.4f, Desired Height: %f \n", altitudeObj.getFiltValue(), altitudeObj.getDesiredValue());
	printf("LowerLim: %f, UpperLim: %f, totalTime: %f \n\n", lowLim, upLim, totalTime);

	printf("Imu: timeSinceStart %f, timeSinceLastUpdate: %f \n\n", timerImu.timeSinceStart(), timerImu.getTimeSinceUpdate());
        printf("        IMU DATA (degrees)    \n");
        if(MAGN) printf("(Using Magnetometer Psi)\n");
	else 	 printf("Using Gyro Integration Psi\n");
	printf("Read Frequency: %f \n", 1/imu_data.dt);
        printf("phi:   %7.2f         phi dot:   %7.2f \n",imu_data.phi, imu_data.phi_dot);//, imu_data.phi, imu_data.phi_dot);
        printf("theta: %7.2f         theta dot: %7.2f\n",imu_data.theta, imu_data.theta_dot);
        printf("psi_gyro:   %7.2f    psi dot:     %7.2f \n\n",imu_data.psi_gyro_integration, imu_data.psi_dot);
    	printf("psi_magn:   %7.2f    psi uncal:   %7.2f\n\n",  imu_data.psi_magn_continuous_calibrated, imu_data.psi_magn_continuous);
	printf("altitude:   %7.4f, altitudeFilt:   %7.2f,  desired_altitude:  %7.2f  max: %7.2f, min: %7.2f,  altitude_deriv:   %7.2f,  prop_error:   %7.2f,  deriv_error:   %7.6f, integral_error:   %7.2f\n\n", altitudeObj.getValue(), altitudeObj.getFiltValue(), altitudeObj.getDesiredValue(),altitudeObj.getMaxValue(),  altitudeObj.getMinValue(), altitudeObj.getValueDeriv(), altitudeObj.getFiltPropError(), altitudeObj.getFiltDerivError(), altitudeObj.getIntegralError());
	printf("Altitude: gains.kp = %f gains.kd = %f\n", gains.kp_altitude, gains.kd_altitude);
	
/*
   
	printf("        VICON DATA                                                          FILTERED VICON DATA   \n");
        printf("phi:      %10.2f       x: %10.2f                           phi:   %10.2f       x: %10.2f\n", vicon.phi, vicon.x, vicon_filt.phi, vicon_filt.x);
        printf("theta:    %10.2f       y: %10.2f                           theta: %10.2f       y: %10.2f\n",vicon.theta, vicon.y, vicon_filt.theta, vicon_filt.y);
        printf("psi:      %10.2f       z: %10.2f                           psi:   %10.2f       z: %10.2f\n\n",vicon.psi, vicon.z, vicon_filt.psi, vicon_filt.z);
      
    printf("        VICON VELOCITY                                                      FILTERED VICON VELOCITY  \n");
g_da
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
        printf("Last read (sec)? %f  freq: %f \n", 0.0, 1.0/xbee_dt);
        printf("phi: %.2f         theta: %.2f      psi: %.2f \n",  desired_angles.phi, desired_angles.theta, desired_angles.psi);
        printf("flight_mode: %i  joystick_thrust:  %i \n",flight_mode, joystick_thrust);
	printf("Bubble Mode: "); 
	if ( (flight_mode > 17.0) && (flight_mode < 19.0)) printf("ON \n\n"); 
	else 						   printf("OFF \n\n");
	

    printf("        THRUST \n");
        printf("U_trim: %i, joystick: %i \n\n", U_trim.thrust, joystick_thrust);   
 /*     

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
 */
/*    
    printf("          GAINS \n");
        printf("Theta: KP %10.2f KD: %10.2f \n", gains.kp_theta, gains.kd_theta);
        printf("Phi: KP %10.2f KD: %10.2f \n", gains.kp_phi, gains.kd_phi);
        printf("Psi: KP %10.2f KD: %10.2f \n\n", gains.kp_psi, gains.kd_psi);


	printf("        THRUST(0-255): Current mode is");
	printf(VICON_OR_JOY ? " VICON: Thrust = Calc_Thrust + Trim_Thrust\n" : " JOYSTICK: Thrust = joystick_thrust from Joystick\n");    
	printf("U.thrust: %i, joystick_thrust %i, U.trim %i  \n\n", U.thrust, joystick_thrust, U_trim.thrust);

*/


    printf("        FORCES (0-255)     \n");
        printf("motor_1: %i, freq: %f  ",    motor_1.get_force(),  1/motor_1.getDt() );
        printf("motor_2: %i, freq: %f  ",    motor_2.get_force(),  1/motor_2.getDt() );
        printf("motor_3: %i, freq: %f  ",    motor_3.get_force(),  1/motor_3.getDt() );
        printf("motor_4: %i, freq: %f \n\n", motor_4.get_force(), 1/motor_4.getDt() );


    printf("        TIME INFO     \n");
        printf("Controller timestep (s)       :     %7.4f\n", timerController.getDt());
        printf("Controller loop frequency (Hz):     %5.3f\n", 1/timerController.getDt());


    printf("        CAMERA DATA: OPTICAL FLOW NEW     \n");
	printf("Pixel/s:  Velocity X %7.4f, Velocity Y: %7.4f \n",  vel_x_vo,  vel_y_vo);   
	printf("M/s:  Velocity X %7.4f, Velocity Y: %7.4f \n\n", vel_x,  vel_y);
       	printf("Pos X %7.4f, Pos Y: %7.4f \n", pos_x, pos_y);
	printf("Xd: %7.4f , Yd: %7.4f \n",xd,yd);
	printf("Frequency: %7.4f \n", 1/vision_dt);



/*

	float x_pos_repulsion = repulsion_factor*UTILITY::dist2ScaleInv(sonar_x_pos.returnLastDistance(), minDist, maxDist);
	float x_neg_repulsion = repulsion_factor*UTILITY::dist2ScaleInv(sonar_x_neg.returnLastDistance(), minDist, maxDist);
	float y_pos_repulsion = repulsion_factor*UTILITY::dist2ScaleInv(sonar_y_pos.returnLastDistance(), minDist, maxDist);
	float y_neg_repulsion = repulsion_factor*UTILITY::dist2ScaleInv(sonar_y_neg.returnLastDistance(), minDist, maxDist);
	float down_repulsion = repulsion_factor*UTILITY::dist2ScaleInv(sonar_down.returnLastDistance(), minDist, maxDist);
	float up_repulsion = repulsion_factor*UTILITY::dist2ScaleInv(sonar_up.returnLastDistance(), minDist, maxDist);	

	Errors sonar_errors;
        sonar_x_pos.getErrors(sonar_errors);
        float desired_pos_x_dist   = sonar_errors.desired_value;

	printf("\n        Sonar:      \n");
	printf("SONAR_BUBBLE_SIDES = "); printf(SONAR_BUBBLE_SIDES ? "ON\n" : "OFF\n");
	if(sonar_x_pos.getStatus()) printf("X+: ON, DistControlOn?: %i, freq: %f,  distance: %i, desired distance: %f, DistControlOuput: %f, PError: %f, DError: %f, repulsive_scale [0,%i] %f \n", sonar_x_pos.isDistControlOn(),1/sonar_x_pos.getDt(), sonar_x_pos.returnLastDistance(), sonar_x_pos.getDesiredDistance(), sonar_x_pos.getControlOutput(), sonar_errors.prop, sonar_errors.deriv, repulsion_factor, x_pos_repulsion); 
	else printf("X+: OFF, last_read (sec): %f, distance: %i, repulsive_scale [0,%i] \n", sonar_x_pos.timeSinceLastRead(),sonar_x_pos.returnLastDistance(), x_pos_repulsion);

		if(sonar_x_neg.getStatus()) printf("X-: ON, DistControlOn?: %i, freq: %f,  distance: %i, desired distance: %f, DistControlOuput: %f, PError: %f, DError: %f, repulsive_scale [0,%i] %f \n", sonar_x_neg.isDistControlOn(),1/sonar_x_neg.getDt(), sonar_x_neg.returnLastDistance(), sonar_x_neg.getDesiredDistance(), sonar_x_neg.getControlOutput(), sonar_x_neg.getPropError(), sonar_x_neg.getDerivError(), repulsion_factor, x_neg_repulsion); 
	else printf("X-: OFF, last_read (sec): %f, distance: %i, repulsive_scale [0,%i] \n", sonar_x_neg.timeSinceLastRead(),sonar_x_neg.returnLastDistance(), x_neg_repulsion);

	if(sonar_y_pos.getStatus()) printf("Y+: ON, DistControlOn?: %i, freq: %f,  distance: %i, desired distance: %f, DistControlOuput: %f, PError: %f, DError: %f, repulsive_scale [0,%i] %f \n", sonar_y_pos.isDistControlOn(),1/sonar_y_pos.getDt(), sonar_y_pos.returnLastDistance(), sonar_y_pos.getDesiredDistance(), sonar_y_pos.getControlOutput(), sonar_y_pos.getPropError(), sonar_y_pos.getDesiredDistance(), repulsion_factor, y_pos_repulsion); 
	else printf("Y-: OFF, last_read (sec): %f, distance: %i, repulsive_scale [0,%i] \n", sonar_y_pos.timeSinceLastRead(),sonar_y_pos.returnLastDistance(), y_pos_repulsion);

	if(sonar_y_neg.getStatus()) printf("Y-: ON, DistControlOn?: %i, freq: %f,  distance: %i, desired distance: %f, DistControlOuput: %f, PError: %f, DError: %f, repulsive_scale [0,%i] %f \n", sonar_y_neg.isDistControlOn(),1/sonar_y_neg.getDt(), sonar_y_neg.returnLastDistance(), sonar_y_neg.getDesiredDistance(), sonar_y_neg.getControlOutput(), sonar_y_neg.getPropError(), sonar_y_neg.getDerivError(), repulsion_factor, y_neg_repulsion); 
	else printf("Y-: OFF, last_read (sec): %f, distance: %i, repulsive_scale [0,%i] \n", sonar_x_neg.timeSinceLastRead(),sonar_x_neg.returnLastDistance(), x_neg_repulsion);

	if(sonar_up.getStatus()) printf("UP: ON, DistControlOn?: %i, freq: %f,  distance: %i, desired distance: %f, DistControlOuput: %f, PError: %f, DError: %f, repulsive_scale [0,%i] %f \n", sonar_up.isDistControlOn(),1/sonar_up.getDt(), sonar_up.returnLastDistance(), sonar_up.getDesiredDistance(), sonar_up.getControlOutput(), sonar_up.getPropError(), sonar_up.getDesiredDistance(), repulsion_factor, up_repulsion);
        else printf("UP: OFF, last_read (sec): %f, distance: %i, repulsive_scale [0,%i] \n", sonar_up.timeSinceLastRead(),sonar_up.returnLastDistance(), up_repulsion);

	if(sonar_down.getStatus()) printf("DOWN:ON,DistControlOn?: %i, freq: %f,  distance: %i, desired distance: %f, DistControlOuput: %f, PError: %f, DError: %f, repulsive_scale [0,%i] %f \n", sonar_down.isDistControlOn(),1/sonar_down.getDt(), sonar_down.returnLastDistance(), sonar_down.getDesiredDistance(), sonar_down.getControlOutput(), sonar_down.getPropError(), sonar_down.getDesiredDistance(), repulsion_factor, down_repulsion);
        else printf("DOWN: OFF, last_read (sec): %f, distance: %i, repulsive_scale [0,%i] \n", sonar_down.timeSinceLastRead(), sonar_down.returnLastDistance(), down_repulsion);





 	
	if(sonar_x_neg.getStatus()) printf("X-: ON, freq: %f,  distance: %i, repulsive_scale [0,%i] %f \n", x_neg.succ_read, 1/sonar_x_neg.getDt(),  sonar_x_neg.returnLastDistance(), repulsion_factor,x_neg_repulsion);
        else printf("X-: OFF, last_read (sec): %f, distance: %i, repulsive_scale [0,%i] %f  \n", sonar_x_neg.timeSinceLastRead(),sonar_x_neg.returnLastDistance(), x_neg_repulsion);

	if(sonar_y_pos.getStatus()) printf("Y+: ON, freq: %f,  distance: %i, repulsive_scale [0,%i] %f \n", y_pos.succ_read, 1/sonar_y_pos.getDt(),  sonar_y_pos.returnLastDistance(), repulsion_factor, y_pos_repulsion);
        else printf("Y+: OFF, last_read (sec): %f, distance: %i, repulsive_scale [0,%i] %f \n", sonar_x_pos.timeSinceLastRead(),  sonar_y_pos.returnLastDistance(), y_pos_repulsion);

	 if(sonar_y_neg.getStatus()) printf("Y-: ON, freq: %f,  distance: %i, repulsive_scale [0,%i] %f \n", y_neg.succ_read, 1/sonar_y_neg.getDt(),  sonar_y_neg.returnLastDistance(), repulsion_factor,y_neg_repulsion);
        else printf("Y-: OFF, last_read (sec): %f distance: %i, repulsive_scale [0,%i] %f \n\n", sonar_y_neg.timeSinceLastRead(), sonar_y_neg.returnLastDistance(), y_neg_repulsion);

        printf("SONAR_BUBBLE_DOWN = "); printf(SONAR_BUBBLE_DOWN ? "ON,  " : "OFF,  ");
        printf("SONAR_BUBBLE_UP = "); printf(SONAR_BUBBLE_UP ? "ON\n" : "OFF\n");
	if(sonar_down.getStatus()) printf("DOWN: ON, freq: %f,  distance: %i, repulsive_scale [0,%i] %f \n", down.succ_read, 1/sonar_down.getDt(),  sonar_down.returnLastDistance(), repulsion_factor, down_repulsion);
        else printf("DOWN: OFF, last_read (sec): %f, distance: %i, repulsive_scale [0,%i] %f \n", sonar_down.timeSinceLastRead(), sonar_down.returnLastDistance(), down_repulsion );

	if(sonar_up.getStatus()) printf("UP:   ON, freq: %f,  distance: %i, repulsive_scale [0,%i] %f \n", up.succ_read, 1/sonar_up.getDt(),  sonar_up.returnLastDistance(), repulsion_factor,up_repulsion);
        else printf("UP: OFF, last_read (sec): %f, distance: %i, repulsive_scale [0,%i] %f \n", sonar_up.timeSinceLastRead(), sonar_up.returnLastDistance(), up_repulsion);

*/
	if(ncurse)refresh();//refreshes shell console to output this text


}

//executes input from host computer on motors, controller gains, displays, and controller
void *command_input(void *thread_id){
   
    //printf("INSIDE COMMAND_INPUT");
    unsigned char command;
    string input;


   int kk = 0;

    while(SYSTEM_RUN) {
	    //printf(" give input for command_input: ");
        //command = getchar(); 
        //getline(cin, input);
        command = getch();
	
	kk++;
	//if((kk%1000000) == 0) printf("input: %i \n", command);
        switch (command) {
            case '1':
		printf("AUTO_HEIGHT SWITCH! \n");
		AUTO_HEIGHT = !AUTO_HEIGHT;
		break;
	   /* case '2':
		printf("AUTO_TAKEOFF! \n");
		if(ALREADY_TOOK_OFF == true) break;
		if(AUTO_TAKEOFF == true) break;
		if(CONTROLLER_RUN == false) break;
		if(SYSTEM_RUN == false) break;
		if(AUTO_TAKEOFF == false) 
		{
			timerTakeOff.reset();
			AUTO_TAKEOFF = true;		
			AUTO_HEIGHT = true;
			ALREADY_TOOK_OFF = true;
		}
		break;
	  */
            case '4':
               // start_motors();
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
              //printf("Decrease Y desired_positions\n");
              //desired_positions.y = desired_positions.y - delta_position;
                calibration();

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
	      /* display_on_off(DISPLAY_RUN);
               // system("clear");
               //if(ncurse)clear(); //function in curses library*/
	      
	      gains.kp_x += 0.5;
	      gains.kp_y += 0.5;
	
	      printf("Increase  p gain vision: %f \n" , gains.kp_x);
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
	      /*if((U_trim.thrust-=delta_thrust) <= 0) {
                printf("Thrust is 0. Cannot decrease further\n");
                U_trim.thrust = 0;}
                else {printf("Decrease Thrust: %i\n", U_trim.thrust);}*/
	      gains.kp_x -= 0.5;
	      gains.kp_y -= 0.5;
	
	      printf("Decrease  p gain vision: %f \n" , gains.kp_x);
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
                gains.kp_altitude   += 1.0;
                break;
            
            case 'm':
            case 'M':
	      /*                printf("Decrease kp_phi and kp_theta\n");
                if((gains.kp_phi <= 0) || (gains.kp_phi <= 0)) {printf("kp_phi/theta already 0"); break;}
                gains.kp_phi   = gains.kp_phi   - 0.1;
                gains.kp_theta = gains.kp_theta - 0.1;*/
	      gains.kd_altitude += 1.0;
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
                //if((gains.kd_phi <= 0) || (gains.kd_phi <= 0)) {printf("kd_phi/theta already 0"); break;}
               // gains.kd_phi   = gains.kd_phi   - 0.05;
                //gains.kd_theta = gains.kd_theta - 0.05;
		if(gains.kp_altitude <= 0) break;
		else gains.kp_altitude -= 1.0;
                break;
            
            case 'n':
            case 'N':
	      /* printf("Decrease d\n");
                if(gains.kp_psi <= 0) {printf("kp_psi already 0"); break;}
                gains.kp_psi   = gains.kp_psi   - 0.1;
                break;*/

		if(gains.kd_altitude <= 0) break;
		else gains.kd_altitude -= 1.0;
		break;
            

	      //gains.kd_x -= 50.0;
	      //gains.kd_y = gains.kd_x;
	
	      printf("Decrease  d gain vision: %f \n" , gains.kd_x);
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
	if(ncurse)refresh();
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

	printf("=> creating command_input thread\n");
	if(ncurse)refresh();
	// Lower priority for commmand input
	param.sched_priority = (fifo_max_prio+fifo_min_prio)/2;
	pthread_attr_setschedparam(&attr, &param);
	pthread_create(&threads[2], &attr, command_input, (void *) 2);


	// Wait for all threads to complete
	for (int i = 0; i < NUM_THREADS; i++)  
	{  
	//calling join will block this main thread until every thread exits
	pthread_join(threads[i], NULL);
	}

	printf("EXITING CONFIGURE_THREADS\n");
	if(ncurse)refresh(); 
	close(usb_imu_ivn);

	pthread_attr_destroy(&attr);
}
void init(void)
{
	usleep(onesecond*2);
	//ncurses
	if(ncurse) initscr();
	if(ncurse)refresh();

	//printf("xbee.check_start_thrust() %i \n", xbee.check_start_thrust());

	sonar_x_pos.distanceControlOn(desiredDist);
	sonar_x_neg.distanceControlOn(desiredDist);
	sonar_y_pos.distanceControlOn(desiredDist);
	sonar_y_neg.distanceControlOn(desiredDist);
	sonar_down.distanceControlOn(desiredDist);
	sonar_up.distanceControlOn(desiredDist);
		
	set_gains(gains);
	altitudeObj.setGains(gains.kp_altitude, gains.kd_altitude,gains.ki_altitude);
	altitudeObj.setDesiredValue(0.00);
	set_initial_times(times);
	set_initial_times(times_display);
}

void optical_flow_callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	
        vel_x_vo = (float) (-msg->data[1]);
        vel_y_vo = (float) (msg->data[0]);
        pos_x_vo = (float) (-msg->data[3]);
        pos_y_vo = (float) (msg->data[2]);
	vision_dt =  (float) (msg->data[4]);       

        new_vo = true;

        return;
}

void imuCallback(const controller::ImuData::ConstPtr& imuMsg){
	/*if(imuMsg->succ_read < 0) {
		KILL_MOTORS = true; 
		return;
	}*/
	imu_data.theta = imuMsg->theta;
	imu_data.phi = imuMsg->phi;
	
	imu_data.psi = imuMsg->psi;
	imu_data.psi_magn_continuous_calibrated = imuMsg -> psi_magn_continuous_calibrated;
	imu_data.psi_gyro_integration = imuMsg -> psi_gyro_integration;
	imu_data.theta_dot = imuMsg->theta_dot;
	imu_data.phi_dot = imuMsg -> phi_dot;
	imu_data.psi_dot = imuMsg -> psi_dot;
	imu_data.dt = imuMsg -> dt;
	
	imu_data.altitude_calibrated = imuMsg -> altitude_calibrated;

	imu_data.succ_read = true;
	
	for(int i=0;i<12-1;i++)
	    {
            	gyro_phi_sync[11-i] = gyro_phi_sync[10-i];
            	gyro_theta_sync[11-i] = gyro_theta_sync[10-i];
            }
            
            
            gyro_phi_sync[0] =   imu_data.phi_dot;
            gyro_theta_sync[0] = imu_data.theta_dot;

	return;
	//new_imu_data.psi_gyro_integration = imuMsg ->psi_gyro_integration;
	//printf("imu theta: %f \n",new_imu_data.theta);
	
}

void xbeeCallback(const controller::XbeeData::ConstPtr& xbeeMsg){
	desired_angles.phi = xbeeMsg->joy_des_angles[0];
	desired_angles.theta = xbeeMsg->joy_des_angles[1];
	desired_angles.psi = xbeeMsg->joy_des_angles[2];
	joystick_thrust = xbeeMsg->joy_thrust;
	flight_mode = xbeeMsg->flight_mode;
	new_xbee_data_global = xbeeMsg -> succ;
	xbee_dt = xbeeMsg -> dt;
	return;
}

/*float LowPassFilter(float input, float input_old, float input_old_old, float a, float b, float c)
{
    float output = a * input + b * input_old + c * input_old_old;
    //printf("out: %e %e %e %e %e %e %e\n", a, b, c, input, input_old, input_old_old, output);
    return output;
    }*/

void calibration(void)
{
END_CALIBRATION = true;
}
int main(int argc, char** argv)
{
	if(ncurse)refresh();
        ros::init(argc, argv, "controller");
        //ros::NodeHandle nh;
        //if(ncurse)clear();

        configure_threads();
        usleep(onesecond*2);

        //if(ncurse)clear();
        if(ncurse) endwin();

        return 0;

}

