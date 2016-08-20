#include "logger.h"
//need sudo to run exec

// Member functions definitions including constructor
logger::logger(string filename, int cycles_until_log, bool LOG_DATA)
{
	//when queue gets larget than this, will flush all
	max_queue_length = cycles_until_log; 
    	
	time_t rawtime;
    	struct tm * timeinfo;
	char buffer[80];
	time (&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(buffer,80,"%d-%m-%Y__%I_%M_%S.log",timeinfo);
	first_loop = true;
	
    	const char *filename_p = buffer;// filename.c_str();
	
	printf("opening file for logging: %s\n",filename_p);//.c_str());

	if (LOG_DATA) (this->myfile).open(filename_p, ios::out | ios::app);

    std::cout.fixed;
}

void logger::log(Data_log d)
{
    //push data onto queue
    q.push(d);
    //when queue gets longer than max length write all to file
    if (q.size() > max_queue_length) this->write_to_file();
}

void logger::write_to_file( void )
{
    //write all data in queue to file    
    while(q.size() > 0)
        {
            //get front of queue and remove it
            Data_log d_temp = q.front();
            q.pop();
        
            //unwrap elements and write to file
            this->unwrap(d_temp);
       }
}

void logger::unwrap(Data_log d){
   
   if(first_loop)
	{
	//time
		myfile <<  "time" << ", " <<"controller_loop_dt" <<  ", ";

	//imu
		myfile <<  "theta" <<  ", " << "phi" << ", " << "psi_gyro_integ" << ", " <<  "psi_magn_continuous_calibrated" << ", " << "theta_dot_cal" <<  ", " << "phi_dot_cal" << ", "<< "psi_dot_cal" << ", " << "dt_imu" << ", " << "numPsiRot"<< ", " ;
	//motor
	

	myfile << "altitude" << ", ";

	myfile << "desired_altitude" << ", " << "altitude_prop_error" << ", " << "altitude_deriv_error" << ", " << "altitude_integral_error"<< ", ";

	myfile << "filter_altitude" << ", ";

	myfile << "filter_altitude_prop_error" << ", " << "filter_altitude_deriv_error" << ", " << "filter_altitude_integral_error"<< ", ";

		myfile <<  "total_thrust"  <<  ", ";
		myfile <<  "m1" <<  ", " << "m2" <<  ", " << "m3" <<  ", " << "m4" <<  ", ";        
		myfile << "m1_freq" <<  ", " << "m2_freq" <<  ", " << "m3_freq" <<  ", " << "m4_freq" <<  ", ";
	//OpticalFlow
		

		myfile <<  "pix_velocity_x"  <<  ", " <<  "pix_velocity_y" << ", " << "pix_position_x" << ", " << "pix_position_y" << ", " << "vision_dt" << ", ";

		myfile <<  "pix_velocity_x_vo"  <<  ", " <<  "pix_velocity_y_vo" << ", " << "pix_position_x_vo" << ", " << "pix_position_y_vo" << ", " << "vision_dt_vo" << ", ";

	//desired angles
		myfile <<  "theta_des" <<  ", " << "phi_des" <<  ", " << "psi_des" <<  ", ";

	//xbee
		myfile << "xbee_dt" << ", ";
	
	//SonarTest
		// myfile <<  "num_fds_n1"  << ", " <<  "num_fds_0" << ", " <<  "num_fds_1" << ", " << "num_fds_p" << ", " <<  "succ_read" << ", " <<  "distance" << ", " <<  "index" << ", " <<  "foundFirstByte" << ", " <<  "byte_read";

/*
	//Sonar Distances
		myfile <<  "dist_x_pos"  << ", " <<  "dist_x_neg" << ", " <<  "dist_y_pos" << ", " << "dist_y_neg" <<  ", " <<  "dist_down" << ", " << "dist_up" << ", ";
		
	//Sonar Repulsive forces
//		 myfile <<  "scale_x_pos"  << ", " <<  "scale_x_neg" << ", " <<  "scale_y_pos" << ", " << "scale_y_neg" << ", " <<  "scale_down" << ", " << "scale_up" << ", ";


		myfile <<   "desired_pos_x_dist" << ", " <<  "pos_x_prop_error" << ", " <<  "pos_x_deriv_error" << ", " <<  "pos_x_integral_error" << ", " ;

		myfile <<   "desired_neg_x_dist" << ", " <<  "neg_x_prop_error" << ", " <<  "neg_x_deriv_error" << ", " <<  "neg_x_integral_error" << ", " ;

		myfile <<   "desired_pos_y_dist" << ", " <<  "pos_y_prop_error" << ", " <<  "pos_y_deriv_error" << ", " <<  "pos_y_integral_error" << ", " ;

		myfile <<   "desired_neg_y_dist" << ", " <<  "neg_y_prop_error" << ", " <<  "neg_y_deriv_error" << ", " <<  "neg_y_integral_error" << ", " ;

		myfile <<   "desired_up_dist" << ", " <<  "up_prop_error" << ", " <<  "up_deriv_error" << ", " <<  "up_integral_error" ;

*/
		 myfile << "\n";
	}
   this->format(d.time);		 myfile << ", ";
   this->format(d.dt*100000);            myfile << ", ";    
   // this->format(d.vicon_data);      myfile << ", ";
   // this->format(d.vicon_data_filt); myfile << ", "; 
   // this->format(d.vicon_vel);       myfile << ", "; 
   // this->format(d.vicon_vel_filt);  myfile << ", "; 
   // this->format(d.vicon_error);     myfile << ", ";
   //this->format(d.read_error);	     myfile << ", ";
   this->format(d.imu);            myfile << ", ";
   this->format(d.altitude);	    myfile << ", ";
   this->format(d.altitude_desired);myfile << ", ";
   this->format(d.altitude_errors); myfile << ", ";
   this->format(d.filter_altitude); myfile << ", ";
   this->format(d.filter_altitude_errors); myfile << ", ";
   

   // this->format(d.imu_error);       myfile << ", ";
   this->format(d.thrust);		myfile << ", ";
   this->format(d.forces);          myfile << ", ";
   this->format(d.opticalflow);     myfile << ", ";
   //printf("In B4 Log Data: -  vel-x_vo: %f \n", d.opticalflow_vo.pix_velocity_x);
   this->format(d.opticalflow_vo);   myfile << ", ";

   //printf("In AF Log Data: -  vel-x_vo: %f \n", d.opticalflow_vo.pix_velocity_x);
   
   this->format(d.desired_angles);    myfile << ", "; 
  
   this->format(d.xbee_dt*100000); //         myfile << ", ";
   
/*
   this->format(d.sonar_distances);	myfile << ", ";
  // this->format(d.scales);    	      myfile << ", ";
 
   this->format(d.desired_pos_x_dist);    myfile << ", ";
   this->format(d.pos_x_sonar_errors);        myfile << ", ";
 
   this->format(d.desired_neg_x_dist);    myfile << ", ";
   this->format(d.neg_x_sonar_errors);        myfile << ", ";

   this->format(d.desired_pos_y_dist);    myfile << ", ";
   this->format(d.pos_y_sonar_errors);        myfile << ", ";
   
   this->format(d.desired_up_dist);    myfile << ", ";
   this->format(d.up_sonar_errors);        myfile << ", ";

   this->format(d.desired_down_dist);    myfile << ", ";
   this->format(d.down_sonar_errors);        myfile << ", ";
*/



   myfile << "\n";
   first_loop = false;
}

template <typename num>
string logger::num2str(num f)
{

std::ostringstream ss;
ss <<std::fixed << std::setprecision(10)<< f;
std::string s(ss.str());

return s;
}
//time is outputted in micro_seconds
void logger::format(Times t){
    myfile << num2str(1000*((double) t.delta.tv_sec + (t.delta.tv_nsec / 1000000000.0)));//(t.date_time) << ", " << num2str(1000*((double) t.delta.tv_sec + (t.delta.tv_nsec / 1000000000.0)));    
}
void logger::format(float f){
    myfile << num2str(f);// num2str(100000*f);
}
void logger::format(Vicon v){
    myfile <<  num2str(v.x) <<  ", " << num2str(v.y) << ", " << num2str(v.z) << ", " << num2str(v.theta) <<  ", " << num2str(v.phi) << ", " << num2str(v.psi);
}
void logger::format(State_Error se){
    format(se.x);     myfile << ", ";
    format(se.y);     myfile << ", ";
    format(se.z);     myfile << ", ";
    format(se.theta); myfile << ", ";
    format(se.phi);   myfile << ", ";
    format(se.psi);  //comma added in main function
}
void logger::format(Errors e){
    myfile <<  num2str(e.prop) <<  ", " << num2str(e.deriv) << ", " << num2str(e.integral);
}
void logger::format(int read_error){
    myfile <<  num2str(read_error);
}
void logger::format(State imu){
    myfile <<  num2str(imu.theta) <<  ", " << num2str(imu.phi) << ", " << num2str(imu.psi_gyro_integration) << ", "  << num2str(imu.psi_magn_continuous_calibrated) << ", "<<num2str(imu.theta_dot) <<  ", " << num2str(imu.phi_dot) << ", "<< num2str(imu.psi_dot) << ", " << num2str(100000*imu.dt) << ", " <<  num2str(imu.numPsiRot);

}
void logger::format(Motor_forces mf){
    myfile << num2str(mf.motor_1) << ", " << num2str(mf.motor_2) << ", " << num2str(mf.motor_3) << ", " << num2str(mf.motor_4) << ", " << num2str(mf.m1_freq) << ", " << num2str(mf.m2_freq) << ", " << num2str(mf.m3_freq) << ", " << num2str(mf.m4_freq);
}
void logger::format(SonarTest s){
	myfile <<  num2str(s.num_fds_n1) << ", " << num2str(s.num_fds_0) << ", " << num2str(s.num_fds_1) << ", " << num2str(s.num_fds_p) << ", " << num2str(s.succ_read) << ", " <<num2str(s.distance) <<  ", " << num2str(s.index) <<  ", " << num2str(s.foundFirstByte) <<  ", " << num2str((int)s.lastByte);
}
void logger::format(Angles a){
    myfile << num2str(a.theta) << ", " << num2str(a.phi) << ", " << num2str(a.psi);
}
void logger::format(Distances a){
    myfile << num2str(a.x_pos) << ", " << num2str(a.x_neg) << ", " << num2str(a.y_pos) << ", " << num2str(a.y_neg) << ", " << num2str(a.down) << ", " << num2str(a.up);
}
void logger::format(RepForces a){
    myfile << num2str(a.x_pos) << ", " << num2str(a.x_neg) << ", " << num2str(a.y_pos) << ", " << num2str(a.y_neg)<< ", " << num2str(a.down) << ", " << num2str(a.up);
}

void logger::format(OpticalFlow o){
    myfile << num2str(o.pix_velocity_x) << ", " << num2str(o.pix_velocity_y) << ", " << num2str(o.pix_position_x) << ", " << num2str(o.pix_position_y) << ", " << num2str(100000*o.dt);
}

/*
int main (void){
logger log("file.log",3);    
Data_log d;
Vicon v = {1.5};
State imu = {2.5};
d.vicon = v;
d.imu = imu;
for(int i = 0; i < 20; i++) logger.log(d);

}
*/

