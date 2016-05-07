#include "logger.h"
//need sudo to run exec

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
		myfile <<  "controller_loop_dt" <<  ", ";

	//imu
		myfile <<  "theta" <<  ", " << "phi" << ", " << "psi_magnet" << ", "  << "psi_gyro_integ" << ", "<< "theta_dot" <<  ", " << "phi_dot" << ", "<< "psi_dot" << ", " << "dt_imu" << ", " << "numPsiRot"<< ", ";
	//motor	
		myfile <<  "m1" <<  ", " << "m2" <<  ", " << "m3" <<  ", " << "m4" <<  ", ";        
	//desired angles
		myfile <<  "theta_des" <<  ", " << "phi_des" <<  ", " << "psi_des" <<  ", ";

	//SonarTest
		// myfile <<  "num_fds_n1"  << ", " <<  "num_fds_0" << ", " <<  "num_fds_1" << ", " << "num_fds_p" << ", " <<  "succ_read" << ", " <<  "distance" << ", " <<  "index" << ", " <<  "foundFirstByte" << ", " <<  "byte_read";

	//Sonar Distances
		myfile <<  "dist_x_pos"  << ", " <<  "dist_x_neg" << ", " <<  "dist_y_pos" << ", " << "dist_y_neg" <<  ", ";
		
	//Sonar Repulsive forces
		 myfile <<  "scale_x_pos"  << ", " <<  "scale_x_neg" << ", " <<  "scale_y_pos" << ", " << "scale_y_neg";

		 myfile << "\n";
	}
   this->format(d.dt);            myfile << ", ";    
   // this->format(d.vicon_data);      myfile << ", ";
   // this->format(d.vicon_data_filt); myfile << ", "; 
   // this->format(d.vicon_vel);       myfile << ", "; 
   // this->format(d.vicon_vel_filt);  myfile << ", "; 
   // this->format(d.vicon_error);     myfile << ", ";
   //this->format(d.read_error);	     myfile << ", ";
   this->format(d.imu);             myfile << ", ";
   // this->format(d.imu_error);       myfile << ", ";
   this->format(d.forces);          myfile << ", ";
   this->format(d.desired_angles);    myfile << ", "; 	
   this->format(d.sonar_distances);	myfile << ", ";
   this->format(d.scales);    myfile << ", ";
   myfile << "\n";
   first_loop = false;
}

template <typename num>
string logger::num2str(num f)
{

std::ostringstream ss;
ss <<std::fixed << std::setprecision(3)<< f;
std::string s(ss.str());

return s;
}
//time is outputted in micro_seconds
void logger::format(Times t){
    myfile << num2str(1000*((double) t.delta.tv_sec + (t.delta.tv_nsec / 1000000000.0)));//(t.date_time) << ", " << num2str(1000*((double) t.delta.tv_sec + (t.delta.tv_nsec / 1000000000.0)));    
}
void logger::format(float f){
    myfile <<  num2str(100000*f);
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
    myfile <<  num2str(imu.theta) <<  ", " << num2str(imu.phi) << ", " << num2str(imu.psi) << ", "  << num2str(imu.psi_gyro_integration) << ", "<<num2str(imu.theta_dot)<<  ", " <<num2str(imu.phi_dot)<< ", "<< num2str(imu.psi_dot) << ", " << num2str(100000*imu.dt)<< ", " << num2str(imu.numPsiRot);
}
void logger::format(Motor_forces mf){
    myfile << num2str(mf.motor_1) << ", " << num2str(mf.motor_2) << ", " << num2str(mf.motor_3) << ", " << num2str(mf.motor_4);
}
void logger::format(SonarTest s){
	myfile <<  num2str(s.num_fds_n1) << ", " << num2str(s.num_fds_0) << ", " << num2str(s.num_fds_1) << ", " << num2str(s.num_fds_p) << ", " << num2str(s.succ_read) << ", " <<num2str(s.distance) <<  ", " << num2str(s.index) <<  ", " << num2str(s.foundFirstByte) <<  ", " << num2str((int)s.lastByte);
}
void logger::format(Angles a){
    myfile << num2str(a.theta) << ", " << num2str(a.phi) << ", " << num2str(a.psi);
}
void logger::format(Distances a){
    myfile << num2str(a.x_pos) << ", " << num2str(a.x_neg) << ", " << num2str(a.y_pos) << ", " << num2str(a.y_neg);
}
void logger::format(RepForces a){
    myfile << num2str(a.x_pos) << ", " << num2str(a.x_neg) << ", " << num2str(a.y_pos) << ", " << num2str(a.y_neg);
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

