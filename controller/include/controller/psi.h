//=================================
// include guard
#ifndef PSI
#define PSI

using namespace std;

#include <iostream>
#include <stdio.h>   /* Standard input/output definitions */
#include <iostream>
#include <stdlib.h>
#include <unistd.h>  /* UNIX standard function definitions */
#include <sys/time.h>
#include <time.h>
#include <math.h>
#include <string.h>

#include <timer.h>

class Psi {
        bool first_loop;
        int num_psi_rotations;
        float old_raw_psi, new_raw_psi;
	float old_scaled_psi;
        float dt;

	float scale_factor = 0.059;
 
	Timer psiTimer;

	public:
	Psi(void)
	{
		first_loop = true;
		num_psi_rotations = 0;
		old_raw_psi = 0.0;
		new_raw_psi = 0.0;
	}

	Psi(float dt)
	{
		first_loop = true;
		num_psi_rotations = 0;
		old_raw_psi = 0.0;
		new_raw_psi = 0.0;
		(this -> dt) = dt;
	}

	float updatePsi(float psi_dot)
	{
		float timeSinceLastUpdate = psiTimer.update();
		float old_psi = this->old_raw_psi;

		this->old_raw_psi = (old_raw_psi + psi_dot * timeSinceLastUpdate);
                //printf("Psi Dt: %f, Psi Freq: %f, Old_Psi: %f, Psi_dot: %f, Psi_dot*Dt: %f, new_psi: %f, \n\n", timeSinceLastUpdate, 1/timeSinceLastUpdate,old_psi,psi_dot, psi_dot*timeSinceLastUpdate, (this->old_raw_psi));
		return old_raw_psi;
        }
	float integ_gyro(float psi_dot)
	{
		float dt = psiTimer.update();
		this->old_raw_psi = (old_raw_psi + psi_dot * dt);
		return old_raw_psi;
	}
	//new_raw_psi is the raw psi out of the imu before calibration and with no continuous
	float make_contin(float new_raw_psi)
	{
		//if this is the first loop, store this value for next time
		if(first_loop==true)
			{
				old_raw_psi = new_raw_psi;
				first_loop = false;
			}

		//Check if psi is decreasing toward 0
		if( (old_raw_psi <= 10.0) && (old_raw_psi >= 0.0)  && (new_raw_psi <= 360.0) && (new_raw_psi >= 350.0))
	    {
		    num_psi_rotations--;
		    //printf("num_psi_rotations--");
	    }

		//Else Check if psi is increasing toward 360
		else if( (new_raw_psi <= 10.0) && (new_raw_psi >= 0.0) && (old_raw_psi <= 360.0) && (old_raw_psi >= 350.0))
	    {
		    num_psi_rotations++;
		    //printf("num_psi_rotations++");
	    }


		float psi_to_return  = num_psi_rotations*360 + new_raw_psi;

		old_raw_psi = new_raw_psi;

		return psi_to_return;
	}
	float getDt(void)
	{
		return psiTimer.getDt();
	}
	float getFreq(void)
	{
		return 1/psiTimer.getDt();
	}
	void setDt(float dt)
	{
		(this->dt) = dt;
	}
	int getIter(void)
	{
		return this->num_psi_rotations;
	}
	float getPsi(void)
	{
		return scale_factor * (this->old_raw_psi);
	}

};

#endif
// __PSI_INCLUDED__
