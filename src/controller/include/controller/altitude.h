//=================================
// include guard
#ifndef ALTITUDE
#define ALTITUDE

#include "utility.h"
#include "data_structs.h"
#include "logger.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>
#include <math.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>
using namespace std;

class Altitude {
	private:
	bool first_call_ = true;
	bool AUTO_HEIGHT = false;
	float bias_, altitude_, altitude_deriv_ = 0.0;
	float desiredAltitude_, desiredAltitudeDeriv_ = 0.0;
	float kp_, kd_, ki_ = 0.0;
	float propError_, derivError_, integError_ = 0.0;
	float maxIntegralError_ = 10;
	float minAltitude_, maxAltitude_ = 0.0;

	public:
	void setGains(float kp, float kd, float ki)
	{
		kp_ = kp;
		kd_ = kd;
		ki_ = ki;
	}
	void update(float altitudeNew, float dt)
	{
		if(altitudeNew >= maxAltitude_) maxAltitude_ = altitudeNew;
		if(altitudeNew <= minAltitude_) minAltitude_ = altitudeNew;
	


		if(altitudeNew <= 0.00) altitudeNew = 0.00;
      
	//printf("old altitude: %f, new altitude: %f, desiredAltitude_: %f, dt: %f, propError_: %f, derivError_: %f \n\n\n", altitude_, altitudeNew, desiredAltitude_, dt, desiredAltitude_-altitudeNew,  - (altitudeNew - altitude_)/dt);

		altitude_deriv_ = (altitudeNew - altitude_)/dt;

		propError_  =  desiredAltitude_      - altitudeNew;
		derivError_ =  desiredAltitudeDeriv_ - altitude_deriv_;
		integError_ += propError_*.03;
		if(integError_ >=  maxIntegralError_) integError_   =  maxIntegralError_;
		if(integError_ <= - maxIntegralError_) integError_   = -maxIntegralError_;
 
		altitude_ = altitudeNew;
	}
	float getMaxAltitude(void)
	{
		return maxAltitude_;
	}
	float getMinAltitude(void)
        {
                return minAltitude_;
        }
	void resetIntegral(void)
	{
		integError_ = 0.0;
	}
	void autoAltitudeOff(void)
	{
		resetIntegral();
		AUTO_HEIGHT = false;
	}
	void autoAltitudeOn(float desiredAltitude)
        {
		desiredAltitude_ = desiredAltitude;
                resetIntegral();
                AUTO_HEIGHT = true;
        }
	void setDesiredAltitude(float desiredAltitude)
	{
		desiredAltitude_ = desiredAltitude;
	}
        float getAltitude(void)
	{
		return altitude_;
	}
	float getDesiredAltitude(void)
        {
                return desiredAltitude_;
        }
	float getAltitudeDeriv(void)
	{
		return altitude_deriv_;
	}
	float getPropError(void)
	{	
		return propError_;
	}
	float getDerivError(void)
        {
                return derivError_;
        }
	float getIntegralError(void)
        {
                return integError_;
        }
};

#endif
// __SONAR_INCLUDED__
