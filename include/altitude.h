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
	float bias_, altitude_, altitude_deriv_ = 0.0;
	

	public:
	//Altitude();
	void update(float altitude, float dt)
	{
		altitude_deriv_ = (altitude - altitude_)/dt;

		altitude_ = altitude;
	}
	float getAltitudeDeriv(void)
	{
		return altitude_deriv_;
	
	}
};

#endif
// __SONAR_INCLUDED__
