//=================================
// include guard
#ifndef SONAR
#define SONAR

#include "utility.h"
#include "data_structs.h"
#include "logger.h"
#include "timer.h"
#include "pid_controller.h"

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

#define BAUDRATE_SONAR B9600

class Sonar {
	private:
	int port, data_size;
	struct termios newtio;
	
	bool which_port;
	std::string PATH2SONAR;
	bool first_call = true;
	bool sonarStatus = true;	
	
	float sonarTimeout = 0.6;

	int minDist, maxDist;

	char lastByte = 255;
	char dist[4] = {0};
	int last_distance = 0;
	bool foundFirstByte = false, foundLastByte = false;
	int index = 0;	

	Timer timer;
        PIDController distController_{.001, .001, .001, 1000, 350.00, 2000.00};

	long num_fds_n1 = 0, num_fds_0 = 0, num_fds_1 = 0, num_fds_p = 0;




	fd_set read_fds;
        struct timeval no_timeout;

	public:
	Sonar(std::string PATH2SONAR, int minDist, int maxDist);
	int open_port(std::string PATH2SONAR, int DATASIZE);
	int get_sonar_data(SonarTest& s);
	int check_first_read_test(SonarTest& s);
	SonarTest get_stats(void);
	void printStats(void);
	void print_raw_bytes(const char arr[], int arr_length);
	void calcDt(void);
	float timeSinceLastRead(void);
	int getStatus(void);
	float getDt(void);
	int returnLastDistance(void);
	int indexOfStartByte(const char arr[], int length);
	void clean(void);
	void distanceControlOn(float dist);
	void distanceControlOff(void);
	void getErrors(Errors& e);
	float getPropError(void);
	float getDerivError(void);
	float getIntegralError(void);

	float getDesiredDistance(void);
	bool isDistControlOn(void);
	float getControlOutput(void);

};

#endif
// __SONAR_INCLUDED__
