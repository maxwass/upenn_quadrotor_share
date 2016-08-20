//=================================
// include guard
#ifndef SONAR
#define SONAR

#include "utility.h"

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
	timespec oldT, newT;
	float calc_dt;
	bool which_port;
	std::string PATH2SONAR;

	char dist[4];
	bool foundFirstByte, foundLastByte;
	int index;	


	fd_set read_fds;
        struct timeval no_timeout;

	public:
	Sonar(std::string PATH2SONAR);
	int open_port(std::string PATH2SONAR, int DATASIZE);
	int get_sonar_data(void);
	int check_first_read(void);
	void print_raw_bytes(const char arr[], int arr_length);
	void calcDt(void);
	float getDt(void);
	int indexOfStartByte(const char arr[], int length);
	void clean(void);
};

#endif
// __SONAR_INCLUDED__
