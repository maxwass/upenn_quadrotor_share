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

//=================================
// include guard
#ifndef SONAR
#define SONAR

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

#define BAUDRATE_SONAR B9600
/*
typedef struct sonartest {
int distance;
int succ_read;
char byte_read;
float dt;
int index;
bool foundFirstByte, foundLastByte;
} SonarTest;
*/
class Sonar {
	private:
	int port, data_size;
	struct termios newtio;
	timespec oldT, newT;
	float calc_dt;
	bool which_port;
	std::string PATH2SONAR;
	
	char lastByte = 255;
	char dist[4] = {0};
	int last_distance = 0;
	bool foundFirstByte = false, foundLastByte = false;
	int index = 0;	

	long num_fds_n1 = 0, num_fds_0 = 0, num_fds_1 = 0, num_fds_p = 0;


	fd_set read_fds;
        struct timeval no_timeout;

	public:
	Sonar(std::string PATH2SONAR);
	int open_port(std::string PATH2SONAR, int DATASIZE);
	int get_sonar_data(void);
	int get_sonar_data(SonarTest& s);
	int check_first_read(void);
	int check_first_read_test(SonarTest& s);
	SonarTest get_stats(void);
	void printStats(void);
	void print_raw_bytes(const char arr[], int arr_length);
	void calcDt(void);
	float getDt(void);
	int returnLastDistance(void);
	int indexOfStartByte(const char arr[], int length);
	void clean(void);
};

#endif
// __SONAR_INCLUDED__
