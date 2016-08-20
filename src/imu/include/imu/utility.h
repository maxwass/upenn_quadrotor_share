#ifndef UTILS_H
#define UTILS_H


#include <iostream>
#include <stdio.h>   /* Standard input/output definitions */
#include <iostream>
#include <stdlib.h>
#include <unistd.h>  /* UNIX standard function definitions */
#include <sys/time.h>                                                                                                                                                     
#include <time.h>
#include <math.h>
#include <fcntl.h>   /* File control definitions */
#include <termios.h> /* POSIX terminal control definitions, Unix API for terminal I/O */
#include <string.h>

timespec time_diff(timespec start, timespec end);                                                                                                                         
double timespec2float (const timespec& time);

class UTILITY{
        public:
        static timespec time_diff(timespec start, timespec end)
        {
                timespec temp;
                if ((end.tv_nsec-start.tv_nsec)<0)
                {
                   temp.tv_sec = end.tv_sec-start.tv_sec-1;
                   temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
                }
                else
                {
                    temp.tv_sec = end.tv_sec-start.tv_sec;
                    temp.tv_nsec = end.tv_nsec-start.tv_nsec;
                }
                return temp;
        }

        static double timespec2float (const timespec& time)
        {
                return ((double) time.tv_sec + (time.tv_nsec / 1000000000.0));
        }

	static float calcDt(timespec& oldT, timespec& newT){
                //track dt between reads
                clock_gettime(CLOCK_REALTIME,&newT);
                float dt = UTILITY::timespec2float(UTILITY::time_diff(oldT, newT));
                clock_gettime(CLOCK_REALTIME,&oldT);
                return dt;
	}
	
	static float dist2Scale(int dist, int minDist2Wall, int maxDist2Wall)
	{	//input is a distance
		//linearly mapped to a scale factor [0, 1] 
		// when close to object, will be 0, when far, will be 1
			// will be used as a desired angle for repulsive force and to lmit angle when close to wall
	
		if(dist >=  maxDist2Wall)
		{
		      //printf("case 1: distance  %i >= maxDist2Wall %i, scale: %f \n", dist, maxDist2Wall, 1.00);
		      return 1.00;
			
		}
		else if(dist <= minDist2Wall)
		{
			//printf("case 2: distance  %i <= minDist2Wall %i, scale: %f\n", dist, minDist2Wall, 0.00);
 			return 0.00;
		}

		else
		{

			float scale = (float) dist/(maxDist2Wall-minDist2Wall) -  (float) minDist2Wall/(maxDist2Wall-minDist2Wall) ;
			//printf("case 3: distance  %i,  minDist2Wall %i,  maxDist2Wall %i, scale %f \n\n", dist, minDist2Wall, maxDist2Wall, scale);
			return scale;
		}


	}

	static float dist2ScaleInv(int dist, int minDist2Wall, int maxDist2Wall)
        {       //inverse of the output of dist2Scale
		return (1 - UTILITY::dist2Scale(dist, minDist2Wall, maxDist2Wall)) ;
        }

};


#endif   
