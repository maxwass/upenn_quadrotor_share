//=================================
// include guard
#ifndef TIMER
#define TIMER

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

class Timer {

 private:
  timespec startTime_, oldT_, newT_;
  float calcDt_ = 0.0;
  bool firstCall = true;

 public:
  Timer(void)
    {
      clock_gettime(CLOCK_REALTIME,&oldT_);
      clock_gettime(CLOCK_REALTIME,&newT_);
      clock_gettime(CLOCK_REALTIME,&startTime_);
    }
  
  float update(void)
  {
    if(firstCall)
      {
	firstCall = false;
	UTILITY::calcDt(oldT_,newT_);
	return 0.0;
      }
    (this->calcDt_) = UTILITY::calcDt(oldT_,newT_);
    return calcDt_;
  }
  float getDt(void)
  {
    return calcDt_;
  }

  float timeSinceStart(void)
  {
    timespec now;
    clock_gettime(CLOCK_REALTIME,&now);
    return UTILITY::timespec2float(UTILITY::time_diff(startTime_,now));
  }

};

#endif
// __TIMER_INCLUDED__
