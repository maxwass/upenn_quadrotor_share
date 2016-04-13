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

class Psi {
        bool first_loop;
        int num_psi_rotations;
        float old_raw_psi;
        float new_raw_psi;
        float dt;
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
float integ_gyro(float psi_dot)
{
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
                        //printf("first loop psi %f !!!!!!!!!!!!!!!!!!!\n\n\n\n", new_raw_psi);
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
return this->dt;
}
void setDt(float dt)
{
 (this->dt) = dt;
}
int getIter(void)
{
return this->num_psi_rotations;
}

};

#endif
// __PSI_INCLUDED__
