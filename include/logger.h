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
#ifndef LOGGER_H
#define LOGGER_H


//=================================
// included dependencies
#include "data_structs.h"

#include <stdlib.h>
#include <sys/ioctl.h>
#include <iostream>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
//file io
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <queue>
#include <string>

using namespace std;

class logger
{
   private:
    
    ofstream myfile;
    std::queue<Data_log> q;
    int max_queue_length;
    std::string filename;
    bool first_loop;
    const char *filename_p;
      

    void unwrap(Data_log d);
    void format(Vicon v);
    void format(Times t);
    void format(State s);
    void format(int i);
    void format(State_Error se);
    void format(Errors e);
    void format(Motor_forces mf);
    void format(Angles a);
 
   public:
    logger(std::string filename, int cycles_until_log, bool LOG_DATA);  // This is the constructor
    void log(Data_log d);
    void write_to_file(void);
    template <typename num> string num2str(num f);
    
};
 

#endif
// __LOGGER_H_INCLUDED__
