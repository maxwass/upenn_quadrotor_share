
Copyright (c) <2015>, <University of Pennsylvania: GRASP Lab>
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



    //Compilation Instructions
In order to run the 'main' function in a certain file, it must be uncommented, with all other 'main' functions in the other files in /src commented out. The 'main's in the other files are used for testing the functions internal to that file.

For example, to run the main controller we use g++.
ex) g++ controller.cpp vicon.cpp motor.cpp imu_inv.cpp logger.cpp utility.cpp -I /home/odroid/upenn_quad/include -lpthread -lncurses -lboost_system

This should be run when in the /src folder. Each of the files above (controller.cpp, vicon.cpp, etc) have functions used in the control loop. The -I /home/od... tells the compiler where to find the header files. We also link some libraries at the end, such as -lpthread, for the threads implementation, and -lncurses for our terminal output.

If we would like to run the test functions in somwhere, like vicon.cpp, which house the functions to recieve vicon or joystick data, we would comment out the 'main' in controller, navigate to the vicon.cpp file, uncomment its 'main, and find the compilation instructions at the top of the file.


    //Program Layout
controller.cpp: main control loop. Uses functions from imu_inv.cpp and vicon.cpp to recieve data. It performs calculations on this data to eventually ouput motor signals via a method defined in Motor.cpp. Instantiated logger object to log data.

imu_inv.cpp: hanldes the reading of the tungsten imu. Also houses functions for finding biases.

logger.cpp: implementation of a simple logging class to write CSV data to a .txt file

motor.cpp: class to abstract motor commands. Handles the writing to motors via i2c.

vicon.cpp: handles the reading of vicon/joystick data via xbee.

utility.cpp: houses common functions used by a few files above.

../include/data_structs.h: custom data structures used for abstraction.

the directory send_joystick_data houses the functions necessary use the joystick for sending instructions to the quadrotor


   //Misc
to detect devices on the i2c bus (such as out motors), use the command: sudo i2cdetect -y 10.
