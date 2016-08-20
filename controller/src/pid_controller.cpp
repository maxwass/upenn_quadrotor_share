#include "pid_controller.h"




int main(void)
{

	PIDController pid;
	pid.setGains(1.0, 2.0, .001);

	Timer timer;

	int i = 0;

	while(1)
	{
		i += 2;
		//if(i%100 == 0) pid.autoControlOn(pid.getValue());
		//if(i%77  == 0) pid.autoControlOff();
		timer.update();
		
		pid.printData();	
		
		pid.update(i, timer.getDt());



	}




}
