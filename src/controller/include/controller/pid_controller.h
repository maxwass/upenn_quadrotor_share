//=================================
// include guard
#ifndef PID
#define PID

#include "utility.h"
#include "data_structs.h"
#include "logger.h"
#include "timer.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>
#include <math.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>
#include <iostream>
#include <deque>

using namespace std;

class PIDController {
	private:
	bool first_call_ = true;
	bool FLAG, LARGE_VALUE, SMALL_VALUE = false;
	float maxAllowableValue_, minAllowableValue_ = 0.000;
	float maxDerivValue_ = 0.00;

	float valueNew_, valueOld_, valueDeriv_ = 0.0000;
	float filterValueOld_, filterValueNew_ = 0.000;
	float desiredValue_ = 0.00000;
	float desiredValueDeriv_ = 0.00000;
	float kp_, kd_, ki_ = 0.000;
	float lastDt_ = 0.1000;
	float propError_,propErrorOld_, derivError_, integError_ = 0.0000;
	float filterPropError_, filterPropErrorOld_, filterDerivError_, filterIntegError_ = 0.000;
	float maxIntegralError_ = 10.000;
	float minValue_, maxValue_ = 0.0000;
        int numCalls = 0;

	std::deque<float> prevValues;
	//float weights[10] = {0.35, 0.25, 0.10, 0.10, .05, .05, .025, .025, .025, .025};
	float weights[20] = {0.0875, 0.0875, 0.0875, 0.0875, 0.0625, 0.0625, 0.0625, 0.0625, 0.05, 0.05, 0.05, 0.05, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025, 0.025};

	public:
	
	PIDController(float kp, float kd, float ki, float desired_value, float minAllowableValue, float maxAllowableValue)
	{
		setDesiredValue(desired_value);
		minAllowableValue_  =  minAllowableValue;
		maxAllowableValue_  =  maxAllowableValue;
			
	
		float maxDerivValue = 5.00;
		maxDerivValue_ = maxDerivValue;


		kp_ = kp;
		kd_ = kd;
		ki_ = ki;

		prevValues.resize(100,0.00);
	
		printf("END OF ALT CONSTRUCTOR\n\n");
	}

	float LowPassFilter(float input, float input_old, float input_old_old, float a, float b, float c)
	{
	    float output = a * input + b * input_old + c * input_old_old;
	    //printf("out: %e %e %e %e %e %e %e\n", a, b, c, input, input_old, input_old_old, output);
	    return output;
	}

	void setGains(float kp, float kd, float ki)
	{
		kp_ = kp;
		kd_ = kd;
		ki_ = ki;
	}
	void printData(void)
	{

		
	       //printf("FLAG: %i, kp: %f, kd: %f, ki: %f \n", FLAG, kp_, kd_, ki_);
	       printf("new value: %f, old value: %f, desiredValue_: %f, dt: %f, freq: %f \n", valueNew_, valueOld_, getDesiredValue(), lastDt_, 1/lastDt_);
		printf("filterValueNew: %f, filterValueOld: %f \n\n", filterValueNew_, filterValueOld_);
	       //printf("Calc: Error: desired value - new value : %f, Diff: Error:propError - propErrorOld: %f Deriv Error: (Diff Error)/dt %f \n", (getDesiredValue() -valueNew_), (propError_ - propErrorOld_), (propError_ - propErrorOld_)/lastDt_);
	       //printf("Print: Error : %f,  Deriv Error: %f, Integral Error: %f \n", propError_, derivError_, integError_);
	       //printf("Control Output: kp*propError: %f + kd*DerivError: %f: = %f \n\n",kp_*propError_, kd_*derivError_, this->getControlOutput());

	}
	void update(float valueNew, float dt)
	{

		if(first_call_)
		{
			first_call_ = false;
			maxValue_ = valueNew;
			minValue_ = valueNew;
			lastDt_ = dt;
			valueOld_ = valueNew;
			valueNew_ = valueNew;	
		}		

		//record the extreme values are
		if(valueNew >= maxValue_) maxValue_ = valueNew;
		if(valueNew <= minValue_) minValue_ = valueNew;
		
		//Cap and floor values
		if(valueNew > maxAllowableValue_) 
		{
			LARGE_VALUE = true;
			valueNew_ = valueOld_;
		}
		else if(valueNew < minAllowableValue_) 
		{
			SMALL_VALUE = true;
			valueNew_ = minAllowableValue_;
		} 
		else 
		{
			valueNew_ = valueNew;
			LARGE_VALUE = false; 
			SMALL_VALUE = false;
		}
		
	       // printf("old value: %f, new value raw : %f, new value cap:: %f,  desiredValue_: %f, dt: %f, propError_: %f, derivError_: %f \n\n\n", valueOld_, valueNew, valueNew_, getDesiredValue(), dt, desiredValue_-valueNew,  - (valueNew - valueOld_)/dt);
		//printData();


		lastDt_ = dt;
		valueDeriv_ = (valueNew_ - valueOld_)/dt;

		
		propErrorOld_ = propError_;
		propError_  =  getDesiredValue()      - valueNew_;
		
		derivError_ =  (propError_ - propErrorOld_)/dt;
		integError_ += propError_*dt;
		if(integError_ >=  maxIntegralError_) integError_   =  maxIntegralError_;
		if(integError_ <= - maxIntegralError_) integError_   = -maxIntegralError_;
 		
		//Filter
		prevValues.push_front(valueNew_);
		prevValues.pop_back();
		
		filterValueOld_ = filterValueNew_;
		filterValueNew_ = calcFilterValue();

		filterPropErrorOld_ = filterPropError_;
		filterPropError_    = desiredValue_ - filterValueNew_;

		filterDerivError_   = (filterPropError_ - filterPropErrorOld_)/dt;  
		filterIntegError_ += filterPropError_*dt;
		if(filterIntegError_ >=  maxIntegralError_)  filterIntegError_   =  maxIntegralError_;
		if(filterIntegError_ <= - maxIntegralError_) filterIntegError_   = -maxIntegralError_;
 		
		if(filterDerivError_ >= maxDerivValue_) filterDerivError_ = maxDerivValue_;
		if(filterDerivError_ <= -maxDerivValue_) filterDerivError_ = -maxDerivValue_;


		valueOld_ = valueNew_;
		//printf("valueNew: %f, filtValOld: %f, filtValNew: %f, filtPropError: %f, filterDerivError: %f \n", valueNew_, filterValueOld_, filterValueNew_, filterPropError_, filterDerivError_);


	}
	float calcFilterValue(void)
	{
		float output = 0.0;
		numCalls = numCalls + 1;
		 //printf("numCalls %i: ", numCalls);
		 int i = 0;
		 int size = prevValues.size();
		 for (std::deque<float>::iterator it = prevValues.begin(); it!=prevValues.end(); ++it)
		 {
			//printf("(%f * %f) + ",weights[i], *it);
			//output += weights[i]*(*it);
			output += (*it);
			i++;
			//printf(" new_output: %f, output: %f, i: %i \n",(*it), output, i);
		 }
		output = output/size;
		//printf(" = output: %f", output);


	//printf("\n\n");
		return output;

	}
	
	float getControlOutput(void)
	{
		if(FLAG)
		{

			if(LARGE_VALUE || SMALL_VALUE) return 0;
			float  output = kp_*propError_ + kd_*derivError_;// + ki_*integError_;
			if(output > 10) return 10.00;
			else if(output < -10) return -10.00;

			return output;
		}
		
		else 	return 0;
	}
	float getMaxValue(void)
	{
		return maxValue_;
	}
	float getMinValue(void)
        {
                return minValue_;
        }
	void resetIntegral(void)
	{
		integError_ = 0.0;
	}
	void autoControlOff(void)
	{
		resetIntegral();
		FLAG = false;
	}
	void autoControlOn(float desiredValue)
        {
		setDesiredValue(desiredValue);
		//for smooth transition
		valueOld_ = desiredValue;
                resetIntegral();
                FLAG = true;
        }
	bool isAutoControlOn(void)
	{
		return FLAG;
	}
	void setDesiredValue(float desiredValue)
	{
		
		//printf("  !!!!!!!!!CHANGING: before: %f   after:  %f !!!!!!\n\n\n", desiredValue_, desiredValue);
		//usleep(1000000);
		desiredValue_ = desiredValue;
	}
	float getFiltValue(void)
	{
		return this->filterValueNew_;
	}
        float getValue(void)
	{
		return valueNew_;
	}
	float getDesiredValue(void)
        {
                return desiredValue_;
        }
	
	float getValueDeriv(void)
	{
		return valueDeriv_;
	}
	float getPropError(void)
	{	
		return propError_;
	}
	float getFiltPropError(void)
	{
		return filterPropError_;
	}
	float getDerivError(void)
        {
                return derivError_;
        }
	float getFiltDerivError(void)
        {
                return filterDerivError_;
        }
	float getIntegralError(void)
        {
                return integError_;
        }
	float getFiltIntegralError(void)
        {
                return filterIntegError_;
        }
};

#endif
// __PID__
