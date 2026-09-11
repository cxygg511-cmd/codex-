#ifndef __PID_H
#define __PID_H
#include "stdint.h"

typedef struct
{
    float Kp,Ki;
    float error,lasterror;
    float integral,maxintegral;
    float output_V,maxoutput_V;
		float reference;
	
    
	
		float output;

}PID;

void PID_Init(PID* pid, float p, float i, float maxOut, float maxI, float reference);

float PID_Calculate(PID *pid, float feedback);




#endif
