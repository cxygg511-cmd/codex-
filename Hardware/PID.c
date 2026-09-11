#include "stm32f10x.h"                  // Device header
#include "PID.h"

void PID_Init(PID* pid, float p, float i, float maxOut, float maxI, float reference)
{
	pid->Kp = p;
	pid->Ki = i;
	pid->maxintegral = maxI;
	pid->maxoutput_V = maxOut;
	pid->reference = reference;
	

	pid->error=0;
	pid->integral=0;
	pid->lasterror=0;

	
}





float PID_Calculate(PID *pid, float feedback)
{
	pid->error = pid->reference - feedback;
	
	pid->integral += pid->error * pid->Ki;	

	if(pid->integral > pid->maxintegral) 
        pid->integral = pid->maxintegral;
	if(pid->integral<-pid->maxintegral)
				pid->integral=-pid->maxintegral;

	pid->output_V = ((pid->error) * pid->Kp) + (pid->integral);	
	if(pid->output_V > pid->maxoutput_V) pid->output_V = pid->maxoutput_V;	
	if(pid->output_V < -pid->maxoutput_V) pid->output_V = -pid->maxoutput_V;
	
	pid->output  = pid->output_V;
	    return pid->output;
}
