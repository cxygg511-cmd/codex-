#ifndef __PWM_H
#define __PWM_H
#include<stdint.h>

void Pwm_Init(void);
void Pwm_SetCompare3(uint16_t Comare);
void Pwm_SetCompare4(uint16_t Comare);
void Pwm_SetCompare2(uint16_t Comare);
void Pwm_SetCompare1(uint16_t Comare);


#endif
