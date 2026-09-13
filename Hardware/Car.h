#ifndef __CAR_H
#define __CAR_H

#include <stdint.h>

void Car_Init(void);
void Car_Move(int8_t vx, int8_t vy, int8_t omega);
void Car_Update(void);
void Move_Forward(void);
void Move_Backward(void);
void Car_Stop(void);
void Car_BrakeNow(void);
void Move_Left(void);
void Move_Right(void);

#endif


