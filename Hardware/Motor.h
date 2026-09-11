#ifndef __MOTOR_H
#define __MOTOR_H
#include "stdint.h"

void Motor_Init(void);

/* 4 轮独立控制（麦轮全向移动需要） */
void Wheel_FL_Speed(int8_t Speed);  // 左前
void Wheel_FR_Speed(int8_t Speed);  // 右前
void Wheel_RL_Speed(int8_t Speed);  // 左后
void Wheel_RR_Speed(int8_t Speed);  // 右后

#endif
