#if 0  /* === TRACKING CODE DISABLED === */
#ifndef __XUN_H
#define __XUN_H
#include<stdint.h>


void Track_PID_Init(void);

float Get_Position_Feedback(uint8_t *SensorData);
void Track_Control(void);
void Track_Control2(void);

extern uint16_t s;

#endif
#endif  /* === END TRACKING === */
