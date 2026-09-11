#if 0  /* === TRACKING CODE DISABLED === */
#ifndef __GRAY_SENSOR_H
#define __GRAY_SENSOR_H

#include "stdint.h"

void GraySensor_Init(void);
void GraySensor_ReadAllValues(uint16_t* values);
float GraySensor_GetPosition(void);


#endif
#endif  /* === END TRACKING === */
