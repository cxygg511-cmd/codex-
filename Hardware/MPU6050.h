#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdint.h>

void MPU6050_WriteReg(uint8_t reg_address, uint8_t data);
uint8_t MPU6050_ReadReg(uint8_t reg_address);
void MPU6050_Init(void);
uint8_t MPU6050_GetID(void);
void MPU6050_GetData(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z,
                     int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);

#endif
