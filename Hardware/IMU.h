#ifndef __IMU_H
#define __IMU_H

#include <stdint.h>

typedef struct
{
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} IMU_RawData;

typedef struct
{
    float roll;
    float pitch;
    float yaw;
    float gyro_z_dps;
} IMU_Attitude;

extern IMU_RawData g_imu_raw;
extern IMU_Attitude g_imu_attitude;

void IMU_Init(void);
void IMU_Update(void);
uint8_t IMU_IsReady(void);
float IMU_GetYaw(void);
float IMU_GetGyroZ(void);
void IMU_ResetYaw(void);

#endif
