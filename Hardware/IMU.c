#include "IMU.h"
#include "MPU6050.h"
#include "Delay.h"
#include <math.h>

#define IMU_DT_SEC              0.005f
#define IMU_RAD_TO_DEG          57.2957795f
#define IMU_ACC_SCALE           16384.0f
#define IMU_GYRO_SCALE          131.0f
#define IMU_KP                  0.9f
#define IMU_KI                  0.15f
#define IMU_CALIBRATE_SAMPLES   300
#define IMU_STARTUP_SKIP        50
#define IMU_MPU6050_ID          0x68
#define IMU_MPU6500_ID          0x70
#define IMU_GYRO_Z_Q            0.02f
#define IMU_GYRO_Z_R            1.20f
#define IMU_YAW_Q               0.01f
#define IMU_YAW_R               3.00f
#define IMU_GYRO_Z_DEADBAND_DPS 0.80f
#define IMU_ACC_NORM_MIN        0.49f
#define IMU_ACC_NORM_MAX        1.69f

IMU_RawData g_imu_raw;
IMU_Attitude g_imu_attitude;

static int16_t g_offsets[6];
static uint8_t g_is_ready = 0;

static float q0 = 1.0f;
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;
static float integral_x = 0.0f;
static float integral_y = 0.0f;
static float integral_z = 0.0f;

typedef struct
{
    float value;
    float covariance;
    float process_noise;
    float measurement_noise;
} Kalman1D;

static Kalman1D gyro_z_filter = {0.0f, 1.0f, IMU_GYRO_Z_Q, IMU_GYRO_Z_R};
static Kalman1D yaw_filter = {0.0f, 1.0f, IMU_YAW_Q, IMU_YAW_R};
static uint8_t yaw_filter_ready = 0;
static float yaw_gyro = 0.0f;

static void Kalman_Reset(Kalman1D *filter, float value)
{
    filter->value = value;
    filter->covariance = 1.0f;
}

static float Kalman_Update(Kalman1D *filter, float measurement)
{
    float gain;

    filter->covariance += filter->process_noise;
    gain = filter->covariance / (filter->covariance + filter->measurement_noise);
    filter->value += gain * (measurement - filter->value);
    filter->covariance *= (1.0f - gain);

    return filter->value;
}

static float WrapAngle(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }

    while (angle < -180.0f)
    {
        angle += 360.0f;
    }

    return angle;
}

static float AngleDiff(float target, float current)
{
    return WrapAngle(target - current);
}

static float Kalman_UpdateAngle(Kalman1D *filter, float measurement)
{
    float gain;
    float error;

    filter->covariance += filter->process_noise;
    gain = filter->covariance / (filter->covariance + filter->measurement_noise);
    error = AngleDiff(measurement, filter->value);
    filter->value = WrapAngle(filter->value + gain * error);
    filter->covariance *= (1.0f - gain);

    return filter->value;
}

static float InvSqrt(float x)
{
    float half_x = 0.5f * x;
    float y = x;
    long i = *(long *)&y;

    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (half_x * y * y));
    return y;
}

static void ReadRawData(void)
{
    MPU6050_GetData(&g_imu_raw.acc_x, &g_imu_raw.acc_y, &g_imu_raw.acc_z,
                    &g_imu_raw.gyro_x, &g_imu_raw.gyro_y, &g_imu_raw.gyro_z);
}

static void CalibrateOffsets(void)
{
    uint16_t i;
    int32_t sum[6] = {0};

    for (i = 0; i < IMU_STARTUP_SKIP; i++)
    {
        ReadRawData();
        Delay_ms(5);
    }

    for (i = 0; i < IMU_CALIBRATE_SAMPLES; i++)
    {
        ReadRawData();
        sum[0] += g_imu_raw.acc_x;
        sum[1] += g_imu_raw.acc_y;
        sum[2] += g_imu_raw.acc_z - (int16_t)IMU_ACC_SCALE;
        sum[3] += g_imu_raw.gyro_x;
        sum[4] += g_imu_raw.gyro_y;
        sum[5] += g_imu_raw.gyro_z;
        Delay_ms(5);
    }

    for (i = 0; i < 6; i++)
    {
        g_offsets[i] = (int16_t)(sum[i] / IMU_CALIBRATE_SAMPLES);
    }
}

void IMU_Init(void)
{
    uint8_t mpu_id;

    MPU6050_Init();
    mpu_id = MPU6050_GetID();
    g_is_ready = (mpu_id == IMU_MPU6050_ID || mpu_id == IMU_MPU6500_ID);

    if (g_is_ready)
    {
        CalibrateOffsets();
        Kalman_Reset(&gyro_z_filter, 0.0f);
        Kalman_Reset(&yaw_filter, 0.0f);
        yaw_filter_ready = 0;
        yaw_gyro = 0.0f;
    }
}

void IMU_Update(void)
{
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float recip_norm;
    float vx;
    float vy;
    float vz;
    float ex;
    float ey;
    float ez;
    float half_gx;
    float half_gy;
    float half_gz;
    float q0_delta;
    float q1_delta;
    float q2_delta;
    float q3_delta;
    float r1;
    float r4;
    float r7;
    float r8;
    float r9;
    float raw_gyro_z_dps;
    float acc_norm_sq;

    if (!g_is_ready)
    {
        return;
    }

    ReadRawData();

    ax = (g_imu_raw.acc_x - g_offsets[0]) / IMU_ACC_SCALE;
    ay = (g_imu_raw.acc_y - g_offsets[1]) / IMU_ACC_SCALE;
    az = (g_imu_raw.acc_z - g_offsets[2]) / IMU_ACC_SCALE;
    gx = ((g_imu_raw.gyro_x - g_offsets[3]) / IMU_GYRO_SCALE) / IMU_RAD_TO_DEG;
    gy = ((g_imu_raw.gyro_y - g_offsets[4]) / IMU_GYRO_SCALE) / IMU_RAD_TO_DEG;
    gz = ((g_imu_raw.gyro_z - g_offsets[5]) / IMU_GYRO_SCALE) / IMU_RAD_TO_DEG;

    raw_gyro_z_dps = (g_imu_raw.gyro_z - g_offsets[5]) / IMU_GYRO_SCALE;
    g_imu_attitude.gyro_z_dps = Kalman_Update(&gyro_z_filter, raw_gyro_z_dps);
    if (g_imu_attitude.gyro_z_dps > -IMU_GYRO_Z_DEADBAND_DPS &&
        g_imu_attitude.gyro_z_dps < IMU_GYRO_Z_DEADBAND_DPS)
    {
        g_imu_attitude.gyro_z_dps = 0.0f;
    }
    gz = g_imu_attitude.gyro_z_dps / IMU_RAD_TO_DEG;

    acc_norm_sq = ax * ax + ay * ay + az * az;
    recip_norm = InvSqrt(acc_norm_sq);
    ax *= recip_norm;
    ay *= recip_norm;
    az *= recip_norm;

    vx = 2.0f * (q1 * q3 - q0 * q2);
    vy = 2.0f * (q0 * q1 + q2 * q3);
    vz = 1.0f - 2.0f * (q1 * q1 + q2 * q2);

    if (acc_norm_sq > IMU_ACC_NORM_MIN && acc_norm_sq < IMU_ACC_NORM_MAX)
    {
        ex = ay * vz - az * vy;
        ey = az * vx - ax * vz;
    }
    else
    {
        ex = 0.0f;
        ey = 0.0f;
    }
    ez = 0.0f;

    integral_x += ex * IMU_KI * IMU_DT_SEC;
    integral_y += ey * IMU_KI * IMU_DT_SEC;
    integral_z = 0.0f;

    gx += ex * IMU_KP + integral_x;
    gy += ey * IMU_KP + integral_y;

    half_gx = 0.5f * gx * IMU_DT_SEC;
    half_gy = 0.5f * gy * IMU_DT_SEC;
    half_gz = 0.5f * gz * IMU_DT_SEC;

    q0_delta = -q1 * half_gx - q2 * half_gy - q3 * half_gz;
    q1_delta =  q0 * half_gx + q2 * half_gz - q3 * half_gy;
    q2_delta =  q0 * half_gy - q1 * half_gz + q3 * half_gx;
    q3_delta =  q0 * half_gz + q1 * half_gy - q2 * half_gx;

    q0 += q0_delta;
    q1 += q1_delta;
    q2 += q2_delta;
    q3 += q3_delta;

    recip_norm = InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recip_norm;
    q1 *= recip_norm;
    q2 *= recip_norm;
    q3 *= recip_norm;

    r1 = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
    r4 = 2.0f * (q1 * q2 + q0 * q3);
    r7 = 2.0f * (q1 * q3 - q0 * q2);
    r8 = 2.0f * (q2 * q3 + q0 * q1);
    r9 = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    if (r7 > 1.0f) r7 = 1.0f;
    if (r7 < -1.0f) r7 = -1.0f;

    g_imu_attitude.roll = atan2f(r8, r9) * IMU_RAD_TO_DEG;
    g_imu_attitude.pitch = -asinf(r7) * IMU_RAD_TO_DEG;

    yaw_gyro = WrapAngle(yaw_gyro + g_imu_attitude.gyro_z_dps * IMU_DT_SEC);
    if (!yaw_filter_ready)
    {
        Kalman_Reset(&yaw_filter, yaw_gyro);
        yaw_filter_ready = 1;
    }
    g_imu_attitude.yaw = Kalman_UpdateAngle(&yaw_filter, yaw_gyro);
}

uint8_t IMU_IsReady(void)
{
    return g_is_ready;
}

float IMU_GetYaw(void)
{
    return g_imu_attitude.yaw;
}

float IMU_GetGyroZ(void)
{
    return g_imu_attitude.gyro_z_dps;
}
void IMU_ResetYaw(void)
{
    yaw_gyro = 0.0f;
    g_imu_attitude.yaw = 0.0f;
    Kalman_Reset(&yaw_filter, 0.0f);
    yaw_filter_ready = 1;
}
