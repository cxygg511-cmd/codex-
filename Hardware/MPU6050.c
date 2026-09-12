#include "MPU6050.h"
#include "MPU6050_REG.h"
#include "MyI2C.h"
#include "Delay.h"

#define MPU6050_ADDRESS_LOW   0xD0
#define MPU6050_ADDRESS_HIGH  0xD2
#define MPU6050_ID_6050       0x68
#define MPU6050_ID_6500       0x70

static uint8_t MpuAddress = MPU6050_ADDRESS_LOW;

void MPU6050_WriteReg(uint8_t reg_address, uint8_t data)
{
    MyI2C_Start();
    MyI2C_SendByte(MpuAddress);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(reg_address);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(data);
    MyI2C_ReceiveAck();
    MyI2C_Stop();
}

static uint8_t MPU6050_ReadRegFrom(uint8_t device_address, uint8_t reg_address)
{
    uint8_t data;

    MyI2C_Start();
    MyI2C_SendByte(device_address);
    MyI2C_ReceiveAck();
    MyI2C_SendByte(reg_address);
    MyI2C_ReceiveAck();

    MyI2C_Start();
    MyI2C_SendByte(device_address | 0x01);
    MyI2C_ReceiveAck();
    data = MyI2C_ReceiveByte();
    MyI2C_SendAck(1);
    MyI2C_Stop();

    return data;
}

static uint8_t MPU6050_DetectAddress(void)
{
    MpuAddress = MPU6050_ADDRESS_LOW;
    if (MPU6050_ReadRegFrom(MPU6050_ADDRESS_LOW, MPU6050_WHO_AM_I) == MPU6050_ID_6050 ||
        MPU6050_ReadRegFrom(MPU6050_ADDRESS_LOW, MPU6050_WHO_AM_I) == MPU6050_ID_6500)
    {
        return 1;
    }

    MpuAddress = MPU6050_ADDRESS_HIGH;
    if (MPU6050_ReadRegFrom(MPU6050_ADDRESS_HIGH, MPU6050_WHO_AM_I) == MPU6050_ID_6050 ||
        MPU6050_ReadRegFrom(MPU6050_ADDRESS_HIGH, MPU6050_WHO_AM_I) == MPU6050_ID_6500)
    {
        return 1;
    }

    MpuAddress = MPU6050_ADDRESS_LOW;
    return 0;
}

uint8_t MPU6050_ReadReg(uint8_t reg_address)
{
    return MPU6050_ReadRegFrom(MpuAddress, reg_address);
}

void MPU6050_Init(void)
{
    MyI2C_Init();
    Delay_ms(100);

    MPU6050_DetectAddress();

    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x80);
    Delay_ms(100);
    MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01);
    Delay_ms(20);
    MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
    MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x04);    // 200 Hz output from 1 kHz gyro rate
    MPU6050_WriteReg(MPU6050_CONFIG, 0x03);        // DLPF about 44 Hz
    MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x00);   // +/-250 deg/s
    MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00);  // +/-2 g
}

uint8_t MPU6050_GetID(void)
{
    MPU6050_DetectAddress();
    return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

void MPU6050_GetData(int16_t *acc_x, int16_t *acc_y, int16_t *acc_z,
                     int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z)
{
    uint8_t data_h;
    uint8_t data_l;

    data_h = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H);
    data_l = MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
    *acc_x = (int16_t)((data_h << 8) | data_l);

    data_h = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H);
    data_l = MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
    *acc_y = (int16_t)((data_h << 8) | data_l);

    data_h = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H);
    data_l = MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
    *acc_z = (int16_t)((data_h << 8) | data_l);

    data_h = MPU6050_ReadReg(MPU6050_GYRO_XOUT_H);
    data_l = MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
    *gyro_x = (int16_t)((data_h << 8) | data_l);

    data_h = MPU6050_ReadReg(MPU6050_GYRO_YOUT_H);
    data_l = MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
    *gyro_y = (int16_t)((data_h << 8) | data_l);

    data_h = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H);
    data_l = MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
    *gyro_z = (int16_t)((data_h << 8) | data_l);
}
