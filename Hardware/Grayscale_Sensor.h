#if 0  /* === TRACKING CODE DISABLED === */
#ifndef _GRAYSCALE_SENSOR_H
#define _GRAYSCALE_SENSOR_H

#include "stm32f10x.h"                  // Device header

/*----------------------灰度传感器的I2C引脚----------------------*/
#define			GRAY_I2C_PORT										GPIOB
#define			GRAY_I2C_SDA										GPIO_Pin_14
#define			GRAY_I2C_SCL										GPIO_Pin_15


#define     GRAY_DELAY_TIME                 5
#define     GRAYSCALE_SENSOR_ADDRESS        0x98


/*------------------------I2C---------------------------*/
uint8_t GRAY_I2C_R_SDA(void);
void GRAY_I2C_Init(void);
void GRAY_I2C_Start(void);
void GRAY_I2C_Stop(void);
void GRAY_I2C_SendByte(uint8_t Byte);
uint8_t GRAY_I2C_ReceiveByte(void);
void GRAY_I2C_SendAck(uint8_t AckBit);
uint8_t GRAY_I2C_ReceiveAck(void);

/*------------------------Grayscale_Sensor---------------------------*/
void GRAY_Init(void);
void GRAY_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t GRAY_ReadReg(uint8_t RegAddress);
uint8_t GRAY_Only_ReadReg(void);
uint8_t GRAY_ADDR_Scan(void);
uint8_t GRAY_Ping(void);
void GRAY_Single_Analog_ReadReg(uint8_t Channel,uint8_t *Data);
void GRAY_Analog_ReadReg(uint8_t *Data);
void GRAY_Only_Analog_ReadReg(uint8_t *Data);
void Check_Firmware_Version(uint8_t *Version);
uint8_t Transmission_Channel(uint8_t R_OR_W,uint8_t Data);
uint8_t Channel_Data_Normalization(uint8_t R_OR_W,uint8_t Data);

#endif
#endif  /* === END TRACKING === */
