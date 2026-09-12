#ifndef __MYI2C_H
#define __MYI2C_H

#include <stdint.h>

void MyI2C_Init(void);
void MyI2C_Start(void);
void MyI2C_Stop(void);
void MyI2C_SendByte(uint8_t byte);
uint8_t MyI2C_ReceiveByte(void);
void MyI2C_SendAck(uint8_t ack_bit);
uint8_t MyI2C_ReceiveAck(void);
uint8_t MyI2C_ReadSclLine(void);
uint8_t MyI2C_ReadSdaLine(void);
uint8_t MyI2C_CheckDevice(uint8_t device_address);

#endif
