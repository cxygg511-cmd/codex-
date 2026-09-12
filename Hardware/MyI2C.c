#include "stm32f10x.h"
#include "Delay.h"
#include "MyI2C.h"

#define MYI2C_GPIO_PORT      GPIOB
#define MYI2C_GPIO_CLOCK     RCC_APB2Periph_GPIOB
#define MYI2C_SCL_PIN        GPIO_Pin_13
#define MYI2C_SDA_PIN        GPIO_Pin_12

static void MyI2C_Delay(void)
{
    Delay_us(5);
}

static void MyI2C_SetSdaOutput(void)
{
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Mode = GPIO_Mode_Out_OD;
    gpio.GPIO_Pin = MYI2C_SDA_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MYI2C_GPIO_PORT, &gpio);
}

static void MyI2C_SetSdaInput(void)
{
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Pin = MYI2C_SDA_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MYI2C_GPIO_PORT, &gpio);
}

static void MyI2C_WriteScl(uint8_t level)
{
    if (level)
    {
        GPIO_SetBits(MYI2C_GPIO_PORT, MYI2C_SCL_PIN);
    }
    else
    {
        GPIO_ResetBits(MYI2C_GPIO_PORT, MYI2C_SCL_PIN);
    }
    MyI2C_Delay();
}

static void MyI2C_WriteSda(uint8_t level)
{
    if (level)
    {
        GPIO_SetBits(MYI2C_GPIO_PORT, MYI2C_SDA_PIN);
    }
    else
    {
        GPIO_ResetBits(MYI2C_GPIO_PORT, MYI2C_SDA_PIN);
    }
    MyI2C_Delay();
}

static uint8_t MyI2C_ReadSda(void)
{
    uint8_t level = GPIO_ReadInputDataBit(MYI2C_GPIO_PORT, MYI2C_SDA_PIN);
    MyI2C_Delay();
    return level;
}

void MyI2C_Init(void)
{
    RCC_APB2PeriphClockCmd(MYI2C_GPIO_CLOCK, ENABLE);

    GPIO_InitTypeDef gpio;
    gpio.GPIO_Mode = GPIO_Mode_Out_OD;
    gpio.GPIO_Pin = MYI2C_SCL_PIN | MYI2C_SDA_PIN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MYI2C_GPIO_PORT, &gpio);

    GPIO_SetBits(MYI2C_GPIO_PORT, MYI2C_SCL_PIN | MYI2C_SDA_PIN);
}

void MyI2C_Start(void)
{
    MyI2C_SetSdaOutput();
    MyI2C_WriteSda(1);
    MyI2C_WriteScl(1);
    MyI2C_WriteSda(0);
    MyI2C_WriteScl(0);
}

void MyI2C_Stop(void)
{
    MyI2C_SetSdaOutput();
    MyI2C_WriteSda(0);
    MyI2C_WriteScl(1);
    MyI2C_WriteSda(1);
}

void MyI2C_SendByte(uint8_t byte)
{
    uint8_t i;
    MyI2C_SetSdaOutput();
    for (i = 0; i < 8; i++)
    {
        MyI2C_WriteSda(byte & (0x80 >> i));
        MyI2C_WriteScl(1);
        MyI2C_WriteScl(0);
    }
}

uint8_t MyI2C_ReceiveByte(void)
{
    uint8_t i;
    uint8_t byte = 0;

    MyI2C_SetSdaInput();
    for (i = 0; i < 8; i++)
    {
        MyI2C_WriteScl(1);
        if (MyI2C_ReadSda())
        {
            byte |= (0x80 >> i);
        }
        MyI2C_WriteScl(0);
    }
    MyI2C_SetSdaOutput();
    return byte;
}

void MyI2C_SendAck(uint8_t ack_bit)
{
    MyI2C_SetSdaOutput();
    MyI2C_WriteSda(ack_bit);
    MyI2C_WriteScl(1);
    MyI2C_WriteScl(0);
}

uint8_t MyI2C_ReceiveAck(void)
{
    uint8_t ack_bit;

    MyI2C_SetSdaInput();
    MyI2C_WriteScl(1);
    ack_bit = MyI2C_ReadSda();
    MyI2C_WriteScl(0);
    MyI2C_SetSdaOutput();

    return ack_bit;
}

uint8_t MyI2C_ReadSclLine(void)
{
    return GPIO_ReadInputDataBit(MYI2C_GPIO_PORT, MYI2C_SCL_PIN);
}

uint8_t MyI2C_ReadSdaLine(void)
{
    return GPIO_ReadInputDataBit(MYI2C_GPIO_PORT, MYI2C_SDA_PIN);
}

uint8_t MyI2C_CheckDevice(uint8_t device_address)
{
    uint8_t ack;

    MyI2C_Start();
    MyI2C_SendByte(device_address);
    ack = MyI2C_ReceiveAck();
    MyI2C_Stop();

    return ack == 0;
}
