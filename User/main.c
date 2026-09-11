#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "Motor.h"
#include "Car.h"
#include "Serial.h"
#include "PWM.h"
//#include "Grayscale_Sensor.h"
//#include "XUN.h"
#include "Key.h"


int main(void)
{
    Motor_Init();
    //GRAY_Init();
    Serial_Init();
    //Track_PID_Init();

    /* 等LoRa模块就绪后发送 31 0A */
    Delay_ms(500);

    while(1){
				
				Serial_SendByte(0x31);
				Serial_SendByte(0x0A);
        //Track_Control();
        //if(s == 1)
        //{
        //    break;
        //}
        Delay_ms(5);
    }

}

