#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "Motor.h"
#include "Car.h"
#include "Serial.h"
#include "PWM.h"
#include "RemoteControl.h"
#include "IMU.h"
//#include "Grayscale_Sensor.h"
//#include "XUN.h"
#include "Key.h"

#define MAIN_LOOP_PERIOD_MS  5

int main(void)
{
    Motor_Init();
    //GRAY_Init();
    Serial_Init();
    RemoteControl_Init();
    IMU_Init();
    //Track_PID_Init();

    while (1)
    {
        RemoteControl_Update(MAIN_LOOP_PERIOD_MS);
        IMU_Update();
        Car_Update();
        //Track_Control();
        //if(s == 1)
        //{
        //    break;
        //}
        Delay_ms(MAIN_LOOP_PERIOD_MS);
    }
}

