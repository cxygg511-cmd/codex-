#if 0  /* === TRACKING CODE DISABLED === */
#include "stm32f10x.h"
#include "Grayscale_Sensor.h"
#include "Motor.h"
#include "Car.h"
#include "PID.h"
#include "Serial.h"
#include "PWM.h"
#include "Delay.h"

/* ====== 循迹参数 ====== */
#define LEFT_BASE1      8
#define RIGHT_BASE1     5
#define KP             35.0f
#define MAX_OUT        50.0f

/* ====== 起止线检测 ====== */
#define ARM_DELAY        800     /* 上电后等~4秒再启用检测 */
#define LINE_THRESHOLD   80      /* 反色后>80算黑 */
#define LINE_COUNT       6       /* Six sensors are enough for the finish bar. */
#define STOP_HOLD        3       /* Confirm for about 15 ms before stopping. */
/* ======================== */

PID mypid;
uint16_t a = 0;
uint16_t b = 0;
uint16_t s = 0;
uint32_t count = 0; 
uint16_t LEFT_BASE =  LEFT_BASE1;
uint16_t RIGHT_BASE = RIGHT_BASE1;
float weight[8] = {
        -3.5f, -2.5f, -1.5f, -0.5f,
         0.5f,  1.5f,  2.5f,  3.5f
    };
 
 float    current_position;
 float    pid_output;
 int16_t  L, R;
		
void Track_PID_Init(void)
{
    PID_Init(&mypid, KP, 0.5f, MAX_OUT, 0.0f, -0.02f);
}



float Get_Position_Feedback(uint8_t *SensorData)
{
    static float last_position = 0.0f;
    
    float sum_weight_data = 0.0f;
    float sum_data = 0.0f;
    int i;

    for (i = 0; i < 8; i++)
    {
        SensorData[i] = 255 - SensorData[i];
        if (SensorData[i] < 10)
            SensorData[i] = 0;

        sum_weight_data += (float)SensorData[i] * weight[i];
        sum_data        += (float)SensorData[i];
				if(i == 7) a = SensorData[i];
				if(i == 0) b = SensorData[i];
    }

    if (sum_data <= 0.0f)
        return 0.0f;

    last_position = sum_weight_data / sum_data;
    return last_position;
}



void Track_Control(void)
{

   
	  uint8_t  Gray_Data[8] = {0};

    GRAY_Analog_ReadReg(Gray_Data);


    current_position = Get_Position_Feedback(Gray_Data);
		
		
	
    /* ---- PID循迹 ---- */
    pid_output = PID_Calculate(&mypid, current_position);
		

    L = (int16_t)(LEFT_BASE  - pid_output);
    R = (int16_t)(RIGHT_BASE + pid_output);

		if(a >= 200)
		{
				L += 25;
				R -= 12;
		}
		
    if (L >  100) L =  100;
    if (L < -100) L = -100;
    if (R >  100) R =  50;
    if (R < -100) R = -100;
		
    leftSpeed((int8_t)L);
    rightSpeed((int8_t)R);
		count++;
		if(count == 3000)
		{
			LEFT_BASE += 5;
			RIGHT_BASE += 5;
			count = 5001;
		}
		if(count == 10500)
		{
			LEFT_BASE += 10;
			RIGHT_BASE += 10;
		}
		if(a >= 230 && count >= 12100)
		{
				Car_Stop();
			  s = 1;
		}
		
}




#endif  /* === END TRACKING === */
