#if 0  /* === TRACKING CODE DISABLED === */
#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "Serial.h"

uint16_t gray_values[8] = {0};

void GraySensor_Init(void)
{

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB, GPIO_Pin_15);
	
}

void GraySensor_ReadAllValues(uint16_t* values)
{
    GPIO_SetBits(GPIOB, GPIO_Pin_15);
    Delay_us(5);
    
    for(int i = 0; i < 8; i++) {
        GPIO_ResetBits(GPIOB, GPIO_Pin_15);
        Delay_us(10);
        
        if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14) == Bit_SET) {
            values[i] = 1;
        } else {
            values[i] = 0;
        }
        
        GPIO_SetBits(GPIOB, GPIO_Pin_15);
        Delay_us(10);
    }

}

float GraySensor_GetPosition(void)
{
	uint16_t sensor_values[8];
	float weighted_sum = 0.0f;
	 float sum = 0.0f;  
	
	GraySensor_ReadAllValues(sensor_values);
	
	Serial_SendString("Sensors: ");
    for(int i = 0; i < 8; i++) {
        Serial_SendNumber(sensor_values[i], 1);
        Serial_SendString(" ");
        gray_values[i] = sensor_values[i];  // 保存到全局变量用于调试
    }
	
	
	 for(int i = 0; i < 8; i++) {
        weighted_sum += sensor_values[i] * (i - 3.5f);
        sum += sensor_values[i];

    }
    
		if(sum>0){
		return weighted_sum/sum;
		}
		else{
		return 0.0f;
		}


}
	
	





#endif  /* === END TRACKING === */
