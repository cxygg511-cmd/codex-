#include "stm32f10x.h"                  // Device header
#include "PWM.h"

/* ============================================================
 * 麦轮 4 轮独立驱动（TB6612）
 *
 * 轮子 -> 方向脚(AIN1/AIN2) -> PWM 通道
 *   左前 FL : PA11 / PA12 -> TIM2_CH4 (PA3)
 *   左后 RL : PA6  / PA7  -> TIM2_CH2 (PA1)
 *   右前 FR : PA5  / PA4  -> TIM2_CH1 (PA0)
 *   右后 RR : PB5  / PB6  -> TIM2_CH3 (PA2)
 *
 * 注意：前/后轮的对应是按原 leftSpeed/rightSpeed 推测的。
 *       如果侧移变成斜着走，说明前后轮映射反了，
 *       交换 FL/RL（或 FR/RR）两行的引脚即可。
 * ============================================================ */

void Motor_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

	GPIO_InitTypeDef GPIO_Initstructure;
	GPIO_Initstructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Initstructure.GPIO_Pin=GPIO_Pin_4 | GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_11|GPIO_Pin_12;
	GPIO_Initstructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_Initstructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef GPIO_Initstructure1;
	GPIO_Initstructure1.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_Initstructure1.GPIO_Pin=GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_Initstructure1.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_Initstructure1);

	Pwm_Init();
}

/* 通用：设置单个电机的方向和速度
   PortA/PinA = 正转方向脚, PortB/PinB = 反转方向脚 */
static void Wheel_SetSpeed(GPIO_TypeDef* PortA, uint16_t PinA,
                           GPIO_TypeDef* PortB, uint16_t PinB,
                           uint8_t Channel, int8_t Speed)
{
	if(Speed > 0)
	{
		GPIO_SetBits(PortA, PinA);    // 正转
		GPIO_ResetBits(PortB, PinB);
	}
	else if(Speed < 0)
	{
		GPIO_ResetBits(PortA, PinA);  // 反转
		GPIO_SetBits(PortB, PinB);
	}
	else
	{
		GPIO_SetBits(PortA, PinA);    // 刹车：两脚同高
		GPIO_SetBits(PortB, PinB);
	}

	uint16_t duty = (Speed < 0) ? (uint16_t)(-Speed) : (uint16_t)Speed;
	switch(Channel)
	{
		case 1: Pwm_SetCompare1(duty); break;
		case 2: Pwm_SetCompare2(duty); break;
		case 3: Pwm_SetCompare3(duty); break;
		case 4: Pwm_SetCompare4(duty); break;
	}
}

void Wheel_FL_Speed(int8_t Speed) { Wheel_SetSpeed(GPIOA, GPIO_Pin_11, GPIOA, GPIO_Pin_12, 4, Speed); }
void Wheel_RL_Speed(int8_t Speed) { Wheel_SetSpeed(GPIOA, GPIO_Pin_6,  GPIOA, GPIO_Pin_7,  2, Speed); }
void Wheel_FR_Speed(int8_t Speed) { Wheel_SetSpeed(GPIOA, GPIO_Pin_5,  GPIOA, GPIO_Pin_4,  1, Speed); }
void Wheel_RR_Speed(int8_t Speed) { Wheel_SetSpeed(GPIOB, GPIO_Pin_5,  GPIOB, GPIO_Pin_6,  3, Speed); }
