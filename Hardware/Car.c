#include "stm32f10x.h"                  // Device header
#include "Motor.h"

#define CAR_SPEED   50    // 移动速度（0~100，PWM 占空比）

void Car_Init(void)
{
	Motor_Init();
}

/* 麦轮 X 形逆解：底盘运动 (vx 前进, vy 左移, omega 左转) -> 4 轮速度 */
static void Mecanum_Move(int8_t vx, int8_t vy, int8_t omega)
{
	int16_t fl =  vx + vy - omega;   // 左前
	int16_t fr =  vx - vy + omega;   // 右前
	int16_t rl =  vx - vy - omega;   // 左后
	int16_t rr =  vx + vy + omega;   // 右后

	Wheel_FL_Speed((int8_t)fl);
	Wheel_FR_Speed((int8_t)fr);
	Wheel_RL_Speed((int8_t)rl);
	Wheel_RR_Speed((int8_t)rr);
}

void Move_Forward(void)  { Mecanum_Move( CAR_SPEED,  0,  0); }
void Move_Backward(void) { Mecanum_Move(-CAR_SPEED,  0,  0); }
void Move_Left(void)     { Mecanum_Move( 0,  CAR_SPEED,  0); }
void Move_Right(void)    { Mecanum_Move( 0, -CAR_SPEED,  0); }
void Car_Stop(void)      { Mecanum_Move( 0,  0,  0); }
