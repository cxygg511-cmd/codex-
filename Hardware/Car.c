#include "stm32f10x.h"                  // Device header
#include "Motor.h"

#define CAR_SPEED   50    // Move speed: -100~100 PWM duty

void Car_Init(void)
{
    Motor_Init();
}

static int8_t ClampSpeed(int16_t speed)
{
    if (speed > 100) return 100;
    if (speed < -100) return -100;
    return (int8_t)speed;
}

/*
 * Mecanum inverse kinematics.
 * vx    > 0: move forward
 * vy    > 0: move left
 * omega > 0: turn left
 */
void Car_Move(int8_t vx, int8_t vy, int8_t omega)
{
    int16_t fl =  vx + vy - omega;   // front left
    int16_t fr =  vx - vy + omega;   // front right
    int16_t rl =  vx - vy - omega;   // rear left
    int16_t rr =  vx + vy + omega;   // rear right

    Wheel_FL_Speed(ClampSpeed(fl));
    Wheel_FR_Speed(ClampSpeed(fr));
    Wheel_RL_Speed(ClampSpeed(rl));
    Wheel_RR_Speed(ClampSpeed(rr));
}

void Move_Forward(void)  { Car_Move( CAR_SPEED,  0,  0); }
void Move_Backward(void) { Car_Move(-CAR_SPEED,  0,  0); }
void Move_Left(void)     { Car_Move( 0,  CAR_SPEED,  0); }
void Move_Right(void)    { Car_Move( 0, -CAR_SPEED,  0); }
void Car_Stop(void)      { Car_Move( 0,  0,  0); }
