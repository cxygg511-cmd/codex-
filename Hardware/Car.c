#include "stm32f10x.h"                  // Device header
#include "Motor.h"

#define CAR_SPEED      50    // Move speed: -100~100 PWM duty
#define CAR_RAMP_STEP  2     // Max wheel speed change per Car_Update() call

/* Wheel output calibration. Reduce a value if that wheel is too fast. */
#define CAR_FL_FORWARD_SCALE   1.00f
#define CAR_FR_FORWARD_SCALE   0.95f
#define CAR_RL_FORWARD_SCALE   1.00f
#define CAR_RR_FORWARD_SCALE   0.95f

#define CAR_FL_BACKWARD_SCALE  0.90f
#define CAR_FR_BACKWARD_SCALE  0.95f
#define CAR_RL_BACKWARD_SCALE  0.90f
#define CAR_RR_BACKWARD_SCALE  0.95f

void Car_Stop(void);
void Car_BrakeNow(void);

static int8_t Target_FL = 0;
static int8_t Target_FR = 0;
static int8_t Target_RL = 0;
static int8_t Target_RR = 0;
static int8_t Current_FL = 0;
static int8_t Current_FR = 0;
static int8_t Current_RL = 0;
static int8_t Current_RR = 0;

static int8_t ClampSpeed(int16_t speed)
{
    if (speed > 100) return 100;
    if (speed < -100) return -100;
    return (int8_t)speed;
}

static int8_t RampToward(int8_t current, int8_t target)
{
    if (current < target)
    {
        current += CAR_RAMP_STEP;
        if (current > target)
        {
            current = target;
        }
    }
    else if (current > target)
    {
        current -= CAR_RAMP_STEP;
        if (current < target)
        {
            current = target;
        }
    }

    return current;
}

static int8_t ApplyScale(int8_t speed, float forward_scale, float backward_scale)
{
    float scale = speed >= 0 ? forward_scale : backward_scale;
    float scaled = speed * scale;

    if (scaled >= 0.0f)
    {
        return ClampSpeed((int16_t)(scaled + 0.5f));
    }
    return ClampSpeed((int16_t)(scaled - 0.5f));
}

static void ApplyWheelSpeeds(void)
{
    Wheel_FL_Speed(ApplyScale(Current_FL, CAR_FL_FORWARD_SCALE, CAR_FL_BACKWARD_SCALE));
    Wheel_FR_Speed(ApplyScale(Current_FR, CAR_FR_FORWARD_SCALE, CAR_FR_BACKWARD_SCALE));
    Wheel_RL_Speed(ApplyScale(Current_RL, CAR_RL_FORWARD_SCALE, CAR_RL_BACKWARD_SCALE));
    Wheel_RR_Speed(ApplyScale(Current_RR, CAR_RR_FORWARD_SCALE, CAR_RR_BACKWARD_SCALE));
}

void Car_Init(void)
{
    Motor_Init();
    Car_BrakeNow();
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

    Target_FL = ClampSpeed(fl);
    Target_FR = ClampSpeed(fr);
    Target_RL = ClampSpeed(rl);
    Target_RR = ClampSpeed(rr);
}

void Car_Update(void)
{
    Current_FL = RampToward(Current_FL, Target_FL);
    Current_FR = RampToward(Current_FR, Target_FR);
    Current_RL = RampToward(Current_RL, Target_RL);
    Current_RR = RampToward(Current_RR, Target_RR);
    ApplyWheelSpeeds();
}

void Car_Stop(void)
{
    Target_FL = 0;
    Target_FR = 0;
    Target_RL = 0;
    Target_RR = 0;
}

void Car_BrakeNow(void)
{
    Target_FL = 0;
    Target_FR = 0;
    Target_RL = 0;
    Target_RR = 0;
    Current_FL = 0;
    Current_FR = 0;
    Current_RL = 0;
    Current_RR = 0;
    ApplyWheelSpeeds();
}

void Move_Forward(void)  { Car_Move( CAR_SPEED,  0,  0); }
void Move_Backward(void) { Car_Move(-CAR_SPEED,  0,  0); }
void Move_Left(void)     { Car_Move( 0,  CAR_SPEED,  0); }
void Move_Right(void)    { Car_Move( 0, -CAR_SPEED,  0); }
