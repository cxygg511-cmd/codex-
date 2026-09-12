#include "RemoteControl.h"
#include "Car.h"
#include "IMU.h"
#include "Motor.h"
#include "MPU6050.h"
#include "MyI2C.h"
#include "Serial.h"
#include <string.h>

#define REMOTE_LINE_BUFFER_SIZE  48
#define REMOTE_TIMEOUT_MS        500
#define TELEMETRY_MIN_PERIOD_MS  20
#define HEADING_HOLD_KP          1.2f
#define HEADING_HOLD_KD          0.25f
#define HEADING_HOLD_MAX_OMEGA   25

static char RxLine[REMOTE_LINE_BUFFER_SIZE];
static uint8_t RxLineLength = 0;
static uint16_t TimeSinceLastCommand = 0;
static uint16_t TelemetryPeriodMs = 0;
static uint16_t TelemetryElapsedMs = 0;
static uint8_t HeadingHoldEnabled = 0;
static uint8_t HeadingHoldActive = 0;
static float HeadingTargetYaw = 0.0f;
static int8_t CommandVx = 0;
static int8_t CommandVy = 0;
static int8_t CommandOmega = 0;

static int8_t LimitCommandSpeed(int value)
{
    if (value > 100) return 100;
    if (value < -100) return -100;
    return (int8_t)value;
}

static void SkipSpaces(char **text)
{
    while (**text == ' ' || **text == '\t')
    {
        (*text)++;
    }
}

static char *NormalizeLine(char *line)
{
    char *start = line;
    char *end;

    while (*start == ' ' || *start == '\t' || *start == '\0')
    {
        start++;
    }

    end = start + strlen(start);
    while (end > start && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\0'))
    {
        end--;
    }
    *end = '\0';

    return start;
}
static uint8_t ParseInt(char **text, int *value)
{
    int sign = 1;
    int result = 0;
    uint8_t hasDigit = 0;

    SkipSpaces(text);

    if (**text == '-')
    {
        sign = -1;
        (*text)++;
    }
    else if (**text == '+')
    {
        (*text)++;
    }

    while (**text >= '0' && **text <= '9')
    {
        hasDigit = 1;
        result = result * 10 + (**text - '0');
        (*text)++;
    }

    if (!hasDigit)
    {
        return 0;
    }

    *value = result * sign;
    return 1;
}

static float WrapAngleError(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }

    while (angle < -180.0f)
    {
        angle += 360.0f;
    }

    return angle;
}

static int8_t ClampCorrection(float value)
{
    if (value > HEADING_HOLD_MAX_OMEGA)
    {
        return HEADING_HOLD_MAX_OMEGA;
    }

    if (value < -HEADING_HOLD_MAX_OMEGA)
    {
        return -HEADING_HOLD_MAX_OMEGA;
    }

    if (value >= 0.0f)
    {
        return (int8_t)(value + 0.5f);
    }
    return (int8_t)(value - 0.5f);
}

static uint8_t IsLinearMoveCommand(void)
{
    return (CommandVx != 0 || CommandVy != 0) && CommandOmega == 0;
}

static void ApplyMotionCommand(void)
{
    int8_t omega;
    float error;
    float correction;

    omega = CommandOmega;
    if (HeadingHoldEnabled && IsLinearMoveCommand() && IMU_IsReady())
    {
        if (!HeadingHoldActive)
        {
            HeadingTargetYaw = IMU_GetYaw();
            HeadingHoldActive = 1;
        }

        error = WrapAngleError(HeadingTargetYaw - IMU_GetYaw());
        correction = error * HEADING_HOLD_KP - IMU_GetGyroZ() * HEADING_HOLD_KD;
        omega = ClampCorrection(correction);
    }
    else
    {
        HeadingHoldActive = 0;
    }

    Car_Move(CommandVx, CommandVy, omega);
}

static void SetMotionCommand(int8_t vx, int8_t vy, int8_t omega)
{
    CommandVx = vx;
    CommandVy = vy;
    CommandOmega = omega;
    ApplyMotionCommand();
}
static int16_t FloatToCent(float value)
{
    if (value >= 0.0f)
    {
        return (int16_t)(value * 100.0f + 0.5f);
    }
    return (int16_t)(value * 100.0f - 0.5f);
}

static void SetWheelSpeeds(int fl, int fr, int rl, int rr)
{
    Wheel_FL_Speed(LimitCommandSpeed(fl));
    Wheel_FR_Speed(LimitCommandSpeed(fr));
    Wheel_RL_Speed(LimitCommandSpeed(rl));
    Wheel_RR_Speed(LimitCommandSpeed(rr));
}

static void ReplyOk(char *command)
{
    Serial_SendString("OK ");
    Serial_SendString(command);
    Serial_SendString("\r\n");
}

static void SendTelemetry(void)
{
    Serial_Printf("DATA %d %d %d %d %d %d %d %d %d\r\n",
                  IMU_IsReady(),
                  FloatToCent(IMU_GetYaw()),
                  FloatToCent(IMU_GetGyroZ()),
                  g_imu_raw.acc_x,
                  g_imu_raw.acc_y,
                  g_imu_raw.acc_z,
                  g_imu_raw.gyro_x,
                  g_imu_raw.gyro_y,
                  g_imu_raw.gyro_z);
}

static void HandleStreamCommand(char *line)
{
    char *args = &line[6];
    int period_ms;

    if (!ParseInt(&args, &period_ms))
    {
        Serial_SendString("ERR STREAM_PERIOD\r\n");
        return;
    }

    if (period_ms <= 0)
    {
        TelemetryPeriodMs = 0;
        TelemetryElapsedMs = 0;
        ReplyOk("STREAM_OFF");
        return;
    }

    if (period_ms < TELEMETRY_MIN_PERIOD_MS)
    {
        period_ms = TELEMETRY_MIN_PERIOD_MS;
    }

    TelemetryPeriodMs = (uint16_t)period_ms;
    TelemetryElapsedMs = 0;
    ReplyOk("STREAM");
}

static void HandleHoldCommand(char *line)
{
    char *args = &line[4];
    int enabled;

    if (!ParseInt(&args, &enabled))
    {
        Serial_Printf("HOLD %d\r\n", HeadingHoldEnabled);
        return;
    }

    HeadingHoldEnabled = enabled ? 1 : 0;
    HeadingHoldActive = 0;
    ReplyOk(HeadingHoldEnabled ? "HOLD_ON" : "HOLD_OFF");
}
static void HandleCommand(char *line)
{
    char command;
    char *args;
    int a;
    int b;
    int c;
    int d;

    line = NormalizeLine(line);
    command = line[0];
    args = &line[1];

    if (strcmp(line, "STOP") == 0 || strcmp(line, "S") == 0)
    {
        SetMotionCommand(0, 0, 0);
        ReplyOk("STOP");
        return;
    }

    if (strcmp(line, "PING") == 0 || strcmp(line, "?") == 0)
    {
        ReplyOk("PONG");
        return;
    }


    if (strcmp(line, "YAW0") == 0)
    {
        IMU_ResetYaw();
        HeadingTargetYaw = 0.0f;
        HeadingHoldActive = 0;
        ReplyOk("YAW0");
        return;
    }

    if (strncmp(line, "HOLD", 4) == 0)
    {
        HandleHoldCommand(line);
        return;
    }

    if (strcmp(line, "MPUID") == 0 || strcmp(line, "U") == 0)
    {
        Serial_Printf("MPUID %d\r\n", MPU6050_GetID());
        return;
    }

    if (strcmp(line, "I2CDBG") == 0)
    {
        MyI2C_Init();
        Serial_Printf("I2CDBG SCL %d SDA %d ACK68 %d ACK69 %d\r\n",
                      MyI2C_ReadSclLine(),
                      MyI2C_ReadSdaLine(),
                      MyI2C_CheckDevice(0xD0),
                      MyI2C_CheckDevice(0xD2));
        return;
    }
    if (strcmp(line, "DATA") == 0 || strcmp(line, "STATUS") == 0 || strcmp(line, "D") == 0)
    {
        SendTelemetry();
        return;
    }

    if (strncmp(line, "STREAM", 6) == 0)
    {
        HandleStreamCommand(line);
        return;
    }

    if (command == 'F' && ParseInt(&args, &a))
    {
        SetMotionCommand(LimitCommandSpeed(a), 0, 0);
        ReplyOk("F");
        return;
    }

    if (command == 'B' && ParseInt(&args, &a))
    {
        SetMotionCommand(-LimitCommandSpeed(a), 0, 0);
        ReplyOk("B");
        return;
    }

    if (command == 'M' && ParseInt(&args, &a) && ParseInt(&args, &b) && ParseInt(&args, &c))
    {
        SetMotionCommand(LimitCommandSpeed(a), LimitCommandSpeed(b), LimitCommandSpeed(c));
        ReplyOk("M");
        return;
    }

    if (command == 'W' && ParseInt(&args, &a) && ParseInt(&args, &b) && ParseInt(&args, &c) && ParseInt(&args, &d))
    {
        SetWheelSpeeds(a, b, c, d);
        ReplyOk("W");
        return;
    }

    Serial_SendString("ERR UNKNOWN\r\n");
}

static void PushReceivedByte(uint8_t byte)
{
    if (byte == 0)
    {
        return;
    }

    if (byte == '\n' || byte == '\r')
    {
        if (RxLineLength > 0)
        {
            RxLine[RxLineLength] = '\0';
            HandleCommand(RxLine);
            RxLineLength = 0;
            TimeSinceLastCommand = 0;
        }
        return;
    }

    if (RxLineLength < REMOTE_LINE_BUFFER_SIZE - 1)
    {
        RxLine[RxLineLength++] = (char)byte;
    }
    else
    {
        RxLineLength = 0;
        Serial_SendString("ERR LINE_TOO_LONG\r\n");
    }
}

void RemoteControl_Init(void)
{
    RxLineLength = 0;
    TimeSinceLastCommand = 0;
    TelemetryPeriodMs = 0;
    TelemetryElapsedMs = 0;
    HeadingHoldEnabled = 0;
    HeadingHoldActive = 0;
    HeadingTargetYaw = 0.0f;
    SetMotionCommand(0, 0, 0);
    Serial_SendString("READY\r\n");
}

void RemoteControl_Update(uint16_t elapsed_ms)
{
    uint8_t byte;

    while (Serial_ReadByte(&byte))
    {
        PushReceivedByte(byte);
    }

    if (TimeSinceLastCommand < REMOTE_TIMEOUT_MS)
    {
        TimeSinceLastCommand += elapsed_ms;
        if (TimeSinceLastCommand >= REMOTE_TIMEOUT_MS)
        {
            SetMotionCommand(0, 0, 0);
        }
    }

    if (TimeSinceLastCommand < REMOTE_TIMEOUT_MS && HeadingHoldEnabled && IsLinearMoveCommand())
    {
        ApplyMotionCommand();
    }

    if (TelemetryPeriodMs > 0)
    {
        TelemetryElapsedMs += elapsed_ms;
        if (TelemetryElapsedMs >= TelemetryPeriodMs)
        {
            TelemetryElapsedMs = 0;
            SendTelemetry();
        }
    }
}
