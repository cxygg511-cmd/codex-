#include "RemoteControl.h"
#include "Car.h"
#include "Motor.h"
#include "Serial.h"
#include <string.h>

#define REMOTE_LINE_BUFFER_SIZE  48
#define REMOTE_TIMEOUT_MS        500

static char RxLine[REMOTE_LINE_BUFFER_SIZE];
static uint8_t RxLineLength = 0;
static uint16_t TimeSinceLastCommand = 0;

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

static void HandleCommand(char *line)
{
    char command = line[0];
    char *args = &line[1];
    int a;
    int b;
    int c;
    int d;

    if (strcmp(line, "STOP") == 0 || strcmp(line, "S") == 0)
    {
        Car_Stop();
        ReplyOk("STOP");
        return;
    }

    if (strcmp(line, "PING") == 0)
    {
        ReplyOk("PONG");
        return;
    }

    if (command == 'F' && ParseInt(&args, &a))
    {
        Car_Move(LimitCommandSpeed(a), 0, 0);
        ReplyOk("F");
        return;
    }

    if (command == 'B' && ParseInt(&args, &a))
    {
        Car_Move(-LimitCommandSpeed(a), 0, 0);
        ReplyOk("B");
        return;
    }

    if (command == 'M' && ParseInt(&args, &a) && ParseInt(&args, &b) && ParseInt(&args, &c))
    {
        Car_Move(LimitCommandSpeed(a), LimitCommandSpeed(b), LimitCommandSpeed(c));
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
    Car_Stop();
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
            Car_Stop();
        }
    }
}
