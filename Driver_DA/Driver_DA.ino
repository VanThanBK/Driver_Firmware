/*
  Name:		Driver_DA.ino
  Created:	5/9/2018 7:51:12 PM
  Author:	Than
*/

#include "MultiThread.h"

#define DRIVER_ADDRESS    1

#define COMMAND_PORT		Serial1

#define VELOCITY     0
#define POSITION     1

#define PWM1				PA2
#define PWM2				PA1

#define LED					PB12

#define PWM					PB1
#define DIR					PB0

#define CHANEL_A			PB15
#define CHANEL_B			PB14

//#define SIMPLE_MODE
//#define DEBUG
//#define MCU_TEST
//#define CONFIG
//#define TEST_SPEED
#define X4_ENCODER
//#define TEST_POSITION

#ifdef X4_ENCODER
#define RESOLUTION			537.6
#else
#define RESOLUTION      268.8
#endif

#define MAX_VELOCITY		468

#define MAX_SPEED			255

#define VELOCITY_SAMPLE_TIME_MS		5
#define POSITION_SAMPLE_TIME_MS		1

float Kp = 0.6;
float Ki = 0;
float Kd = 0.06;

float Kp2 = 5;//2;//0.8;
float Ki2 = 0;//1;//8;
float Kd2 = 0.06;//-0.009;//0.08;
float Offset2 = 0;

float P = 0;
float I = 0;
float D = 0;

int CountedPulse = 0;
float DesiredSpeed = 0;
float CurrentSpeed = 0;
float DesiredPosition = 0;
float CurrentPosition = 0;
float PWMValue = 0;
float LastPWMValue = 0;
float NotChangeCounter = 0;
uint8_t Direction;

float Error = 0;
float LastError = 0;
float LastLastError = 0;
float dError = 0;

uint8_t TurningMode = VELOCITY;

MultiThread BlinkScheduler;
MultiThread VelocityPIDScheduler;
MultiThread PositionPIDScheduler;
MultiThread ReceiveScheduler;

int LedState = 1;
char ControlPack[3];

void setup()
{
  COMMAND_PORT.begin(115200);

  InitIO();

#ifdef TEST_POSITION
  TurningMode = POSITION;
#endif
}

//-------------------- MAIN --------------------

void loop()
{
#ifndef MCU_TEST

#ifndef CONFIG

  ReadControlPack();
#else
  //

#endif

#ifdef TEST_SPEED
  AdjustSpeed(60);
#else
#ifdef TEST_POSITION
  AdjustPosition(100);
#else
  if (TurningMode == VELOCITY)
  {
    AdjustSpeed(DesiredSpeed);
  }
  else
  {
    AdjustPosition(DesiredPosition);
  }

#endif

#endif

  Blink();
#else

  COMMAND_PORT.println(millis());

#endif
}

//-----------------------------------------------

void InitIO()
{
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(LED, OUTPUT);

  pinMode(CHANEL_A, INPUT);
  pinMode(CHANEL_B, INPUT);

  digitalWrite(LED, 1);
#ifdef X4_ENCODER
  attachInterrupt(CHANEL_A, CountPulseA_x4, CHANGE);
  attachInterrupt(CHANEL_B, CountPulseB_x4, CHANGE);
#else
  attachInterrupt(CHANEL_A, CountPulseA_x2, FALLING);
  attachInterrupt(CHANEL_B, CountPulseB_x2, FALLING);
#endif
}

void CountPulseA_x4()
{
  if (digitalRead(CHANEL_A) == LOW)
  {
    if (digitalRead(CHANEL_B) == LOW)
    {
      CountedPulse++;
    }
    else
    {
      CountedPulse--;
    }
  }
  else
  {
    if (digitalRead(CHANEL_B) == HIGH)
    {
      CountedPulse++;
    }
    else
    {
      CountedPulse--;
    }
  }

#ifdef DEBUG

  COMMAND_PORT.println(CountedPulse);
#else
  if (DesiredSpeed == 0 && TurningMode == VELOCITY)
  {
    CountedPulse = 0;
  }
#endif
}

void CountPulseB_x4()
{
  if (digitalRead(CHANEL_B) == LOW)
  {
    if (digitalRead(CHANEL_A) == HIGH)
    {
      CountedPulse++;
    }
    else
    {
      CountedPulse--;
    }
  }
  else
  {
    if (digitalRead(CHANEL_A) == LOW)
    {
      CountedPulse++;
    }
    else
    {
      CountedPulse--;
    }
  }

#ifdef DEBUG

  COMMAND_PORT.println(CountedPulse);
#else
  if (DesiredSpeed == 0 && TurningMode == VELOCITY)
  {
    CountedPulse = 0;
  }
#endif
}

void CountPulseA_x2()
{
  if (digitalRead(CHANEL_B) == LOW)
  {
    CountedPulse++;
  }
  else
  {
    CountedPulse--;
  }

#ifdef DEBUG

  COMMAND_PORT.println(CountedPulse);
#else
  if (DesiredSpeed == 0 && TurningMode == VELOCITY)
  {
    CountedPulse = 0;
  }
#endif
}

void CountPulseB_x2()
{
  if (digitalRead(CHANEL_A) == HIGH)
  {
    CountedPulse++;
  }
  else
  {
    CountedPulse--;
  }

#ifdef DEBUG

  COMMAND_PORT.println(CountedPulse);
#else
  if (DesiredSpeed == 0 && TurningMode == VELOCITY)
  {
    CountedPulse = 0;
  }
#endif
}

void Blink()
{
  if (BlinkScheduler.isSchedule(MAX_SPEED - abs(DesiredSpeed)))
  {
    LedState = !LedState;

    if (DesiredSpeed == 0)
    {
      digitalWrite(LED, 1);
    }
    else
    {
      digitalWrite(LED, LedState);
    }
  }
}

void ReadControlPack()
{
  if (COMMAND_PORT.available() > 1)
  {

    COMMAND_PORT.readBytes(ControlPack, 3);
    ReceiveScheduler.Counter = millis();

    switch (ControlPack[0])
    {
      case 0:
        if (TurningMode == VELOCITY)
        {
          DesiredSpeed = ControlPack[1] * 100 + ControlPack[2];
        }
        else
        {
          DesiredPosition = ControlPack[1] * 100 + ControlPack[2];
        }
        break;

      case 1:
        if (TurningMode == VELOCITY)
        {
          DesiredSpeed = -(ControlPack[1] * 100 + ControlPack[2]);
        }
        else
        {
          DesiredPosition = -(ControlPack[1] * 100 + ControlPack[2]);
        }
        break;

      case 2:
        if (ControlPack[1] == 1 && ControlPack[2] == 1)
        {
          TurningMode = VELOCITY;
        }
        else if (ControlPack[1] == 2 && ControlPack[2] == 2)
        {
          TurningMode = POSITION;
        }
        CountedPulse = 0;
        DesiredPosition = 0;
        DesiredSpeed = 0;
        PWMValue = 0;
        I = 0;
        break;

      case 3:
        if (TurningMode == VELOCITY)
        {
          Kp = (float)ControlPack[1] + (float)ControlPack[2] / 100;
        }
        else
        {
          Kp2 = (float)ControlPack[1] + (float)ControlPack[2] / 100;
        }
        break;

      case 4:
        if (TurningMode == VELOCITY)
        {
          Ki = (float)ControlPack[1] + (float)ControlPack[2] / 100;
        }
        else
        {
          Ki2 = (float)ControlPack[1] + (float)ControlPack[2] / 100;
        }
        break;

      case 5:
        if (TurningMode == VELOCITY)
        {
          Kd = (float)ControlPack[1] + (float)ControlPack[2] / 100;
        }
        else
        {
          Kd2 = (float)ControlPack[1] + (float)ControlPack[2] / 100;
        }
        break;

      default:
        break;
    }
  }

}

void AdjustSpeed(float speed)
{
  RUN_EVERY(VelocityPIDScheduler, VELOCITY_SAMPLE_TIME_MS)

#ifndef SIMPLE_MODE

  if (TurningMode != VELOCITY)
    return;

  if (speed == 0)
  {
    PWMValue = 0;
    SetDesiredSpeed(PWMValue);
  }

  float desiredVel = abs(speed);
  float currenVel = abs((CountedPulse * 60 * MAX_SPEED) / (RESOLUTION * 0.1 * MAX_VELOCITY));

  CountedPulse = 0;

  Error = desiredVel - currenVel;
  dError = Error - 2 * LastError + LastLastError;
  //dError = Error - LastError;
  LastLastError = LastError;
  LastError = Error;

  //P = (Error - LastError) * Kp;
  P = (Error) * Kp;

  I += (Error * Ki) * ((float)VELOCITY_SAMPLE_TIME_MS / 1000);

  I = constrain(I, -100, 100);

  D = (dError * Kd) / ((float)VELOCITY_SAMPLE_TIME_MS / 1000);

  PWMValue = PWMValue + P + I + D;

  PWMValue = constrain(PWMValue, -255, 255);

  SetDesiredSpeed(PWMValue * (speed / abs(speed)));

  COMMAND_PORT.println(currenVel);

#ifdef TEST_SPEED
  COMMAND_PORT.print("c");
  COMMAND_PORT.println(currenVel);

  COMMAND_PORT.print("P");
  COMMAND_PORT.println(PWMValue);
#endif

#else

  SetDesiredSpeed(speed);

#endif

#ifdef CONFIG

  COMMAND_PORT.print("v");
  COMMAND_PORT.println(abs(PWMValue));

#endif
}

void AdjustPosition(int16_t position)
{
  RUN_EVERY(PositionPIDScheduler, POSITION_SAMPLE_TIME_MS)

  if (TurningMode != POSITION)
    return;

  float desiredPos = position;
  float currentPos = CountedPulse;

  Error = desiredPos - currentPos;
  dError = Error - LastError;
  LastError = Error;

  P = Error * Kp2;

  I += Error * Ki2 * ((float)POSITION_SAMPLE_TIME_MS / 1000);

  //I = constrain(I, -100, 100);

  D = (dError * Kd2) / ((float)POSITION_SAMPLE_TIME_MS / 1000);

  PWMValue = P + I + D;

  if (PWMValue > 0)
  {
    PWMValue += Offset2;
  }
  else if (PWMValue < 0)
  {
    PWMValue -= Offset2;
  }

  PWMValue = constrain(PWMValue, -255, 255);

  SetDesiredSpeed(PWMValue);

  COMMAND_PORT.println(currentPos);

#ifdef TEST_POSITION
  COMMAND_PORT.println(currentPos);
  //COMMAND_PORT.println(PWMValue);
#endif
}

void SetDesiredSpeed(int pwmValue)
{
  if (pwmValue > 0)
  {
    analogWrite(PWM1, abs(pwmValue));
    analogWrite(PWM2, 0);
  }
  else
  {
    analogWrite(PWM1, 0);
    analogWrite(PWM2, abs(pwmValue));
  }
}
