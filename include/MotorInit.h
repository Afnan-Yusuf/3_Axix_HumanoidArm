#pragma once
#include <AccelStepper.h>

#define LSx_direction_pin 13
#define LSx_step_pin 12
#define RSx_direction_pin 11
#define RSx_step_pin 10

#define LSy_direction_pin 14
#define LSy_step_pin 15
#define RSy_direction_pin 18
#define RSy_step_pin 19


#define ENA_pin 9

const int stepsPerRevolution = 200;

AccelStepper LSx(1, LSx_step_pin, LSx_direction_pin);
AccelStepper RSx(1, RSx_step_pin, RSx_direction_pin);
AccelStepper LSy(1, LSy_step_pin, LSy_direction_pin);
AccelStepper RSy(1, RSy_step_pin, RSy_direction_pin);


void ArmInit()
{
    LSx.setMaxSpeed(1000);
    RSx.setMaxSpeed(1000);
    LSy.setMaxSpeed(1000);
    RSy.setMaxSpeed(1000);
    pinMode(ENA_pin, OUTPUT);
    digitalWrite(ENA_pin, HIGH);
}


