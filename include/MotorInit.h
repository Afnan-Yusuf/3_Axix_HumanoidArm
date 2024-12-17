#pragma once
#include <AccelStepper.h>

#define LSa_DirPin 16
#define LSa_StepPin 4
#define LSb_DirPin 2
#define LSb_StepPin 15
#define motorInterfaceType 1
#define enablepin 17

long left_pitchpos = 0;
long left_rollpos = 0;

// Create a new instance of the AccelStepper class:
AccelStepper LSx = AccelStepper(motorInterfaceType, LSa_StepPin, LSa_DirPin);
AccelStepper LSy = AccelStepper(motorInterfaceType, LSb_StepPin, LSb_DirPin);

void ArmInit() {
  // Set the maximum speed and acceleration:
  LSx.setMaxSpeed(100);
  LSx.setAcceleration(500);

  LSy.setMaxSpeed(100);
  LSy.setAcceleration(500);
  pinMode(enablepin, OUTPUT);
  digitalWrite(enablepin, LOW);
}


void testmot() {
  LSy.move(8000);
  LSx.move(8000);
}



void left_arm_pitch_roll(int pitch_pos, int roll_pos)
{
    long cuurent_Y = LSy.currentPosition();
    long cuurent_X = LSx.currentPosition();

    left_pitchpos = (cuurent_Y - cuurent_X) /2;
    left_rollpos = (cuurent_X + cuurent_Y) /2;

    long delta_pitch = pitch_pos - left_pitchpos;   
    long delta_roll = roll_pos - left_rollpos;
    
    
    long LSx_delta_steps = delta_pitch + delta_roll;
    long LSy_delta_steps = delta_roll - delta_pitch;
    Serial.print(left_pitchpos);
    Serial.print("\t");
    Serial.print(left_rollpos);
    Serial.print("\t");
    Serial.print(delta_pitch);
    Serial.print("\t");
    Serial.print(delta_roll);
    Serial.print("\t");
    Serial.print(LSx_delta_steps);
    Serial.print("\t");
    Serial.print(LSy_delta_steps);
    Serial.print("\t");
    Serial.print(LSx.currentPosition());
    Serial.print("\t");
    Serial.println(LSy.currentPosition());

    LSx.moveTo(LSx_delta_steps);
    LSy.moveTo(LSy_delta_steps);
    
}
