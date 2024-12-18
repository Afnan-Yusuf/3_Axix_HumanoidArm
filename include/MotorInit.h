#pragma once
#include <AccelStepper.h>
#include <MultiStepper.h>

#define LSa_DirPin 16
#define LSa_StepPin 4
#define LSb_DirPin 2
#define LSb_StepPin 15
#define motorInterfaceType 1
#define enablepin 17

long left_pitchpos = 0;
long left_rollpos = 0;

long pitchTarget = 0;
long rollTarget = 0;

const float stepsPerDegreePitch = 20; // Steps needed for 1 degree of pitch
const float stepsPerDegreeRoll = 200;  // Steps needed for 1 degree of roll


// Create a new instance of the AccelStepper class:
AccelStepper LSx = AccelStepper(motorInterfaceType, LSa_StepPin, LSa_DirPin);
AccelStepper LSy = AccelStepper(motorInterfaceType, LSb_StepPin, LSb_DirPin);

MultiStepper left_steppers;

void ArmInit()
{
  // Set the maximum speed and acceleration:
  LSx.setMaxSpeed(100);
  LSx.setAcceleration(500);

  LSy.setMaxSpeed(100);
  LSy.setAcceleration(500);
  pinMode(enablepin, OUTPUT);
  digitalWrite(enablepin, LOW);

  left_steppers.addStepper(LSy);
  left_steppers.addStepper(LSx);
  LSx.setCurrentPosition(0);
    LSy.setCurrentPosition(0);

    // Initialize targets to match the current physical positions
    pitchTarget = 0; // Assuming starting pitch is 0 degrees
    rollTarget = 0;  // Assuming starting roll is 0 degrees
}

void testmot()
{
  LSy.move(8000);
  LSx.move(8000);
}

void left_arm_pitch_roll(int pitch_pos, int roll_pos)
{
  long positions[2];
  long cuurent_Y = LSy.currentPosition();
  long cuurent_X = LSx.currentPosition();

  left_pitchpos = (cuurent_Y - cuurent_X) / 2;
  left_rollpos = (cuurent_X + cuurent_Y) / 2;

  long delta_pitch = left_pitchpos - pitch_pos;
  long delta_roll = left_pitchpos - roll_pos;

  positions[0] = -(delta_pitch + delta_roll);
  positions[1] = (-delta_pitch + delta_roll);

  Serial.print(positions[0]);
  Serial.print("\t");
  Serial.print(positions[1]);
  Serial.print("\t");
  Serial.print(delta_pitch);  
  Serial.print("\t");
  Serial.print(delta_roll);
  Serial.print("\t");
  Serial.print(left_pitchpos);
  Serial.print("\t");
  Serial.print(left_rollpos);
  Serial.println("\t");

  left_steppers.moveTo(positions);
  left_steppers.run();
}

void moveToPitchAndRoll(long pitchPosition, long rollPosition) {
    long positions[2];

    // Calculate the changes in pitch and roll
    long pitchDelta = pitchPosition - pitchTarget;  // Delta for pitch
    long rollDelta = rollPosition - rollTarget;     // Delta for roll

    // Calculate motor movements for differential control
    long motorAMove = pitchDelta + rollDelta;       // Movement for motor A
    long motorBMove = -pitchDelta + rollDelta;      // Movement for motor B

    // Calculate absolute positions for MultiStepper
    positions[0] = LSx.currentPosition() + motorAMove * stepsPerDegreePitch;
    positions[1] = LSy.currentPosition() + motorBMove * stepsPerDegreeRoll;

    // Command the motors to move
    left_steppers.moveTo(positions);

    // Run the steppers synchronously
    left_steppers.run();

    // Update the pitch and roll targets AFTER calculations
    pitchTarget = pitchPosition;
    rollTarget = rollPosition;

    // Debugging outputs
    Serial.print("Motor A Target: ");
    Serial.print(positions[0]);
    Serial.print("\tMotor B Target: ");
    Serial.print(positions[1]);
    Serial.print("\tPitch Delta: ");
    Serial.print(pitchDelta);
    Serial.print("\tRoll Delta: ");
    Serial.print(rollDelta);
    Serial.print("\tMotor A Move: ");
    Serial.print(motorAMove);
    Serial.print("\tMotor B Move: ");
    Serial.println(motorBMove);
}


