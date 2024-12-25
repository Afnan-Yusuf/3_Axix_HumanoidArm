#pragma once
#include <AccelStepper.h>
#include <ESP32Servo.h>

#define left_elbow_servopin 12
#define right_elbow_servopin 13

const int MOTOR1_STEP_PIN = 4;   // Motor 1 step pin
const int MOTOR1_DIR_PIN = 16;   // Motor 1 direction pin
const int MOTOR2_STEP_PIN = 15;  // Motor 2 step pin
const int MOTOR2_DIR_PIN = 2;    // Motor 2 direction pin

// Define motor parameters
const float STEPS_PER_REV = 200.0;  // Steps per revolution for NEMA 17
const float MICROSTEPS = 4.0;       // Microstepping setting on your driver
const float GEAR_RATIO = 5.25;      // Change if using gear reduction

const float MAX_SPEED = 4000;      // Maximum steps per second
const float MAX_ACCELERATION = 4000; // Maximum acceleration

const float STEPS_PER_DEGREE = (STEPS_PER_REV * MICROSTEPS * GEAR_RATIO) / 360.0;

AccelStepper motor1(AccelStepper::DRIVER, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN);
AccelStepper motor2(AccelStepper::DRIVER, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN);

// Variables to store current angles and target angles
float currentPitch = 0.0;
float currentRoll = 0.0;
float targetPitch = 0.0;
float targetRoll = 0.0;

// Flag to track if movement is in progress
bool isMoving = false;


Servo left_elbow_servo;
Servo right_elbow_servo;

void ArmInit()
{
  
  motor1.setMaxSpeed(4000);
  motor1.setAcceleration(4000);  
  motor2.setMaxSpeed(4000);
  motor2.setAcceleration(4000);
  
  motor1.setCurrentPosition(0);
  motor2.setCurrentPosition(0);
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	left_elbow_servo.setPeriodHertz(50); 
  right_elbow_servo.setPeriodHertz(50);  

	left_elbow_servo.attach(left_elbow_servopin, 1000, 2000);
  right_elbow_servo.attach(right_elbow_servopin, 1000, 2000);
}

void calculateMotorSteps(float pitch, float roll, long &steps1, long &steps2) {
  float motor1Angle = pitch + roll;
  float motor2Angle = pitch - roll;
  
  steps1 = (long)(motor1Angle * STEPS_PER_DEGREE);
  steps2 = (long)(motor2Angle * STEPS_PER_DEGREE);
}

// Function to start movement to specified pitch and roll angles
void startMove(float pitch, float roll) {
  pitch = constrain(pitch, -20.0, 20.0);
  roll = constrain(roll, 0.0, 360.0);
  
  long steps1, steps2;
  calculateMotorSteps(pitch, roll, steps1, steps2);
  
  long distance1 = abs(steps1 - motor1.currentPosition());
  long distance2 = abs(steps2 - motor2.currentPosition());
  
  long maxDistance = max(distance1, distance2);
  
  float speedRatio1 = distance1 > 0 ? (float)distance1 / maxDistance : 1.0;
  float speedRatio2 = distance2 > 0 ? (float)distance2 / maxDistance : 1.0;
  
  motor1.setMaxSpeed(MAX_SPEED * speedRatio1);
  motor2.setMaxSpeed(MAX_SPEED * speedRatio2);
  
  motor1.setAcceleration(MAX_ACCELERATION * speedRatio1);
  motor2.setAcceleration(MAX_ACCELERATION * speedRatio2);
  
  motor1.moveTo(steps1);
  motor2.moveTo(steps2);
  
  targetPitch = pitch;
  targetRoll = roll;
  
  isMoving = true;
  
  Serial.println("Starting synchronized move:");
  Serial.printf("Distance 1: %ld, Speed 1: %.2f\n", distance1, MAX_SPEED * speedRatio1);
  Serial.printf("Distance 2: %ld, Speed 2: %.2f\n", distance2, MAX_SPEED * speedRatio2);
}

void updateMovement() {
  if (isMoving) {
    bool motor1Running = motor1.run();
    bool motor2Running = motor2.run();
    
    if (!motor1Running && !motor2Running && 
        motor1.distanceToGo() == 0 && motor2.distanceToGo() == 0) {
      isMoving = false;
      currentPitch = targetPitch;
      currentRoll = targetRoll;
      
      Serial.print("Movement complete - Pitch: ");
      Serial.print(currentPitch);
      Serial.print("° Roll: ");
      Serial.println(currentRoll);
    }
  }
}

bool isMovementInProgress() {
  return isMoving;
}