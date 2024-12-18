#pragma once
#include <AccelStepper.h>
#include <ESP32Servo.h>

#define left_elbow_servopin 12
#define right_elbow_servopin 13

// Define pin connections
const int MOTOR1_STEP_PIN = 4;   // Motor 1 step pin
const int MOTOR1_DIR_PIN = 16;   // Motor 1 direction pin
const int MOTOR2_STEP_PIN = 15;  // Motor 2 step pin
const int MOTOR2_DIR_PIN = 2;    // Motor 2 direction pin

// Define motor parameters
const float STEPS_PER_REV = 200.0;  // Steps per revolution for NEMA 17
const float MICROSTEPS = 4.0;       // Microstepping setting on your driver
const float GEAR_RATIO = 5.25;      // Change if using gear reduction

// Maximum speeds and acceleration
const float MAX_SPEED = 4000;      // Maximum steps per second
const float MAX_ACCELERATION = 4000; // Maximum acceleration

// Calculate total steps for one degree of rotation
const float STEPS_PER_DEGREE = (STEPS_PER_REV * MICROSTEPS * GEAR_RATIO) / 360.0;

// Create two instances of AccelStepper for the motors
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
  // Configure motor settings
  motor1.setMaxSpeed(4000);      // Steps per second
  motor1.setAcceleration(4000);  // Steps per second per second
  motor2.setMaxSpeed(4000);
  motor2.setAcceleration(4000);
  
  // Set initial position as zero
  motor1.setCurrentPosition(0);
  motor2.setCurrentPosition(0);
  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
	left_elbow_servo.setPeriodHertz(50); 
  right_elbow_servo.setPeriodHertz(50);   // standard 50 hz servo
	left_elbow_servo.attach(left_elbow_servopin, 1000, 2000);
  right_elbow_servo.attach(right_elbow_servopin, 1000, 2000);
}


void calculateMotorSteps(float pitch, float roll, long &steps1, long &steps2) {
  // For differential mechanism:
  // Motor1 = pitch + roll
  // Motor2 = pitch - roll
  
  float motor1Angle = pitch + roll;
  float motor2Angle = pitch - roll;
  
  steps1 = (long)(motor1Angle * STEPS_PER_DEGREE);
  steps2 = (long)(motor2Angle * STEPS_PER_DEGREE);
}

// Function to start movement to specified pitch and roll angles
void startMove(float pitch, float roll) {
  // Constrain angles to prevent mechanical damage
  pitch = constrain(pitch, 0.0, 360.0);
  roll = constrain(roll, 0.0, 360.0);
  
  long steps1, steps2;
  calculateMotorSteps(pitch, roll, steps1, steps2);
  
  // Calculate the absolute distances to move
  long distance1 = abs(steps1 - motor1.currentPosition());
  long distance2 = abs(steps2 - motor2.currentPosition());
  
  // Find the longer distance
  long maxDistance = max(distance1, distance2);
  
  // Calculate speed ratios to synchronize movement completion
  float speedRatio1 = distance1 > 0 ? (float)distance1 / maxDistance : 1.0;
  float speedRatio2 = distance2 > 0 ? (float)distance2 / maxDistance : 1.0;
  
  // Set speeds proportionally to maintain synchronization
  motor1.setMaxSpeed(MAX_SPEED * speedRatio1);
  motor2.setMaxSpeed(MAX_SPEED * speedRatio2);
  
  // Set accelerations proportionally
  motor1.setAcceleration(MAX_ACCELERATION * speedRatio1);
  motor2.setAcceleration(MAX_ACCELERATION * speedRatio2);
  
  // Set target positions
  motor1.moveTo(steps1);
  motor2.moveTo(steps2);
  
  // Set target angles
  targetPitch = pitch;
  targetRoll = roll;
  
  // Set moving flag
  isMoving = true;
  
  // Debug output
  Serial.println("Starting synchronized move:");
  Serial.printf("Distance 1: %ld, Speed 1: %.2f\n", distance1, MAX_SPEED * speedRatio1);
  Serial.printf("Distance 2: %ld, Speed 2: %.2f\n", distance2, MAX_SPEED * speedRatio2);
}

// Function to update motor movement (non-blocking)
void updateMovement() {
  if (isMoving) {
    // Run both motors
    bool motor1Running = motor1.run();
    bool motor2Running = motor2.run();
    
    // Check if movement is complete
    if (!motor1Running && !motor2Running && 
        motor1.distanceToGo() == 0 && motor2.distanceToGo() == 0) {
      // Movement complete
      isMoving = false;
      currentPitch = targetPitch;
      currentRoll = targetRoll;
      
      // Print current position
      Serial.print("Movement complete - Pitch: ");
      Serial.print(currentPitch);
      Serial.print("° Roll: ");
      Serial.println(currentRoll);
    }
  }
}

// Function to check if movement is in progress
bool isMovementInProgress() {
  return isMoving;
}