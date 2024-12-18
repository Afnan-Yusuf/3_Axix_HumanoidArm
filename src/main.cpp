#include <Arduino.h>
#include <MotorInit.h>

// motor_pulley_tooths = 16
// pinion_pulley_tooths = 40
// pinion_gear_tooth_count 20
// gear_tooth_count 42
#include <AccelStepper.h>

// Define pin connections
const int MOTOR1_STEP_PIN = 4;  // Motor 1 step pin
const int MOTOR1_DIR_PIN = 16;   // Motor 1 direction pin
const int MOTOR2_STEP_PIN = 15;  // Motor 2 step pin
const int MOTOR2_DIR_PIN = 2;   // Motor 2 direction pin

// Define motor parameters
const float STEPS_PER_REV = 200.0;  // Steps per revolution for NEMA 17
const float MICROSTEPS = 16.0;      // Microstepping setting on your driver
const float GEAR_RATIO = .1905;       // Change if using gear reduction

// Calculate total steps for one degree of rotation
const float STEPS_PER_DEGREE = (STEPS_PER_REV * MICROSTEPS * GEAR_RATIO) / 360.0;

// Create two instances of AccelStepper for the motors
AccelStepper motor1(AccelStepper::DRIVER, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN);
AccelStepper motor2(AccelStepper::DRIVER, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN);

// Variables to store current angles
float currentPitch = 0.0;
float currentRoll = 0.0;

void setup() {
  Serial.begin(115200);
  
  // Configure motor settings
  motor1.setMaxSpeed(1000);     // Steps per second
  motor1.setAcceleration(500);  // Steps per second per second
  motor2.setMaxSpeed(1000);
  motor2.setAcceleration(500);
  
  // Set initial position as zero
  motor1.setCurrentPosition(0);
  motor2.setCurrentPosition(0);
}

// Function to convert angles to motor steps for differential control
void calculateMotorSteps(float pitch, float roll, long &steps1, long &steps2) {
  // For differential mechanism:
  // Motor1 = pitch + roll
  // Motor2 = pitch - roll
  
  float motor1Angle = pitch + roll;
  float motor2Angle = pitch - roll;
  
  steps1 = (long)(motor1Angle * STEPS_PER_DEGREE);
  steps2 = (long)(motor2Angle * STEPS_PER_DEGREE);
}

// Function to move to specified pitch and roll angles
void moveToAngles(float pitch, float roll) {
  // Constrain angles to prevent mechanical damage
  pitch = constrain(pitch, 0.0, 360.0);
  roll = constrain(roll, 0.0, 360.0);
  
  long steps1, steps2;
  calculateMotorSteps(pitch, roll, steps1, steps2);
  
  // Set target positions for both motors
  motor1.moveTo(steps1);
  motor2.moveTo(steps2);
  
  // Move both motors simultaneously
  // while (motor1.distanceToGo() != 0 || motor2.distanceToGo() != 0) {
  //   motor1.run();
  //   motor2.run();
  // }
  
if (motor1.distanceToGo() != 0) {
    motor1.run();
  }
  if (motor2.distanceToGo() != 0) {
    motor2.run();
  }

  // Update current angles
  currentPitch = pitch;
  currentRoll = roll;
  
  // Print current position
  Serial.print("Current Position - Pitch: ");
  Serial.print(currentPitch);
  Serial.print("° Roll: ");
  Serial.println(currentRoll);
}

// Example movement sequence
void loop() {
  // Wait for serial commands or execute a demo sequence
  
    // Demo sequence
    // Move to various positions
    moveToAngles(30.0, 30.0);    // Pitch up
    
  
}