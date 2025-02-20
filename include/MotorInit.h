#pragma once
#include <AccelStepper.h>
#include <ESP32Servo.h>

// Add limit switch pin definitions
#define LEFT_MOTOR1_LIMIT_PIN 34  // Adjust pins as needed
#define LEFT_MOTOR2_LIMIT_PIN 35
#define RIGHT_MOTOR1_LIMIT_PIN 36 
#define RIGHT_MOTOR2_LIMIT_PIN 39

// Define homing parameters
#define HOMING_SPEED_RATIO 0.3f      // 30% of max speed
#define HOMING_ACCEL_RATIO 0.5f      // 50% of max acceleration
#define HOMING_BACKOFF_STEPS 100     // Steps to back off after hitting limit
#define HOMING_TIMEOUT_STEPS 50000   // Maximum steps before timeout

class DifferentialWrist {
private:
    AccelStepper motor1;
    AccelStepper motor2;
    float currentPitch = 0.0;
    float currentRoll = 0.0;
    float targetPitch = 0.0;
    float targetRoll = 0.0;
    bool isHomed = false;
    bool isHomingInProgress = false;
    
    const float STEPS_PER_DEGREE;
    const float MAX_SPEED;
    const float MAX_ACCELERATION;
    
    // Limit switch pins
    const int limitSwitch1;
    const int limitSwitch2;
    
    // Homing state variables
    int homingStage = 0;
    unsigned long lastHomingUpdate = 0;

    void updateCurrentPosition() {
        // Calculate current pitch and roll from motor positions
        long pos1 = motor1.currentPosition();
        long pos2 = motor2.currentPosition();
        
        currentPitch = ((float)(pos1 + pos2) / (2.0 * STEPS_PER_DEGREE));
        currentRoll = ((float)(pos1 - pos2) / (2.0 * STEPS_PER_DEGREE));
    }

public:
    DifferentialWrist(int step1, int dir1, int step2, int dir2, 
                     int limit1, int limit2,
                     float stepsPerDeg, float maxSpeed, float maxAccel) :
        motor1(AccelStepper::DRIVER, step1, dir1),
        motor2(AccelStepper::DRIVER, step2, dir2),
        STEPS_PER_DEGREE(stepsPerDeg),
        MAX_SPEED(maxSpeed),
        MAX_ACCELERATION(maxAccel),
        limitSwitch1(limit1),
        limitSwitch2(limit2) {
            
        motor1.setMaxSpeed(MAX_SPEED);
        motor1.setAcceleration(MAX_ACCELERATION);
        motor2.setMaxSpeed(MAX_SPEED);
        motor2.setAcceleration(MAX_ACCELERATION);
        
        motor1.setCurrentPosition(0);
        motor2.setCurrentPosition(0);
        
        // Configure limit switch pins
        pinMode(limitSwitch1, INPUT_PULLUP);
        pinMode(limitSwitch2, INPUT_PULLUP);
    }

    void calculateMotorSteps(float pitch, float roll, long &steps1, long &steps2) {
        float motor1Angle = pitch + roll;
        float motor2Angle = pitch - roll;
        
        steps1 = (long)(motor1Angle * STEPS_PER_DEGREE);
        steps2 = (long)(motor2Angle * STEPS_PER_DEGREE);
    }

    void setTarget(float pitch, float roll) {
        // Skip if homing is in progress
        if (isHomingInProgress) return;
        
        // Constrain input values
        pitch = constrain(pitch, -20.0, 20.0);
        roll = constrain(roll, 0.0, 360.0);
        
        targetPitch = pitch;
        targetRoll = roll;
        
        long steps1, steps2;
        calculateMotorSteps(pitch, roll, steps1, steps2);
        
        // Update target positions immediately
        motor1.moveTo(steps1);
        motor2.moveTo(steps2);
        
        // Calculate speeds for coordinated movement
        long distance1 = abs(steps1 - motor1.currentPosition());
        long distance2 = abs(steps2 - motor2.currentPosition());
        long maxDistance = max(distance1, distance2);
        
        if (maxDistance > 0) {
            float speedRatio1 = distance1 > 0 ? (float)distance1 / maxDistance : 1.0;
            float speedRatio2 = distance2 > 0 ? (float)distance2 / maxDistance : 1.0;
            
            motor1.setMaxSpeed(MAX_SPEED * speedRatio1);
            motor2.setMaxSpeed(MAX_SPEED * speedRatio2);
            
            // Scale acceleration for smoother transitions
            motor1.setAcceleration(MAX_ACCELERATION * speedRatio1);
            motor2.setAcceleration(MAX_ACCELERATION * speedRatio2);
        }
    }
    
    void startHoming() {
        // Don't start homing if already in progress
        if (isHomingInProgress) return;
        
        // Initialize homing mode
        isHomingInProgress = true;
        isHomed = false;
        homingStage = 0;
        lastHomingUpdate = millis();
        
        // Set slower speeds for homing
        motor1.setMaxSpeed(MAX_SPEED * HOMING_SPEED_RATIO);
        motor2.setMaxSpeed(MAX_SPEED * HOMING_SPEED_RATIO);
        motor1.setAcceleration(MAX_ACCELERATION * HOMING_ACCEL_RATIO);
        motor2.setAcceleration(MAX_ACCELERATION * HOMING_ACCEL_RATIO);
        
        // Start moving motor1 toward home position
        motor1.move(-HOMING_TIMEOUT_STEPS); // Move in negative direction
    }
    
    bool updateHoming() {
        if (!isHomingInProgress) return true;
        
        switch (homingStage) {
            case 0: // Homing motor1
                motor1.run();
                
                // Check if limit switch is triggered
                if (digitalRead(limitSwitch1) == LOW) {
                    motor1.stop(); // Stop immediately
                    motor1.setCurrentPosition(0); // Set this as zero
                    motor1.moveTo(HOMING_BACKOFF_STEPS); // Move away from limit
                    homingStage = 1;
                }
                
                // Check for timeout or error
                if (abs(motor1.currentPosition()) >= HOMING_TIMEOUT_STEPS) {
                    isHomingInProgress = false;
                    return false; // Homing failed
                }
                break;
                
            case 1: // Backing off motor1
                if (motor1.run() == false && motor1.distanceToGo() == 0) {
                    motor1.setCurrentPosition(0);
                    // Start homing motor2
                    motor2.move(-HOMING_TIMEOUT_STEPS);
                    homingStage = 2;
                }
                break;
                
            case 2: // Homing motor2
                motor2.run();
                
                // Check if limit switch is triggered
                if (digitalRead(limitSwitch2) == LOW) {
                    motor2.stop(); // Stop immediately
                    motor2.setCurrentPosition(0); // Set this as zero
                    motor2.moveTo(HOMING_BACKOFF_STEPS); // Move away from limit
                    homingStage = 3;
                }
                
                // Check for timeout or error  
                if (abs(motor2.currentPosition()) >= HOMING_TIMEOUT_STEPS) {
                    isHomingInProgress = false;
                    return false; // Homing failed
                }
                break;
                
            case 3: // Backing off motor2
                if (motor2.run() == false && motor2.distanceToGo() == 0) {
                    motor2.setCurrentPosition(0);
                    // Homing complete
                    isHomingInProgress = false;
                    isHomed = true;
                    
                    // Reset to normal speeds
                    motor1.setMaxSpeed(MAX_SPEED);
                    motor2.setMaxSpeed(MAX_SPEED);
                    motor1.setAcceleration(MAX_ACCELERATION);
                    motor2.setAcceleration(MAX_ACCELERATION);
                    
                    currentPitch = 0.0;
                    currentRoll = 0.0;
                    targetPitch = 0.0;
                    targetRoll = 0.0;
                    
                    return true; // Homing successful
                }
                break;
        }
        
        // Check for timeout
        if (millis() - lastHomingUpdate > 30000) { // 30 second timeout
            isHomingInProgress = false;
            return false;
        }
        
        return false; // Still homing
    }

    void update() {
        // Handle homing if in progress
        if (isHomingInProgress) {
            updateHoming();
            return;
        }
        
        // Otherwise run normal movement
        motor1.run();
        motor2.run();
        
        // Update current position
        updateCurrentPosition();
    }

    // Get current positions
    float getCurrentPitch() { 
        return currentPitch; 
    }
    
    float getCurrentRoll() { 
        return currentRoll; 
    }
    
    // Get target positions
    float getTargetPitch() { 
        return targetPitch; 
    }
    
    float getTargetRoll() { 
        return targetRoll; 
    }
    
    // Check if motors are at their targets
    bool isAtTarget() {
        return (motor1.distanceToGo() == 0 && motor2.distanceToGo() == 0);
    }
    
    // Get movement completion percentage (0-100)
    float getProgress() {
        long totalDistance = abs(motor1.distanceToGo()) + abs(motor2.distanceToGo());
        long maxInitialDistance = abs(motor1.targetPosition()) + abs(motor2.targetPosition());
        if (maxInitialDistance == 0) return 100.0;
        return 100.0 * (1.0 - ((float)totalDistance / maxInitialDistance));
    }
    
    bool isHomingActive() {
        return isHomingInProgress;
    }
    
    bool hasHomed() {
        return isHomed;
    }
};

// Pin definitions
#define LEFT_ELBOW_SERVO_PIN 32
#define RIGHT_ELBOW_SERVO_PIN 33

// Motor pins
#define LEFT_MOTOR1_STEP_PIN 12
#define LEFT_MOTOR1_DIR_PIN 13
#define LEFT_MOTOR2_STEP_PIN 26
#define LEFT_MOTOR2_DIR_PIN 27
#define RIGHT_MOTOR1_STEP_PIN 2
#define RIGHT_MOTOR1_DIR_PIN 15
#define RIGHT_MOTOR2_STEP_PIN 5
#define RIGHT_MOTOR2_DIR_PIN 4
#define headstepperdirection 18 
#define headstepperstep 19
#define enablepin 25

// Motor parameters
const float STEPS_PER_REV = 200.0;
const float MICROSTEPS = 4.0;
const float GEAR_RATIO = 5.25;
const float MAX_SPEED = 500;
const float MAX_ACCELERATION = 2000;
const float STEPS_PER_DEGREE = (STEPS_PER_REV * MICROSTEPS * GEAR_RATIO) / 360.0;

// Create wrist instances with limit switch pins
DifferentialWrist leftWrist(
    LEFT_MOTOR1_STEP_PIN, LEFT_MOTOR1_DIR_PIN,
    LEFT_MOTOR2_STEP_PIN, LEFT_MOTOR2_DIR_PIN,
    LEFT_MOTOR1_LIMIT_PIN, LEFT_MOTOR2_LIMIT_PIN,
    STEPS_PER_DEGREE, MAX_SPEED, MAX_ACCELERATION
);

DifferentialWrist rightWrist(
    RIGHT_MOTOR1_STEP_PIN, RIGHT_MOTOR1_DIR_PIN,
    RIGHT_MOTOR2_STEP_PIN, RIGHT_MOTOR2_DIR_PIN,
    RIGHT_MOTOR1_LIMIT_PIN, RIGHT_MOTOR2_LIMIT_PIN,
    STEPS_PER_DEGREE, MAX_SPEED, MAX_ACCELERATION
);

// Servo objects
Servo left_elbow_servo;
Servo right_elbow_servo;

void ArmInit() {
    pinMode(enablepin, OUTPUT);
    digitalWrite(enablepin, LOW);
    
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    left_elbow_servo.setPeriodHertz(50);
    right_elbow_servo.setPeriodHertz(50);
    
    left_elbow_servo.attach(LEFT_ELBOW_SERVO_PIN, 1000, 2000);
    right_elbow_servo.attach(RIGHT_ELBOW_SERVO_PIN, 1000, 2000);
}

void enablesteppers() {
    digitalWrite(enablepin, LOW);
}

void disablesteppers() {
    digitalWrite(enablepin, HIGH);
}

void moveWrists(float leftPitch, float leftRoll, float rightPitch, float rightRoll) {
    enablesteppers();
    leftWrist.setTarget(leftPitch, leftRoll);
    rightWrist.setTarget(rightPitch, rightRoll);
}

void updateWrists() {
    leftWrist.update();
    rightWrist.update();
}

bool isAnyWristMoving() {
    return !leftWrist.isAtTarget() || !rightWrist.isAtTarget();
}

float getLeftWristProgress() {
    return leftWrist.getProgress();
}

float getRightWristProgress() {
    return rightWrist.getProgress();
}

// New homing functions
void startHomingWrists() {
    enablesteppers();
    leftWrist.startHoming();
    rightWrist.startHoming();
}

bool isHomingInProgress() {
    return leftWrist.isHomingActive() || rightWrist.isHomingActive();
}

bool areWristsHomed() {
    return leftWrist.hasHomed() && rightWrist.hasHomed();
}