#pragma once
#include <AccelStepper.h>
#include <ESP32Servo.h>

class DifferentialWrist {
private:
    AccelStepper motor1;
    AccelStepper motor2;
    float currentPitch = 0.0;
    float currentRoll = 0.0;
    float targetPitch = 0.0;
    float targetRoll = 0.0;
    
    const float STEPS_PER_DEGREE;
    const float MAX_SPEED;
    const float MAX_ACCELERATION;

    void updateCurrentPosition() {
        // Calculate current pitch and roll from motor positions
        long pos1 = motor1.currentPosition();
        long pos2 = motor2.currentPosition();
        
        currentPitch = ((float)(pos1 + pos2) / (2.0 * STEPS_PER_DEGREE));
        currentRoll = ((float)(pos1 - pos2) / (2.0 * STEPS_PER_DEGREE));
    }

public:
    DifferentialWrist(int step1, int dir1, int step2, int dir2, 
                     float stepsPerDeg, float maxSpeed, float maxAccel) :
        motor1(AccelStepper::DRIVER, step1, dir1),
        motor2(AccelStepper::DRIVER, step2, dir2),
        STEPS_PER_DEGREE(stepsPerDeg),
        MAX_SPEED(maxSpeed),
        MAX_ACCELERATION(maxAccel) {
            
        motor1.setMaxSpeed(MAX_SPEED);
        motor1.setAcceleration(MAX_ACCELERATION);
        motor2.setMaxSpeed(MAX_SPEED);
        motor2.setAcceleration(MAX_ACCELERATION);
        
        motor1.setCurrentPosition(0);
        motor2.setCurrentPosition(0);
    }

    void calculateMotorSteps(float pitch, float roll, long &steps1, long &steps2) {
        float motor1Angle = pitch + roll;
        float motor2Angle = pitch - roll;
        
        steps1 = (long)(motor1Angle * STEPS_PER_DEGREE);
        steps2 = (long)(motor2Angle * STEPS_PER_DEGREE);
    }

    void setTarget(float pitch, float roll) {
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

    void update() {
        // Run both motors
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
};

// Pin definitions remain the same
#define LEFT_ELBOW_SERVO_PIN 32
#define RIGHT_ELBOW_SERVO_PIN 33

// Motor pins remain the same
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

// Motor parameters remain the same
const float STEPS_PER_REV = 200.0;
const float MICROSTEPS = 4.0;
const float GEAR_RATIO = 5.25;
const float MAX_SPEED = 500;
const float MAX_ACCELERATION = 2000;
const float STEPS_PER_DEGREE = (STEPS_PER_REV * MICROSTEPS * GEAR_RATIO) / 360.0;

// Create wrist instances
DifferentialWrist leftWrist(
    LEFT_MOTOR1_STEP_PIN, LEFT_MOTOR1_DIR_PIN,
    LEFT_MOTOR2_STEP_PIN, LEFT_MOTOR2_DIR_PIN,
    STEPS_PER_DEGREE, MAX_SPEED, MAX_ACCELERATION
);

DifferentialWrist rightWrist(
    RIGHT_MOTOR1_STEP_PIN, RIGHT_MOTOR1_DIR_PIN,
    RIGHT_MOTOR2_STEP_PIN, RIGHT_MOTOR2_DIR_PIN,
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