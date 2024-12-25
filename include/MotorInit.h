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
    bool isMoving = false;
    
    const float STEPS_PER_DEGREE;
    const float MAX_SPEED;
    const float MAX_ACCELERATION;

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
            }
        }
    }

    bool isMovementInProgress() {
        return isMoving;
    }

    float getCurrentPitch() { return currentPitch; }
    float getCurrentRoll() { return currentRoll; }
};

// Pin definitions
#define LEFT_ELBOW_SERVO_PIN 12
#define RIGHT_ELBOW_SERVO_PIN 13

// Left wrist motor pins
#define LEFT_MOTOR1_STEP_PIN 5
#define LEFT_MOTOR1_DIR_PIN 4
#define LEFT_MOTOR2_STEP_PIN 2
#define LEFT_MOTOR2_DIR_PIN 15

// Right wrist motor pins (new pins, adjust as needed)
#define RIGHT_MOTOR1_STEP_PIN 12
#define RIGHT_MOTOR1_DIR_PIN 13
#define RIGHT_MOTOR2_STEP_PIN 26
#define RIGHT_MOTOR2_DIR_PIN 27

// Motor parameters
const float STEPS_PER_REV = 200.0;
const float MICROSTEPS = 4.0;
const float GEAR_RATIO = 5.25;
const float MAX_SPEED = 4000;
const float MAX_ACCELERATION = 4000;
const float STEPS_PER_DEGREE = (STEPS_PER_REV * MICROSTEPS * GEAR_RATIO) / 360.0;

// Create instances for both wrists
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
    // Initialize servo timers
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    // Configure servos
    left_elbow_servo.setPeriodHertz(50);
    right_elbow_servo.setPeriodHertz(50);
    
    left_elbow_servo.attach(LEFT_ELBOW_SERVO_PIN, 1000, 2000);
    right_elbow_servo.attach(RIGHT_ELBOW_SERVO_PIN, 1000, 2000);
}

// Function to move both wrists
void moveWrists(float leftPitch, float leftRoll, float rightPitch, float rightRoll) {
    leftWrist.startMove(leftPitch, leftRoll);
    rightWrist.startMove(rightPitch, rightRoll);
}

// Function to update both wrists
void updateWrists() {
    leftWrist.updateMovement();
    rightWrist.updateMovement();
}

// Function to check if either wrist is moving
bool isAnyWristMoving() {
    return leftWrist.isMovementInProgress() || rightWrist.isMovementInProgress();
}