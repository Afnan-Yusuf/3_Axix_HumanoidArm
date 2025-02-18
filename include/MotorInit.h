#pragma once
#include <AccelStepper.h>
#include <ESP32Servo.h>

// Add limit switch pin definitions
#define PITCH_MIN_LIMIT_PIN 34  // Limit switch for minimum pitch
#define PITCH_MAX_LIMIT_PIN 35  // Limit switch for maximum pitch
#define ROLL_MIN_LIMIT_PIN 36   // Limit switch for minimum roll
#define ROLL_MAX_LIMIT_PIN 39   // Limit switch for maximum roll

// Define homing parameters
#define HOMING_SPEED_RATIO 0.3f       // 30% of max speed
#define HOMING_ACCEL_RATIO 0.5f       // 50% of max acceleration
#define HOMING_BACKOFF_DEGREES 2.0f   // Degrees to back off after hitting limit
#define HOMING_TIMEOUT_MS 30000       // 30 second timeout for homing

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
    
    // Limit switch pins organized by joint axes
    const int pitchMinLimitPin;
    const int pitchMaxLimitPin;
    const int rollMinLimitPin;
    const int rollMaxLimitPin;
    
    // Homing state variables
    enum HomingStage {
        HOMING_IDLE = 0,
        HOMING_PITCH_MIN = 1,
        HOMING_PITCH_MIN_BACKOFF = 2,
        HOMING_PITCH_MAX = 3,
        HOMING_PITCH_MAX_BACKOFF = 4,
        HOMING_ROLL_MIN = 5,
        HOMING_ROLL_MIN_BACKOFF = 6,
        HOMING_ROLL_MAX = 7,
        HOMING_ROLL_MAX_BACKOFF = 8,
        HOMING_CENTER = 9,
        HOMING_COMPLETE = 10,
        HOMING_FAILED = 11
    };
    
    HomingStage homingStage = HOMING_IDLE;
    unsigned long homingStartTime = 0;
    float pitchMinPosition = 0.0;
    float pitchMaxPosition = 0.0;
    float rollMinPosition = 0.0;
    float rollMaxPosition = 0.0;

    void updateCurrentPosition() {
        // Calculate current pitch and roll from motor positions
        long pos1 = motor1.currentPosition();
        long pos2 = motor2.currentPosition();
        
        currentPitch = ((float)(pos1 + pos2) / (2.0 * STEPS_PER_DEGREE));
        currentRoll = ((float)(pos1 - pos2) / (2.0 * STEPS_PER_DEGREE));
    }

    bool isPitchMinLimitTriggered() {
        return digitalRead(pitchMinLimitPin) == LOW;
    }
    
    bool isPitchMaxLimitTriggered() {
        return digitalRead(pitchMaxLimitPin) == LOW;
    }
    
    bool isRollMinLimitTriggered() {
        return digitalRead(rollMinLimitPin) == LOW;
    }
    
    bool isRollMaxLimitTriggered() {
        return digitalRead(rollMaxLimitPin) == LOW;
    }
    
    // Move in a specific joint direction
    void moveToJointLimit(float pitch, float roll) {
        long steps1, steps2;
        calculateMotorSteps(pitch, roll, steps1, steps2);
        
        motor1.moveTo(steps1);
        motor2.moveTo(steps2);
    }

public:
    DifferentialWrist(int step1, int dir1, int step2, int dir2, 
                     int pitchMin, int pitchMax, int rollMin, int rollMax,
                     float stepsPerDeg, float maxSpeed, float maxAccel) :
        motor1(AccelStepper::DRIVER, step1, dir1),
        motor2(AccelStepper::DRIVER, step2, dir2),
        STEPS_PER_DEGREE(stepsPerDeg),
        MAX_SPEED(maxSpeed),
        MAX_ACCELERATION(maxAccel),
        pitchMinLimitPin(pitchMin),
        pitchMaxLimitPin(pitchMax),
        rollMinLimitPin(rollMin),
        rollMaxLimitPin(rollMax) {
            
        motor1.setMaxSpeed(MAX_SPEED);
        motor1.setAcceleration(MAX_ACCELERATION);
        motor2.setMaxSpeed(MAX_SPEED);
        motor2.setAcceleration(MAX_ACCELERATION);
        
        motor1.setCurrentPosition(0);
        motor2.setCurrentPosition(0);
        
        // Configure limit switch pins with pullup
        pinMode(pitchMinLimitPin, INPUT_PULLUP);
        pinMode(pitchMaxLimitPin, INPUT_PULLUP);
        pinMode(rollMinLimitPin, INPUT_PULLUP);
        pinMode(rollMaxLimitPin, INPUT_PULLUP);
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
        homingStage = HOMING_PITCH_MIN;
        homingStartTime = millis();
        
        // Set slower speeds for homing
        motor1.setMaxSpeed(MAX_SPEED * HOMING_SPEED_RATIO);
        motor2.setMaxSpeed(MAX_SPEED * HOMING_SPEED_RATIO);
        motor1.setAcceleration(MAX_ACCELERATION * HOMING_ACCEL_RATIO);
        motor2.setAcceleration(MAX_ACCELERATION * HOMING_ACCEL_RATIO);
        
        // Start moving in -pitch direction (both motors negative)
        moveToJointLimit(-50.0, 0.0); // Exaggerated negative pitch
    }
    
    bool updateHoming() {
        if (!isHomingInProgress) return true;
        
        // Check for timeout
        if (millis() - homingStartTime > HOMING_TIMEOUT_MS) {
            homingStage = HOMING_FAILED;
        }
        
        // Run both motors
        motor1.run();
        motor2.run();
        updateCurrentPosition();
        
        // Handle each homing stage
        switch (homingStage) {
            case HOMING_PITCH_MIN:
                // Moving to minimum pitch
                if (isPitchMinLimitTriggered()) {
                    // Reached minimum pitch limit
                    motor1.stop();
                    motor2.stop();
                    pitchMinPosition = currentPitch;
                    // Back off slightly
                    moveToJointLimit(currentPitch + HOMING_BACKOFF_DEGREES, currentRoll);
                    homingStage = HOMING_PITCH_MIN_BACKOFF;
                }
                break;
                
            case HOMING_PITCH_MIN_BACKOFF:
                // Waiting for backoff to complete
                if (motor1.distanceToGo() == 0 && motor2.distanceToGo() == 0) {
                    // Move to find maximum pitch
                    moveToJointLimit(50.0, 0.0); // Exaggerated positive pitch
                    homingStage = HOMING_PITCH_MAX;
                }
                break;
                
            case HOMING_PITCH_MAX:
                // Moving to maximum pitch
                if (isPitchMaxLimitTriggered()) {
                    // Reached maximum pitch limit
                    motor1.stop();
                    motor2.stop();
                    pitchMaxPosition = currentPitch;
                    // Back off slightly
                    moveToJointLimit(currentPitch - HOMING_BACKOFF_DEGREES, currentRoll);
                    homingStage = HOMING_PITCH_MAX_BACKOFF;
                }
                break;
                
            case HOMING_PITCH_MAX_BACKOFF:
                // Waiting for backoff to complete
                if (motor1.distanceToGo() == 0 && motor2.distanceToGo() == 0) {
                    // Now find minimum roll (pitch at center)
                    float centerPitch = (pitchMinPosition + pitchMaxPosition) / 2.0;
                    moveToJointLimit(centerPitch, -180.0); // Move to negative roll
                    homingStage = HOMING_ROLL_MIN;
                }
                break;
                
            case HOMING_ROLL_MIN:
                // Moving to minimum roll
                if (isRollMinLimitTriggered()) {
                    // Reached minimum roll limit
                    motor1.stop();
                    motor2.stop();
                    rollMinPosition = currentRoll;
                    // Back off slightly
                    moveToJointLimit(currentPitch, currentRoll + HOMING_BACKOFF_DEGREES);
                    homingStage = HOMING_ROLL_MIN_BACKOFF;
                }
                break;
                
            case HOMING_ROLL_MIN_BACKOFF:
                // Waiting for backoff to complete
                if (motor1.distanceToGo() == 0 && motor2.distanceToGo() == 0) {
                    // Now find maximum roll
                    moveToJointLimit(currentPitch, 180.0); // Move to positive roll
                    homingStage = HOMING_ROLL_MAX;
                }
                break;
                
            case HOMING_ROLL_MAX:
                // Moving to maximum roll
                if (isRollMaxLimitTriggered()) {
                    // Reached maximum roll limit
                    motor1.stop();
                    motor2.stop();
                    rollMaxPosition = currentRoll;
                    // Back off slightly
                    moveToJointLimit(currentPitch, currentRoll - HOMING_BACKOFF_DEGREES);
                    homingStage = HOMING_ROLL_MAX_BACKOFF;
                }
                break;
                
            case HOMING_ROLL_MAX_BACKOFF:
                // Waiting for backoff to complete
                if (motor1.distanceToGo() == 0 && motor2.distanceToGo() == 0) {
                    // Calculate center position
                    float centerPitch = (pitchMinPosition + pitchMaxPosition) / 2.0;
                    float centerRoll = (rollMinPosition + rollMaxPosition) / 2.0;
                    
                    // Move to center position
                    moveToJointLimit(centerPitch, centerRoll);
                    homingStage = HOMING_CENTER;
                }
                break;
                
            case HOMING_CENTER:
                // Waiting for centering to complete
                if (motor1.distanceToGo() == 0 && motor2.distanceToGo() == 0) {
                    // Reset position counters to establish zero at center
                    motor1.setCurrentPosition(0);
                    motor2.setCurrentPosition(0);
                    currentPitch = 0.0;
                    currentRoll = 0.0;
                    targetPitch = 0.0;
                    targetRoll = 0.0;
                    
                    // Reset to normal speeds
                    motor1.setMaxSpeed(MAX_SPEED);
                    motor2.setMaxSpeed(MAX_SPEED);
                    motor1.setAcceleration(MAX_ACCELERATION);
                    motor2.setAcceleration(MAX_ACCELERATION);
                    
                    // Homing complete!
                    homingStage = HOMING_COMPLETE;
                    isHomed = true;
                    isHomingInProgress = false;
                    return true;
                }
                break;
                
            case HOMING_FAILED:
                // Homing failed, reset everything
                motor1.stop();
                motor2.stop();
                motor1.setMaxSpeed(MAX_SPEED);
                motor2.setMaxSpeed(MAX_SPEED);
                motor1.setAcceleration(MAX_ACCELERATION);
                motor2.setAcceleration(MAX_ACCELERATION);
                isHomingInProgress = false;
                isHomed = false;
                return false;
                
            case HOMING_COMPLETE:
                isHomingInProgress = false;
                return true;
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
    
    HomingStage getHomingStage() {
        return homingStage;
    }
    
    String getHomingStageString() {
        switch(homingStage) {
            case HOMING_IDLE: return "IDLE";
            case HOMING_PITCH_MIN: return "PITCH_MIN";
            case HOMING_PITCH_MIN_BACKOFF: return "PITCH_MIN_BACKOFF";
            case HOMING_PITCH_MAX: return "PITCH_MAX";
            case HOMING_PITCH_MAX_BACKOFF: return "PITCH_MAX_BACKOFF";
            case HOMING_ROLL_MIN: return "ROLL_MIN";
            case HOMING_ROLL_MIN_BACKOFF: return "ROLL_MIN_BACKOFF";
            case HOMING_ROLL_MAX: return "ROLL_MAX";
            case HOMING_ROLL_MAX_BACKOFF: return "ROLL_MAX_BACKOFF";
            case HOMING_CENTER: return "CENTERING";
            case HOMING_COMPLETE: return "COMPLETE";
            case HOMING_FAILED: return "FAILED";
            default: return "UNKNOWN";
        }
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

// Create wrist instances with joint-oriented limit switch pins
DifferentialWrist leftWrist(
    LEFT_MOTOR1_STEP_PIN, LEFT_MOTOR1_DIR_PIN,
    LEFT_MOTOR2_STEP_PIN, LEFT_MOTOR2_DIR_PIN,
    PITCH_MIN_LIMIT_PIN, PITCH_MAX_LIMIT_PIN, 
    ROLL_MIN_LIMIT_PIN, ROLL_MAX_LIMIT_PIN,
    STEPS_PER_DEGREE, MAX_SPEED, MAX_ACCELERATION
);

DifferentialWrist rightWrist(
    RIGHT_MOTOR1_STEP_PIN, RIGHT_MOTOR1_DIR_PIN,
    RIGHT_MOTOR2_STEP_PIN, RIGHT_MOTOR2_DIR_PIN,
    PITCH_MIN_LIMIT_PIN, PITCH_MAX_LIMIT_PIN, 
    ROLL_MIN_LIMIT_PIN, ROLL_MAX_LIMIT_PIN,
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

// Joint-based homing functions
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

// Get homing stage for debugging
String getHomingStageString() {
    return "L:" + leftWrist.getHomingStageString() + " R:" + rightWrist.getHomingStageString();
}