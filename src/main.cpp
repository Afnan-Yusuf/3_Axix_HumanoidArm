#include <Arduino.h>
#include <MotorInit.h>

const float SHOULDER_PITCH_AMPLITUDE = 90; // Maximum forward/backward swing
const float SHOULDER_ROLL_OFFSET = -45.0;     // Base roll position
const float ELBOW_AMPLITUDE = 15.0;          // Elbow swing amplitude
const float ELBOW_OFFSET = 90.0;             // Center position for elbow
const int WALK_CYCLE_MS = 1000;              // Time for one complete cycle
const float RIGHT_PHASE_OFFSET = 180.0;      // Opposite phase for right arm

// Walking state variables
bool isWalking = false;
unsigned long walkStartTime = 0;
char cmd;

void setup()
{
  Serial.begin(115200);
  ArmInit();
}

void loop()
{
  if (Serial.available())
  {
    cmd = Serial.read();
    switch (cmd)
    {
    case 'w': // Start walking
      isWalking = true;
      walkStartTime = millis();
      Serial.println("Walking started");
      break;

    case 's': // Stop walking
      isWalking = false;
      // Return to neutral position
      moveWrists(0.0, SHOULDER_ROLL_OFFSET, 0.0, SHOULDER_ROLL_OFFSET); // Left shoulder
      left_elbow_servo.write(ELBOW_OFFSET);
      right_elbow_servo.write(ELBOW_OFFSET);
      Serial.println("Walking stopped");
      break;
    }
  }

  if (isWalking)
  {
    // Calculate current phase in the walking cycle
    unsigned long currentTime = millis();
    unsigned long timeInCycle = (currentTime - walkStartTime) % WALK_CYCLE_MS;

    // Normalize phase (0 to 1 within the cycle)
    float phase = float(timeInCycle) / WALK_CYCLE_MS;

    // Generate triangular wave for left and right phases
    float leftPhase = (phase < 0.5) ? (phase * 2.0) : (1.0 - (phase - 0.5) * 2.0);
    float rightPhase = fmod(phase + float(RIGHT_PHASE_OFFSET) / 360.0, 1.0);
    rightPhase = (rightPhase < 0.5) ? (rightPhase * 2.0) : (1.0 - (rightPhase - 0.5) * 2.0);

    // Scale phases to amplitudes for shoulder movements
    float leftPitch = SHOULDER_PITCH_AMPLITUDE * leftPhase;
    float rightPitch = SHOULDER_PITCH_AMPLITUDE * rightPhase;

    // Calculate elbow angles
    int leftElbowAngle = ELBOW_OFFSET + ELBOW_AMPLITUDE * leftPhase;
    int rightElbowAngle = ELBOW_OFFSET + ELBOW_AMPLITUDE * rightPhase;

    // Move shoulders if not already moving
    if (!isAnyWristMoving())
    {
      // Left shoulder movement
      moveWrists(SHOULDER_ROLL_OFFSET, leftPitch, SHOULDER_ROLL_OFFSET, rightPitch);
    }

    // Move elbows
    left_elbow_servo.write(leftElbowAngle);
    right_elbow_servo.write(rightElbowAngle);

    // Update shoulder movements
    updateWrists();

    // Optional: Debug output (uncomment if needed)
    /*
    Serial.print("Left Pitch: "); Serial.print(leftPitch);
    Serial.print(" Right Pitch: "); Serial.print(rightPitch);
    Serial.print(" Left Elbow: "); Serial.print(leftElbowAngle);
    Serial.print(" Right Elbow: "); Serial.println(rightElbowAngle);
    */
  }
  else
  {
    // If not walking, just update any ongoing movements
    updateWrists();
  }
}

