#include <Arduino.h>
#include <MotorInit.h>

const float SHOULDER_PITCH_AMPLITUDE = 90;
const float SHOULDER_ROLL_OFFSET = 10.0;
const float ELBOW_AMPLITUDE = 25.0;
const float ELBOW_OFFSET = 95.0;
const int WALK_CYCLE_MS = 2500;
const float RIGHT_PHASE_OFFSET = 180.0;

bool isWalking = false;
unsigned long walkStartTime = 0;
char cmd = 's';
bool walkphase = false;

String msg;

#define RXD2 16
#define TXD2 17

// Structure to hold joint positions
struct JointPositions
{
  float leftPitch;
  float leftRoll;
  float rightPitch;
  float rightRoll;
  int leftElbow;
  int rightElbow;
} currentPos = {0, 0, 0, 0, ELBOW_OFFSET, ELBOW_OFFSET};

// Function to parse position data from serial
bool parsePositionData(String data, JointPositions &pos)
{
  // Expected format: "P,leftPitch,leftRoll,rightPitch,rightRoll,leftElbow,rightElbow"
  if (data.startsWith("P,"))
  {
    int values[6];
    int startIndex = 2; // Skip "P,"

    for (int i = 0; i < 6; i++)
    {
      int commaIndex = data.indexOf(',', startIndex);
      if (commaIndex == -1 && i < 5)
        return false; // Not enough values

      String value = (i < 5) ? data.substring(startIndex, commaIndex) : data.substring(startIndex);

      values[i] = value.toInt();
      startIndex = commaIndex + 1;
    }

    // Update positions with constraints
    pos.leftPitch = constrain(values[0], -20, 20);
    pos.leftRoll = constrain(values[1], 0, 360);
    pos.rightPitch = constrain(values[2], -20, 20);
    pos.rightRoll = constrain(values[3], 0, 360);
    pos.leftElbow = constrain(values[4], 0, 180);
    pos.rightElbow = constrain(values[5], 0, 180);

    return true;
  }
  return false;
}

void setup()
{
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
  ArmInit();
}

void handlePositionUpdate()
{
  moveWrists(currentPos.leftPitch, currentPos.leftRoll,
             currentPos.rightPitch, currentPos.rightRoll);
  left_elbow_servo.write(currentPos.leftElbow);
  right_elbow_servo.write(currentPos.rightElbow);
}

void loop()
{
  if (Serial1.available())
  {
    msg = Serial1.readStringUntil('\n');
    Serial.println(msg);

    // Handle LED commands
    if (msg == "SPEAK_START")
    {
      pinMode(2, OUTPUT);
      digitalWrite(2, HIGH);
    }
    else if (msg == "SPEAK_STOP")
    {
      pinMode(2, OUTPUT);
      digitalWrite(2, LOW);
    }
    else
    {
      // Check if it's a position command
      JointPositions newPos = currentPos;
      if (parsePositionData(msg, newPos))
      {
        isWalking = false; // Stop walking mode if active
        currentPos = newPos;
        handlePositionUpdate();
      }
      else
      {
        // Handle original commands
        cmd = msg[0];
        switch (cmd)
        {
        case 'w':
          isWalking = true;
          walkStartTime = millis();
          break;
        case 's':
          isWalking = false;
          moveWrists(0.0, 0, 0.0, 0);
          left_elbow_servo.write(ELBOW_OFFSET);
          right_elbow_servo.write(ELBOW_OFFSET);
          break;
        case 'd':
          disablesteppers();
          break;
        case 'e':
          enablesteppers();
          break;
        }
      }
    }
  }

  // Handle Serial1 (Bluetooth) commands
  if (Serial1.available())
  {
    cmd = Serial1.read();
    switch (cmd)
    {
    case 'w':
      isWalking = true;
      walkStartTime = millis();
      Serial.println("Walking started");
      break;
    case 's':
      isWalking = false;
      moveWrists(0.0, 0, 0.0, 0);
      left_elbow_servo.write(ELBOW_OFFSET);
      right_elbow_servo.write(ELBOW_OFFSET);
      Serial.println("Walking stopped");
      break;
    case 'd':
      disablesteppers();
      break;
    case 'e':
      enablesteppers();
      break;
    }
  }

  // Walking mode logic
  if (isWalking)
  {
    unsigned long currentTime = millis();
    unsigned long timeInCycle = (currentTime - walkStartTime) % WALK_CYCLE_MS;
    float phase = float(timeInCycle) / WALK_CYCLE_MS;
    float leftRoll = phase * 45.0;
    float rightRoll = 45.0 - leftRoll;

    if (!isAnyWristMoving())
    {
      if (walkphase == false)
      {
        moveWrists(SHOULDER_ROLL_OFFSET, 0, -SHOULDER_ROLL_OFFSET, 30);
        left_elbow_servo.write(90);
        right_elbow_servo.write(120);
        walkphase = true;
      }
      else
      {
        moveWrists(SHOULDER_ROLL_OFFSET, 30, -SHOULDER_ROLL_OFFSET, 0);
        left_elbow_servo.write(120);
        right_elbow_servo.write(90);
        walkphase = false;
      }
    }
  }

  updateWrists();
}