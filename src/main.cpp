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


#define RXD2 16
#define TXD2 17

void setup()
{
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
  ArmInit();
}

void loop()
{
  if (Serial.available())
  {
    cmd = Serial.read();
    switch (cmd)
    {
    case 'w': 
      isWalking = true;
      walkStartTime = millis();
      Serial.println("Walking started");
      break;
    case 's':
      isWalking = false;
      moveWrists(0.0, 0, 0.0, 0); // Left shoulder
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
      moveWrists(0.0, 0, 0.0, 0); // Left shoulder
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
        moveWrists(SHOULDER_ROLL_OFFSET, 0, -SHOULDER_ROLL_OFFSET, 30); // Forward swing
        left_elbow_servo.write(90);
        walkphase = true;
      }
      else
      {
        moveWrists(SHOULDER_ROLL_OFFSET, 30, -SHOULDER_ROLL_OFFSET, 0); // Backward swing
        left_elbow_servo.write(120);
        walkphase = false;
      }
    }
    updateWrists();
  }
  else
  {
    updateWrists();
  }
}
