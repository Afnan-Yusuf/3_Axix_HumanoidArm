#include <Arduino.h>
#include <MotorInit.h>

void setup() {
  Serial.begin(115200);
  ArmInit();
}

void loop() {
  if (!isAnyWristMoving()) {
    moveWrists(0, 0, 0, 0);
  }
  right_elbow_servo.write(90);
  left_elbow_servo.write(90);

  updateWrists();
  
}