#include <Arduino.h>
#include <MotorInit.h>

void setup() {
  Serial.begin(115200);
  ArmInit();
}

void loop() {
  if (!isMovementInProgress()) {
    startMove(70.0, 720.0);
  }
  updateMovement();
  
}