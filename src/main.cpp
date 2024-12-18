#include <Arduino.h>
#include <MotorInit.h>

void setup() {
  Serial.begin(115200);
  ArmInit();
}

void loop() {
  if (!isMovementInProgress()) {
    startMove(360.0, 180.0);
  }
  updateMovement();
  
}