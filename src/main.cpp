#include <Arduino.h>
#include <MotorInit.h>

void setup() {
  Serial.begin(115200);
  ArmInit();
}

void loop() {
  if (!isMovementInProgress()) {
    startMove(65.0, 360.0);
    startMove(65.0, 0.0);
  }

  updateMovement();
  
}