#include <Arduino.h>
#include <MotorInit.h>



void setup() {
  ArmInit();
  Serial.begin(115200);
}

void loop() {
  left_arm_pitch_roll(100,0);
  LSx.run();
  LSy.run();
}