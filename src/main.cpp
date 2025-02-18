#include <Arduino.h>
#include <MotorInit.h>

const float SHOULDER_PITCH_AMPLITUDE = 90;
const float SHOULDER_ROLL_OFFSET = 10.0;
const float ELBOW_AMPLITUDE = 25.0;
const float ELBOW_OFFSET = 95.0;
const int WALK_CYCLE_MS = 2500;
const float RIGHT_PHASE_OFFSET = 180.0;

// Operation modes
enum OperationMode {
  MODE_STOP = 0,
  MODE_WALKING = 1,
  MODE_POSITION = 2,
  MODE_CONTINUOUS = 3,
  MODE_HOMING = 4
};

OperationMode currentMode = MODE_STOP;
unsigned long walkStartTime = 0;
char cmd = 's';
bool walkphase = false;
unsigned long lastPositionUpdate = 0;
const unsigned long POSITION_UPDATE_INTERVAL = 20; // 20ms = 50Hz update rate

String msg;

// For continuous input
#define LEFT_PITCH_POT_PIN 14   // Analog pin for potentiometer input
#define LEFT_ROLL_POT_PIN 27    // Adjust pins as needed
#define RIGHT_PITCH_POT_PIN 25
#define RIGHT_ROLL_POT_PIN 26
#define LEFT_ELBOW_POT_PIN 32
#define RIGHT_ELBOW_POT_PIN 33

#define RXD2 16
#define TXD2 17





// Structure to hold joint positions
struct JointPositions {
  float leftPitch;
  float leftRoll;
  float rightPitch;
  float rightRoll;
  int leftElbow;
  int rightElbow;
} currentPos = {0, 0, 0, 0, ELBOW_OFFSET, ELBOW_OFFSET};

// Function to parse position data from serial
void applyDeadband(float &value, float threshold);
bool parsePositionData(String data, JointPositions &pos);
void readContinuousInputs(JointPositions &pos);
void handlePositionUpdate();
void processSerialCommand(String command);


bool parsePositionData(String data, JointPositions &pos) {
  // Expected format: "P,leftPitch,leftRoll,rightPitch,rightRoll,leftElbow,rightElbow"
  if (data.startsWith("P,")) {
    int values[6];
    int startIndex = 2; // Skip "P,"

    for (int i = 0; i < 6; i++) {
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

// Function to read potentiometer values and map to joint ranges
void readContinuousInputs(JointPositions &pos) {
  // Read analog values (0-4095 on ESP32)
  int leftPitchRaw = analogRead(LEFT_PITCH_POT_PIN);
  int leftRollRaw = analogRead(LEFT_ROLL_POT_PIN);
  int rightPitchRaw = analogRead(RIGHT_PITCH_POT_PIN);
  int rightRollRaw = analogRead(RIGHT_ROLL_POT_PIN);
  int leftElbowRaw = analogRead(LEFT_ELBOW_POT_PIN);
  int rightElbowRaw = analogRead(RIGHT_ELBOW_POT_PIN);
  
  // Map to appropriate ranges with deadband to prevent jitter
  pos.leftPitch = map(leftPitchRaw, 0, 4095, -20, 20);
  pos.leftRoll = map(leftRollRaw, 0, 4095, 0, 360);
  pos.rightPitch = map(rightPitchRaw, 0, 4095, -20, 20);
  pos.rightRoll = map(rightRollRaw, 0, 4095, 0, 360);
  pos.leftElbow = map(leftElbowRaw, 0, 4095, 0, 180);
  pos.rightElbow = map(rightElbowRaw, 0, 4095, 0, 180);
  
  // Apply anti-jitter filtering (optional)
  applyDeadband(pos.leftPitch, 0.5);
  applyDeadband(pos.leftRoll, 1.0);
  applyDeadband(pos.rightPitch, 0.5);
  applyDeadband(pos.rightRoll, 1.0);
}

// Apply deadband to prevent small noise from causing movement
void applyDeadband(float &value, float threshold) {
  if (abs(value) < threshold) {
    value = 0;
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
  
  // Set up analog input pins
  analogReadResolution(12); // 12-bit resolution for ESP32
  analogSetAttenuation(ADC_11db); // Improves analog reading range
  
  ArmInit();
  
  // Initial LED state
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  
  // Start with all steppers disabled
  disablesteppers();
}

void handlePositionUpdate() {
  moveWrists(currentPos.leftPitch, currentPos.leftRoll,
             currentPos.rightPitch, currentPos.rightRoll);
  left_elbow_servo.write(currentPos.leftElbow);
  right_elbow_servo.write(currentPos.rightElbow);
}

void processSerialCommand(String command) {
  // Check mode change commands first
  if (command == "STOP" || command == "s") {
    currentMode = MODE_STOP;
    moveWrists(0.0, 0, 0.0, 0);
    left_elbow_servo.write(ELBOW_OFFSET);
    right_elbow_servo.write(ELBOW_OFFSET);
    Serial.println("Mode: STOP");
    return;
  } 
  else if (command == "WALK" || command == "w") {
    currentMode = MODE_WALKING;
    walkStartTime = millis();
    Serial.println("Mode: WALKING");
    return;
  }
  else if (command == "HOME" || command == "h") {
    currentMode = MODE_HOMING;
    startHomingWrists();
    Serial.println("Mode: HOMING");
    return;
  }
  else if (command == "CONTINUOUS" || command == "c") {
    currentMode = MODE_CONTINUOUS;
    Serial.println("Mode: CONTINUOUS INPUT");
    return;
  }
  else if (command == "DISABLE" || command == "d") {
    disablesteppers();
    Serial.println("Steppers disabled");
    return;
  }
  else if (command == "ENABLE" || command == "e") {
    enablesteppers();
    Serial.println("Steppers enabled");
    return;
  }
  
  // Handle position command
  JointPositions newPos = currentPos;
  if (parsePositionData(command, newPos)) {
    currentMode = MODE_POSITION;
    currentPos = newPos;
    handlePositionUpdate();
    Serial.println("Mode: POSITION");
  }
  
  // Handle LED commands
  if (command == "SPEAK_START") {
    pinMode(2, OUTPUT);
    digitalWrite(2, HIGH);
  } 
  else if (command == "SPEAK_STOP") {
    pinMode(2, OUTPUT);
    digitalWrite(2, LOW);
  }
}

void loop() {
  // Process serial input if available
  if (Serial.available()) {
    msg = Serial.readStringUntil('\n');
    Serial.println("Received: " + msg);
    processSerialCommand(msg);
  }

  if (Serial1.available()) {
    msg = Serial1.readStringUntil('\n');
    Serial.println("BT Received: " + msg);
    processSerialCommand(msg);
  }

  // Handle different modes
  switch (currentMode) {
    case MODE_HOMING:
      if (!isHomingInProgress()) {
        if (areWristsHomed()) {
          Serial.println("Homing completed successfully");
          currentMode = MODE_STOP;
        } else {
          Serial.println("Homing failed");
          currentMode = MODE_STOP;
        }
      }
      break;
      
    case MODE_WALKING:
      // Walking pattern logic
      if (!isHomingInProgress()) {  // Don't walk if homing
        unsigned long currentTime = millis();
        unsigned long timeInCycle = (currentTime - walkStartTime) % WALK_CYCLE_MS;
        float phase = float(timeInCycle) / WALK_CYCLE_MS;
        
        if (!isAnyWristMoving()) {
          if (walkphase == false) {
            moveWrists(SHOULDER_ROLL_OFFSET, 0, -SHOULDER_ROLL_OFFSET, 30);
            left_elbow_servo.write(90);
            right_elbow_servo.write(120);
            walkphase = true;
          } else {
            moveWrists(SHOULDER_ROLL_OFFSET, 30, -SHOULDER_ROLL_OFFSET, 0);
            left_elbow_servo.write(120);
            right_elbow_servo.write(90);
            walkphase = false;
          }
        }
      }
      break;
      
    case MODE_CONTINUOUS:
      // Check if it's time to update position (throttle updates)
      if (millis() - lastPositionUpdate >= POSITION_UPDATE_INTERVAL) {
        readContinuousInputs(currentPos);
        handlePositionUpdate();
        lastPositionUpdate = millis();
      }
      break;
      
    case MODE_STOP:
      // In stop mode, we do nothing active but still update motors
      break;
      
    case MODE_POSITION:
      // In position mode, we've already set the position, just maintain it
      break;
  }

  // Always update motors
  updateWrists();
  
  // Report status periodically (optional)
  static unsigned long lastStatusUpdate = 0;
  if (millis() - lastStatusUpdate > 1000) { // Once per second
    if (currentMode == MODE_CONTINUOUS) {
      // Report the current position when in continuous mode
      Serial.printf("Position: LP=%.1f, LR=%.1f, RP=%.1f, RR=%.1f, LE=%d, RE=%d\n",
                   currentPos.leftPitch, currentPos.leftRoll,
                   currentPos.rightPitch, currentPos.rightRoll,
                   currentPos.leftElbow, currentPos.rightElbow);
    }
    lastStatusUpdate = millis();
  }
}