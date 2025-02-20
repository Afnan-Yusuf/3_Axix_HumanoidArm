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
  MODE_HOMING = 4,
  MODE_PATTERN = 5  // New mode for auto-generated patterns
};

// Pattern types for automatic movement generation
enum PatternType {
  PATTERN_SINE = 0,
  PATTERN_CIRCLE = 1,
  PATTERN_WAVE = 2,
  PATTERN_FIGURE8 = 3,
  PATTERN_RANDOM = 4,
  PATTERN_COUNT = 5  // Total number of patterns
};

PatternType currentPattern = PATTERN_WAVE;
OperationMode currentMode = MODE_STOP;
unsigned long walkStartTime = 0;
char cmd = 's';
bool walkphase = false;
unsigned long lastPositionUpdate = 0;
unsigned long patternStartTime = 0;
const unsigned long POSITION_UPDATE_INTERVAL = 20; // 20ms = 50Hz update rate

String msg;

// Structure to hold joint positions
struct JointPositions {
  float leftPitch;
  float leftRoll;
  float rightPitch;
  float rightRoll;
  int leftElbow;
  int rightElbow;
} currentPos = {0, 0, 0, 0, ELBOW_OFFSET, ELBOW_OFFSET};

// Function prototypes
bool parsePositionData(String data, JointPositions &pos);
void handlePositionUpdate();
void processSerialCommand(String command);
void generatePatternMovement();
void printPatternInfo();
char prevc = 's';
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

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, 16, 17); // RXD2, TXD2
  
  ArmInit();
  
  // Initial LED state
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  
  // Start with all steppers disabled
  disablesteppers();

  Serial.println("\n\n===== Differential Wrist Control =====");
  Serial.println("Commands:");
  Serial.println("  s or STOP - Stop all movement");
  Serial.println("  w or WALK - Start walking pattern");
  Serial.println("  h or HOME - Start homing sequence");
  Serial.println("  p or PATTERN - Start pattern mode");
  Serial.println("  p0-p4 - Select pattern (sine, circle, wave, figure8, random)");
  Serial.println("  d or DISABLE - Disable steppers");
  Serial.println("  e or ENABLE - Enable steppers");
  Serial.println("  P,lp,lr,rp,rr,le,re - Set position");
  Serial.println("=====================================");
}

void handlePositionUpdate() {
  moveWrists(currentPos.leftPitch, currentPos.leftRoll,
             currentPos.rightPitch, currentPos.rightRoll);
  left_elbow_servo.write(currentPos.leftElbow);
  right_elbow_servo.write(currentPos.rightElbow);
}

void processSerialCommand(String command) {
  command.trim();
  
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
  else if (command == "PATTERN" || command == "p") {
    currentMode = MODE_PATTERN;
    patternStartTime = millis();
    printPatternInfo();
    Serial.println("Mode: PATTERN GENERATOR");
    return;
  }
  else if (command.startsWith("p") && command.length() == 2) {
    // Pattern selection p0-p4
    int patternNum = command.charAt(1) - '0';
    if (patternNum >= 0 && patternNum < PATTERN_COUNT) {
      currentPattern = (PatternType)patternNum;
      patternStartTime = millis();
      printPatternInfo();
      currentMode = MODE_PATTERN;
    }
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

void printPatternInfo() {
  Serial.print("Current pattern: ");
  switch(currentPattern) {
    case PATTERN_SINE:
      Serial.println("Sine waves");
      break;
    case PATTERN_CIRCLE:
      Serial.println("Circular motion");
      break;
    case PATTERN_WAVE:
      Serial.println("Wave pattern");
      break;
    case PATTERN_FIGURE8:
      Serial.println("Figure-8 pattern");
      break;
    case PATTERN_RANDOM:
      Serial.println("Smooth random motion");
      break;
    default:
      Serial.println("Unknown");
      break;
  }
}

void generatePatternMovement() {
  // Calculate elapsed time for smooth patterns
  float elapsed = (millis() - patternStartTime) / 1000.0;
  
  switch(currentPattern) {
    case PATTERN_SINE:
      // Basic sine wave pattern
      currentPos.leftPitch = 15 * sin(elapsed);
      currentPos.leftRoll = 180 + 90 * sin(elapsed * 0.5);
      currentPos.rightPitch = 15 * sin(elapsed + PI);
      currentPos.rightRoll = 180 + 90 * sin(elapsed * 0.5 + PI);
      currentPos.leftElbow = 90 + 45 * sin(elapsed * 0.3);
      currentPos.rightElbow = 90 + 45 * sin(elapsed * 0.3 + PI);
      break;
      
    case PATTERN_CIRCLE:
      // Circular motion
      currentPos.leftPitch = 15 * sin(elapsed);
      currentPos.leftRoll = 180 + 90 * cos(elapsed);
      currentPos.rightPitch = 15 * sin(elapsed + PI);
      currentPos.rightRoll = 180 + 90 * cos(elapsed + PI);
      currentPos.leftElbow = 90 + 20 * sin(elapsed * 0.5);
      currentPos.rightElbow = 90 + 20 * sin(elapsed * 0.5 + PI);
      break;
      
    case PATTERN_WAVE:
      // Waving motion
      if (int(elapsed) % 4 < 2) {
        // Left arm waves
        currentPos.leftPitch = 10 * sin(elapsed * 3);
        currentPos.leftRoll = 45 + 30 * sin(elapsed * 3);
        currentPos.rightPitch = -5;
        currentPos.rightRoll = 0;
        currentPos.leftElbow = 90 + 30 * sin(elapsed * 3);
        currentPos.rightElbow = 95;
      } else {
        // Right arm waves
        currentPos.leftPitch = -5;
        currentPos.leftRoll = 0;
        currentPos.rightPitch = 10 * sin(elapsed * 3);
        currentPos.rightRoll = 45 + 30 * sin(elapsed * 3);
        currentPos.leftElbow = 95;
        currentPos.rightElbow = 90 + 30 * sin(elapsed * 3);
      }
      break;
      
    case PATTERN_FIGURE8:
      // Figure-8 pattern
      currentPos.leftPitch = 15 * sin(elapsed * 2);
      currentPos.leftRoll = 180 + 90 * sin(elapsed);
      currentPos.rightPitch = 15 * sin(elapsed * 2 + PI);
      currentPos.rightRoll = 180 + 90 * sin(elapsed + PI);
      currentPos.leftElbow = 90 + 20 * cos(elapsed);
      currentPos.rightElbow = 90 + 20 * cos(elapsed + PI);
      break;
      
    case PATTERN_RANDOM:
      // Smooth random motion using multiple sine waves of different frequencies
      currentPos.leftPitch = 10 * sin(elapsed) + 5 * sin(elapsed * 2.7) + 3 * sin(elapsed * 4.1);
      currentPos.leftRoll = 180 + 70 * sin(elapsed * 0.6) + 20 * sin(elapsed * 1.3);
      currentPos.rightPitch = 10 * sin(elapsed + 1.5) + 5 * sin(elapsed * 2.3) + 3 * sin(elapsed * 3.7);
      currentPos.rightRoll = 180 + 70 * sin(elapsed * 0.7 + 1) + 20 * sin(elapsed * 1.1);
      currentPos.leftElbow = 90 + 30 * sin(elapsed * 0.4) + 15 * sin(elapsed * 0.9);
      currentPos.rightElbow = 90 + 30 * sin(elapsed * 0.4 + 2) + 15 * sin(elapsed * 0.8);
      break;
  }
  
  // Constrain all values to valid ranges
  currentPos.leftPitch = constrain(currentPos.leftPitch, -20, 20);
  currentPos.leftRoll = constrain(currentPos.leftRoll, 0, 360);
  currentPos.rightPitch = constrain(currentPos.rightPitch, -20, 20);
  currentPos.rightRoll = constrain(currentPos.rightRoll, 0, 360);
  currentPos.leftElbow = constrain(currentPos.leftElbow, 0, 180);
  currentPos.rightElbow = constrain(currentPos.rightElbow, 0, 180);
}

void loop() {
   if (Serial.available()) {
     msg = Serial.readStringUntil('\n');
     Serial.println("Received: " + msg);
     processSerialCommand(msg);
   }

  

  if(Serial1.available()) {
    char c = Serial1.read();
    
    if(c != prevc){
      Serial.println("BT Received: " + msg);
      prevc = c;
      processSerialCommand(String(c)) ;
    }

    //processSerialCommand(String(c)) ;
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
      
    case MODE_PATTERN:
      // Generate dynamic pattern movement every update interval
      if (millis() - lastPositionUpdate >= POSITION_UPDATE_INTERVAL) {
        generatePatternMovement();
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
    if (currentMode == MODE_PATTERN) {
      // Report the current position when in pattern mode
      Serial.printf("Position: LP=%.1f, LR=%.1f, RP=%.1f, RR=%.1f, LE=%d, RE=%d\n",
                   currentPos.leftPitch, currentPos.leftRoll,
                   currentPos.rightPitch, currentPos.rightRoll,
                   currentPos.leftElbow, currentPos.rightElbow);
    }
    lastStatusUpdate = millis();
  }
}