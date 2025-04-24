/**
 * Simple and Robust Line Follower
 * Uses a weighted approach instead of PID for reliable line tracking
 */

// Motor pins
#define RIGHT_MOTOR_FORWARD 10  // IN1
#define RIGHT_MOTOR_BACKWARD 11 // IN2
#define LEFT_MOTOR_FORWARD 9    // IN3
#define LEFT_MOTOR_BACKWARD 6   // IN4

// Constants for line following
#define SENSOR_COUNT 8
#define THRESHOLD 760           // Threshold to detect black line
#define CENTER_POSITION 4.5     // Center position (between sensors 4 and 5)

// Speed settings
#define BASE_SPEED 85           // Normal speed
#define TURN_SPEED 70           // Speed during turns
#define ROTATION_SPEED 100      // Speed for on-the-spot rotation

// Time constants
#define STRAIGHT_RESET_TIME 1000 // Time (ms) to reset direction memory after going straight

// Variables
int sensor[SENSOR_COUNT];       // Array to store sensor values
float linePosition = CENTER_POSITION; // Current line position (1-8)
char lineDirection = 'C';       // 'L' for left, 'R' for right, 'C' for center, 'N' for none
char lastLineDirection = 'N';   // Last known direction of the line
unsigned long lastLineTime = 0; // Last time the line was detected
boolean isTurning = false;      // Flag to indicate we're in a turn

// Variables for direction reset timer
unsigned long straightStartTime = 0;  // When robot started going straight
boolean isGoingStraight = false;      // Flag to track straight motion

/**
 * Setup function - runs once at startup
 */
void setup() {
  // Set motor control pins as outputs
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  
  Serial.begin(9600);
}

/**
 * Main loop - runs continuously
 */
void loop() {
  // Read sensors and update line position
  readSensors();
  
  // Track straight movement for direction reset
  updateStraightTimer();
  
  // Determine what to do based on sensor readings
  if (isLineDetected()) {
    // If in a full turn, only exit when properly centered on the line
    if (isTurning) {
      if (isCentered()) {
        isTurning = false;
      } else {
        // Continue the turn until centered
        continueTurn(lastLineDirection);
        return;
      }
    }
    
    // Normal line following
    followLine();
  } else {
    // No line detected - handle line loss
    handleLineLoss();
  }
  
  // Debug output
  printDebugInfo();
}

/**
 * Track how long robot has been going straight to reset direction memory
 */
void updateStraightTimer() {
  // Check if robot is currently centered on the line and not turning
  if (isCentered() && !isTurning) {
    // Start tracking straight movement if we weren't already
    if (!isGoingStraight) {
      straightStartTime = millis();
      isGoingStraight = true;
    } 
    // If going straight for more than STRAIGHT_RESET_TIME, reset direction memory
    else if (millis() - straightStartTime > STRAIGHT_RESET_TIME) {
      lastLineDirection = 'N';  // Reset direction memory
    }
  } else {
    // Not going straight, reset the flag
    isGoingStraight = false;
  }
}

/**
 * Reads all IR sensors and calculates line position
 */
void readSensors() {
  // Read all 8 IR sensors
  sensor[0] = analogRead(A0);
  sensor[1] = analogRead(A1);
  sensor[2] = analogRead(A2);
  sensor[3] = analogRead(A3);
  sensor[4] = analogRead(A4);
  sensor[5] = analogRead(A5);
  sensor[6] = analogRead(A6);
  sensor[7] = analogRead(A7);
  
  // Calculate weighted position
  float weightedSum = 0;
  int activeSensors = 0;
  
  for (int i = 0; i < SENSOR_COUNT; i++) {
    // Convert to binary (0/1)
    int value = (sensor[i] > THRESHOLD) ? 1 : 0;
    sensor[i] = value;
    
    // Add to weighted sum (position 1-8)
    if (value == 1) {
      weightedSum += (i + 1);
      activeSensors++;
    }
  }
  
  // Update line position if we have active sensors
  if (activeSensors > 0) {
    linePosition = weightedSum / activeSensors;
    lastLineTime = millis();
    
    // Update line direction
    if (linePosition < 3.5) {
      lineDirection = 'L';
    } else if (linePosition > 5.5) {
      lineDirection = 'R';
    } else {
      lineDirection = 'C';
    }
    
    // Update last direction only when we have a clear reading (not centered)
    if (lineDirection != 'C') {
      lastLineDirection = lineDirection;
    }
  } else {
    lineDirection = 'N';
  }
}

/**
 * Checks if any sensor detects the line
 */
bool isLineDetected() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (sensor[i] == 1) {
      return true;
    }
  }
  return false;
}

/**
 * Checks if the robot is centered on the line
 */
bool isCentered() {
  return (sensor[3] == 1 || sensor[4] == 1) && 
         (linePosition >= 3.5 && linePosition <= 5.5);
}

/**
 * Checks if all sensors detect black (junction or end of track)
 */
bool isAllBlack() {
  int sum = 0;
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sum += sensor[i];
  }
  return sum == SENSOR_COUNT;
}

/**
 * Basic line following function that adjusts speed based on line position
 */
void followLine() {
  // Calculate how far off center we are (0.0 to 3.5)
  float offset = abs(linePosition - CENTER_POSITION);
  
  // Special cases for turns
  if (sensor[0] == 1 && !isRightSensorActive() && !isCenterSensorActive()) {
    // Hard left turn detected
    lastLineDirection = 'L';
    turnLeft();
    return;
  }
  
  if (sensor[7] == 1 && !isLeftSensorActive() && !isCenterSensorActive()) {
    // Hard right turn detected
    lastLineDirection = 'R';
    turnRight();
    return;
  }
  
  // Check for all black (junction)
  if (isAllBlack()) {
    delay(20); // Brief delay to make sure it's a junction
    readSensors();
    
    if (isAllBlack()) {
      // All black - stop
      stopMotors();
      
      // Wait until we're no longer on all black
      while (isAllBlack()) {
        readSensors();
        delay(10);
      }
      return;
    }
  }
  
  // Normal line following - adjust speeds based on position
  int leftSpeed, rightSpeed;
  
  // Line is to the left - FIXED DIRECTION LOGIC
  if (linePosition < CENTER_POSITION) {
    float factor = map(offset, 0, 3.5, 0, BASE_SPEED * 0.7);
    // When line is to the left, SLOW DOWN the left motor, speed up right motor
    leftSpeed = BASE_SPEED - factor;
    rightSpeed = BASE_SPEED;
  } 
  // Line is to the right - FIXED DIRECTION LOGIC
  else {
    float factor = map(offset, 0, 3.5, 0, BASE_SPEED * 0.7);
    // When line is to the right, SLOW DOWN the right motor, speed up left motor
    leftSpeed = BASE_SPEED;
    rightSpeed = BASE_SPEED - factor;
  }
  
  // Ensure minimum speed
  if (leftSpeed < TURN_SPEED) leftSpeed = TURN_SPEED;
  if (rightSpeed < TURN_SPEED) rightSpeed = TURN_SPEED;
  
  // Set motor speeds
  setMotors(leftSpeed, rightSpeed);
}

/**
 * Handle case where line is completely lost
 */
void handleLineLoss() {
  // If we just lost the line (within last 100ms), continue same direction
  if (millis() - lastLineTime < 100) {
    // FIXED DIRECTION LOGIC for recovery
    if (lastLineDirection == 'L') {
      // Line was to the left, so turn left (left motor slower than right)
      setMotors(TURN_SPEED, BASE_SPEED);
    } else if (lastLineDirection == 'R') {
      // Line was to the right, so turn right (right motor slower than left)
      setMotors(BASE_SPEED, TURN_SPEED);
    } else {
      setMotors(BASE_SPEED, BASE_SPEED);
    }
  } else {
    // Line has been lost for a while, start a rotation to find it
    isTurning = true;
    
    // FIXED DIRECTION LOGIC for rotation
    if (lastLineDirection == 'L') {
      rotateLeft();
    } else if (lastLineDirection == 'R') {
      rotateRight();
    } else {
      // If we have no direction memory, try a full scan
      fullScan();
    }
  }
}

/**
 * Full scan rotation to find the line when direction is unknown
 */
void fullScan() {
  // First try a left rotation for a short time
  unsigned long scanStartTime = millis();
  
  // Try rotating left for 500ms
  while (millis() - scanStartTime < 500) {
    rotateLeft();
    readSensors();
    if (isLineDetected()) return;
    delay(10);
  }
  
  // If line not found, try right rotation for 1 second
  scanStartTime = millis();
  while (millis() - scanStartTime < 1000) {
    rotateRight();
    readSensors();
    if (isLineDetected()) return;
    delay(10);
  }
  
  // If still not found, do a slower full scan
  rotateLeft();
}

/**
 * Continue turn in the given direction
 */
void continueTurn(char direction) {
  if (direction == 'L') {
    rotateLeft();
  } else if (direction == 'R') {
    rotateRight();
  } else {
    // If no direction known, do a full scan
    fullScan();
  }
}

/**
 * Start a full left turn
 */
void turnLeft() {
  isTurning = true;
  isGoingStraight = false;
  lastLineDirection = 'L';
  rotateLeft();
}

/**
 * Start a full right turn
 */
void turnRight() {
  isTurning = true;
  isGoingStraight = false;
  lastLineDirection = 'R';
  rotateRight();
}

/**
 * Rotate left (in place) to find the line
 */
void rotateLeft() {
  setMotors(-ROTATION_SPEED, ROTATION_SPEED);
}

/**
 * Rotate right (in place) to find the line
 */
void rotateRight() {
  setMotors(ROTATION_SPEED, -ROTATION_SPEED);
}

/**
 * Stop all motors
 */
void stopMotors() {
  setMotors(0, 0);
}

/**
 * Set motor speeds directly
 */
void setMotors(int leftSpeed, int rightSpeed) {
  // Left motor
  if (leftSpeed >= 0) {
    analogWrite(LEFT_MOTOR_FORWARD, leftSpeed);
    analogWrite(LEFT_MOTOR_BACKWARD, 0);
  } else {
    analogWrite(LEFT_MOTOR_FORWARD, 0);
    analogWrite(LEFT_MOTOR_BACKWARD, -leftSpeed);
  }
  
  // Right motor
  if (rightSpeed >= 0) {
    analogWrite(RIGHT_MOTOR_FORWARD, rightSpeed);
    analogWrite(RIGHT_MOTOR_BACKWARD, 0);
  } else {
    analogWrite(RIGHT_MOTOR_FORWARD, 0);
    analogWrite(RIGHT_MOTOR_BACKWARD, -rightSpeed);
  }
}

/**
 * Helper function to check if any left sensors are active
 */
bool isLeftSensorActive() {
  return (sensor[0] == 1 || sensor[1] == 1 || sensor[2] == 1);
}

/**
 * Helper function to check if any right sensors are active
 */
bool isRightSensorActive() {
  return (sensor[5] == 1 || sensor[6] == 1 || sensor[7] == 1);
}

/**
 * Helper function to check if any center sensors are active
 */
bool isCenterSensorActive() {
  return (sensor[3] == 1 || sensor[4] == 1);
}

/**
 * Map a float value from one range to another
 */
float map(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * Print debug information to serial monitor
 */
void printDebugInfo() {
  Serial.print("Position: ");
  Serial.print(linePosition);
  Serial.print(" | Direction: ");
  Serial.print(lineDirection);
  Serial.print(" | Last: ");
  Serial.print(lastLineDirection);
  Serial.print(" | Straight: ");
  Serial.print(isGoingStraight ? "YES" : "NO");
  Serial.print(" | Time: ");
  if (isGoingStraight) {
    Serial.print((millis() - straightStartTime) / 1000.0, 1);
    Serial.print("s");
  } else {
    Serial.print("--");
  }
  Serial.print(" | Sensors: ");
  
  for (int i = 0; i < SENSOR_COUNT; i++) {
    Serial.print(sensor[i]);
  }
  
  Serial.println();
}

