/**
 * Simple and Robust Line Follower
 * Uses edge following approach - follows right edge of the line
 */

// Motor pins
#define RIGHT_MOTOR_FORWARD 10  // IN1
#define RIGHT_MOTOR_BACKWARD 11 // IN2
#define LEFT_MOTOR_FORWARD 9    // IN3
#define LEFT_MOTOR_BACKWARD 6   // IN4

// Constants for line following
#define SENSOR_COUNT 8
#define THRESHOLD 760           // Threshold to detect black line
#define TARGET_SENSOR 6         // Target sensor to keep on the line (right edge following)

// Speed settings
#define BASE_SPEED 95           // Normal speed
#define TURN_SPEED 75           // Speed during turns
#define SEARCH_TIMEOUT 2000     // Max time to search for line (ms)

// Time constants
#define STRAIGHT_RESET_TIME 200 // Time (ms) to reset direction memory after going straight

// Variables
int sensor[SENSOR_COUNT];       // Array to store sensor values
char lineDirection = 'N';       // 'L' for left, 'R' for right, 'N' for none
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
  // Read sensors
  readSensors();
  
  // Track straight movement for direction reset
  updateStraightTimer();
  
  // Determine what to do based on sensor readings
  if (isLineDetected()) {
    // Line is detected - continue with edge following
    followLineEdge();
  } else {
    // No line detected - handle line loss
    handleLineLoss();
  }
  
  // Debug output
  printDebugInfo();
}

/**
 * Track how long robot has been going straight to reset direction memory
 * Resets lastLineDirection after STRAIGHT_RESET_TIME (0.2s)
 */
void updateStraightTimer() {
  // Check if robot is currently moving straight (target sensor on line)
  if (sensor[TARGET_SENSOR] == 1 && !isTurning) {
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
 * Reads all IR sensors
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
  
  // Convert to binary (0/1)
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensor[i] = (sensor[i] > THRESHOLD) ? 1 : 0;
  }
  
  // Update last time line was detected if any sensor sees the line
  if (isLineDetected()) {
    lastLineTime = millis();
    
    // Update line direction based on which sensors detect the line
    updateLineDirection();
  }
}

/**
 * Updates the direction of the line based on sensor readings
 */
void updateLineDirection() {
  // If target sensor is on, we're good (on the right edge)
  if (sensor[TARGET_SENSOR] == 1) {
    lineDirection = 'C'; // centered on edge
  }
  // If sensors to the left of target are on, line is to the left
  else if (sensor[TARGET_SENSOR-1] == 1 || sensor[TARGET_SENSOR-2] == 1) {
    lineDirection = 'L';
    lastLineDirection = 'L';
  }
  // If sensor to the right of target is on, line is to the right
  else if (sensor[7] == 1) {
    lineDirection = 'R';
    lastLineDirection = 'R';
  }
  // If other sensors detect the line, determine direction
  else if (isLineDetected()) {
    // Count sensors on left vs right side
    int leftCount = 0;
    int rightCount = 0;
    
    for (int i = 0; i < TARGET_SENSOR-2; i++) {
      if (sensor[i] == 1) leftCount++;
    }
    
    for (int i = TARGET_SENSOR+1; i < SENSOR_COUNT; i++) {
      if (sensor[i] == 1) rightCount++;
    }
    
    if (leftCount > rightCount) {
      lineDirection = 'L';
      lastLineDirection = 'L';
    } else if (rightCount > 0) {
      lineDirection = 'R';
      lastLineDirection = 'R';
    }
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
 * Edge following function that keeps sensor TARGET_SENSOR on the line
 */
void followLineEdge() {
  // Check for special cases first
  
  // All black - might be a junction
  if (isAllBlack()) {
    delay(10); // Brief delay to confirm reading
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
  
  // Edge following logic
  if (sensor[TARGET_SENSOR] == 1) {
    // Target sensor on line - go straight
    setMotors(BASE_SPEED, BASE_SPEED);
    isTurning = false;
  }
  else if (sensor[TARGET_SENSOR-1] == 1 || sensor[TARGET_SENSOR-2] == 1) {
    // Line is to the left of target - turn left
    // Left wheel slower or stopped, right wheel at normal speed
    setMotors(0, BASE_SPEED);
    lastLineDirection = 'L';
    isTurning = true;
  }
  else if (sensor[7] == 1) {
    // Line is to the right of target - turn right
    // Right wheel slower or stopped, left wheel at normal speed
    setMotors(BASE_SPEED, 0);
    lastLineDirection = 'R';
    isTurning = true;
  }
  else {
    // No sensor on right edge - check other sensors
    int leftActive = 0;
    int rightActive = 0;
    
    // Count sensors active on left and right side
    for (int i = 0; i < 3; i++) {
      if (sensor[i] == 1) leftActive++;
    }
    
    for (int i = 3; i < TARGET_SENSOR; i++) {
      if (sensor[i] == 1) rightActive++;
    }
    
    if (leftActive > 0) {
      // Line is far to the left - sharp left turn
      setMotors(0, BASE_SPEED);
      lastLineDirection = 'L';
      isTurning = true;
    } else if (rightActive > 0) {
      // Line is between center and target - gentle right turn
      setMotors(BASE_SPEED, TURN_SPEED);
      lastLineDirection = 'R';
      isTurning = true;
    } else {
      // No clear indication - use last direction
      if (lastLineDirection == 'L') {
        setMotors(0, BASE_SPEED);
      } else if (lastLineDirection == 'R') {
        setMotors(BASE_SPEED, 0);
      } else {
        // No direction memory - go straight
        setMotors(BASE_SPEED, BASE_SPEED);
      }
      isTurning = true;
    }
  }
}

/**
 * Handle case where line is completely lost
 */
void handleLineLoss() {
  // If we just lost the line (within last 100ms), continue same direction
  if (millis() - lastLineTime < 100) {
    if (lastLineDirection == 'L') {
      // Turn left (left wheel stopped, right wheel forward)
      setMotors(0, BASE_SPEED);
    } else if (lastLineDirection == 'R') {
      // Turn right (right wheel stopped, left wheel forward)
      setMotors(BASE_SPEED, 0);
    } else {
      // No direction memory - search pattern
      searchPattern();
    }
  } else {
    // Line has been lost for a while - search pattern
    searchPattern();
  }
  
  isTurning = true;
}

/**
 * Search pattern to find the line when completely lost
 */
void searchPattern() {
  static unsigned long searchStartTime = 0;
  static boolean searchStarted = false;
  static char searchDirection = 'L';
  
  // Initialize search if we haven't already
  if (!searchStarted) {
    searchStartTime = millis();
    searchStarted = true;
    searchDirection = 'L'; // Start with left search
  }
  
  // Time-based search pattern
  unsigned long searchTime = millis() - searchStartTime;
  
  if (searchTime < 1000) {
    // First try left for 1 second
    setMotors(0, BASE_SPEED);
  } 
  else if (searchTime < 3000) {
    // Then try right for 2 seconds
    setMotors(BASE_SPEED, 0);
  }
  else {
    // Then alternate with wider sweeps
    if (searchDirection == 'L') {
      setMotors(0, BASE_SPEED);
      if (searchTime % 1500 == 0) searchDirection = 'R';
    } else {
      setMotors(BASE_SPEED, 0);
      if (searchTime % 1500 == 0) searchDirection = 'L';
    }
  }
  
  // If search has gone on too long, stop and reset
  if (searchTime > SEARCH_TIMEOUT) {
    stopMotors();
    searchStarted = false;
  }
  
  // If we found the line, reset search state
  if (isLineDetected()) {
    searchStarted = false;
  }
}

/**
 * Stop all motors
 */
void stopMotors() {
  setMotors(0, 0);
}

/**
 * Set motor speeds - forward only, no backward motion
 * @param leftSpeed Left motor speed (0-255)
 * @param rightSpeed Right motor speed (0-255)
 */
void setMotors(int leftSpeed, int rightSpeed) {
  // Ensure values are positive (forward only) and within range
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  // Left motor - forward only
  analogWrite(LEFT_MOTOR_FORWARD, leftSpeed);
  analogWrite(LEFT_MOTOR_BACKWARD, 0);
  
  // Right motor - forward only
  analogWrite(RIGHT_MOTOR_FORWARD, rightSpeed);
  analogWrite(RIGHT_MOTOR_BACKWARD, 0);
}

/**
 * Constrain a value between a minimum and maximum
 */
int constrain(int value, int min, int max) {
  if (value < min) return min;
  if (value > max) return max;
  return value;
}

/**
 * Print debug information to serial monitor
 */
void printDebugInfo() {
  Serial.print("Direction: ");
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
    if (i == TARGET_SENSOR) Serial.print("*"); // Mark target sensor
    else Serial.print(" ");
  }
  
  Serial.println();
}

