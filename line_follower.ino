/**
 * Advanced Line Follower Robot
 * 
 * This code provides a sophisticated implementation of a line follower robot
 * using an enhanced bang-bang control strategy with advanced recovery,
 * dynamic speed control, and anti-overshoot techniques.
 * 
 * Hardware Configuration:
 * - 8 IR sensors (A0-A7)
 * - Left motor: pins 9 (forward), 6 (backward)
 * - Right motor: pins 10 (forward), 11 (backward)
 */

// Motor pins
#define RIGHT_MOTOR_FORWARD 10  // IN1
#define RIGHT_MOTOR_BACKWARD 11 // IN2
#define LEFT_MOTOR_FORWARD 9    // IN3
#define LEFT_MOTOR_BACKWARD 6   // IN4

// Debug mode
#define DEBUG_MODE true  // Set to true for serial debugging information

// Motor speed parameters
#define MAX_SPEED 80           // Maximum forward speed
#define TURN_SPEED 70          // Speed during regular turns
#define SLOW_SPEED 40          // Speed during sharp or difficult turns
#define RECOVERY_SPEED 60      // Speed during recovery
#define BRAKE_SPEED 50         // Speed used for active braking (backward motion)
#define APPROACH_SPEED 30      // Speed when approaching the line during recovery
#define STOP_SPEED 0           // Complete stop

// Advanced turning parameters
#define QUICK_BRAKE_DURATION 40    // Duration of brake pulse in milliseconds
#define BACKWARD_TURN_DURATION 60  // Duration of backward turning when recovering
#define MIN_TURN_DURATION 50       // Minimum turn duration to ensure effect
#define MAX_TURN_DURATION 150      // Maximum turn duration before checking sensors again

// Sensor parameters
int SENSOR_THRESHOLD = 700;    // Threshold to distinguish black from white
#define NUM_SENSORS 8          // Number of sensors being used
#define IDEAL_POSITION 3.5     // Ideal position (center point for 8 sensors)
#define MARGIN 0.6             // How far from ideal position before correction

// Line tracking states
#define ON_LINE 0              // Robot is centered on the line
#define LINE_LEFT 1            // Line is to the left
#define LINE_RIGHT 2           // Line is to the right
#define LINE_LOST 3            // Line is completely lost
#define SHARP_LEFT 4           // Line requires a sharp left turn
#define SHARP_RIGHT 5          // Line requires a sharp right turn
#define JUNCTION 6             // Junction detected (all sensors triggered)

// Recovery and history
#define HISTORY_SIZE 5         // Number of recent positions to remember
float positionHistory[HISTORY_SIZE];  // Array to store recent positions
int historyIndex = 0;          // Current index in history array
bool wasOnLine = false;        // Whether the robot was on the line in previous iteration
unsigned long lastLineDetection = 0;  // Time when line was last detected
#define MAX_RECOVERY_TIME 3000 // Maximum time to spend in recovery before trying a new strategy
#define RECOVERY_CHECK_INTERVAL 300 // How often to check sensors during extended recovery

// Global variables
int lineState = ON_LINE;       // Current line state
int previousLineState = ON_LINE; // Previous line state
int leftMotorSpeed, rightMotorSpeed;
int sensorValues[NUM_SENSORS];   // Raw analog values
int sensorDigital[NUM_SENSORS];  // Converted to 0 or 1
float position = 0;              // Weighted average of sensor positions
char lastTurnDirection = 'N';    // 'L' for left, 'R' for right, 'N' for not set
unsigned long stateChangeTime = 0; // When the line state last changed
bool overshootDetected = false;  // Flag for when overshoot is detected
int recoveryPhase = 0;          // Phase of the recovery process
unsigned long recoveryStartTime = 0; // When recovery started

// Debug variables
unsigned long lastDebugTime = 0;
#define DEBUG_INTERVAL 100     // Print debug info every 100ms

void setup() {
  // Initialize motor control pins as outputs
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
  
  // Initialize serial communication for debugging
  if (DEBUG_MODE) {
    Serial.begin(9600);
    Serial.println(F("Advanced Line Follower Robot - Debug Mode"));
    Serial.println(F("-----------------------------"));
    Serial.println(F("Wait 5 seconds before calibration..."));
    Serial.println(F("ADVANCED CONTROL WITH ANTI-OVERSHOOT"));
  }
  
  // Initialize position history
  for (int i = 0; i < HISTORY_SIZE; i++) {
    positionHistory[i] = IDEAL_POSITION;
  }
  
  // Delay before starting (gives time to place robot on the line)
  delay(5000);
  
  // Perform initial sensor calibration
  calibrateSensors();
  
  // Stop motors at startup
  setMotors(STOP_SPEED, STOP_SPEED);
}

void loop() {
  // Read sensors and calculate position
  readSensors();
  
  // Store position history
  recordPositionHistory();
  
  // Determine line state and follow the line
  determineLineState();
  followLine();
  
  // Print debug information if enabled
  if (DEBUG_MODE) {
    printDebugInfo();
  }
}

/**
 * Read sensor values and calculate robot position
 */
void readSensors() {
  float weightedSum = 0;
  float sum = 0;
  
  // Read all sensors
  for (int i = 0; i < NUM_SENSORS; i++) {
    // Read analog value
    sensorValues[i] = analogRead(i);
    
    // Convert to digital (0 or 1)
    if (sensorValues[i] > SENSOR_THRESHOLD) {
      sensorDigital[i] = 1;  // Black line detected
    } else {
      sensorDigital[i] = 0;  // White surface
    }
    
    // Calculate weighted sum for position
    weightedSum += sensorDigital[i] * (i + 1);  // Weight by sensor position (1-8)
    sum += sensorDigital[i];                    // Total sensors that see the line
  }
  
  // Calculate position if at least one sensor detects the line
  if (sum > 0) {
    position = weightedSum / sum;
    lastLineDetection = millis();
    wasOnLine = true;
  } else {
    // No line detected - position remains unchanged
    wasOnLine = false;
  }
}

/**
 * Store the recent position history for recovery purposes
 */
void recordPositionHistory() {
  // Only record history if line is detected
  if (wasOnLine) {
    positionHistory[historyIndex] = position;
    historyIndex = (historyIndex + 1) % HISTORY_SIZE;
  }
}

/**
 * Analyze sensor readings to determine the current line state
 */
void determineLineState() {
  previousLineState = lineState;
  
  // Check if any sensor sees the line
  bool lineDetected = false;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorDigital[i] == 1) {
      lineDetected = true;
      break;
    }
  }
  
  if (lineDetected) {
    // Check for junction (all or most sensors active)
    int activeCount = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
      activeCount += sensorDigital[i];
    }
    
    if (activeCount >= 6) {
      lineState = JUNCTION;
    }
    // Check for sharp turns (only edge sensors active)
    else if (sensorDigital[0] == 1 && sensorDigital[1] == 1 && sensorDigital[NUM_SENSORS-1] == 0) {
      lineState = SHARP_LEFT;
      lastTurnDirection = 'L';
    }
    else if (sensorDigital[NUM_SENSORS-1] == 1 && sensorDigital[NUM_SENSORS-2] == 1 && sensorDigital[0] == 0) {
      lineState = SHARP_RIGHT;
      lastTurnDirection = 'R';
    }
    // Regular positioning
    else if (position < (IDEAL_POSITION - MARGIN)) {
      lineState = LINE_LEFT;
      lastTurnDirection = 'L';
    }
    else if (position > (IDEAL_POSITION + MARGIN)) {
      lineState = LINE_RIGHT;
      lastTurnDirection = 'R';
    }
    else {
      lineState = ON_LINE;
    }
    
    // Reset recovery phase when line is detected
    recoveryPhase = 0;
  }
  else {
    lineState = LINE_LOST;
  }
  
  // Record when the state changed
  if (lineState != previousLineState) {
    stateChangeTime = millis();
  }
  
  // Detect potential overshoot
  detectOvershoot();
}

/**
 * Detect potential overshoot conditions based on sudden state changes
 */
void detectOvershoot() {
  overshootDetected = false;
  
  // Detect when the robot crosses from one side to the other very quickly
  // This indicates potential overshoot
  if ((previousLineState == LINE_LEFT && lineState == LINE_RIGHT) || 
      (previousLineState == LINE_RIGHT && lineState == LINE_LEFT)) {
    overshootDetected = true;
    if (DEBUG_MODE) {
      Serial.println(F("OVERSHOOT DETECTED!"));
    }
  }
}

/**
 * Main line following algorithm with anti-overshoot capabilities
 */
void followLine() {
  unsigned long currentTime = millis();
  
  switch (lineState) {
    case ON_LINE:
      // Centered on line - go straight
      setMotors(MAX_SPEED, MAX_SPEED);
      break;
      
    case LINE_LEFT:
      if (overshootDetected) {
        // Apply anti-overshoot correction with brief brake
        applyBrake();
        setMotors(SLOW_SPEED, TURN_SPEED);
      } else {
        // Normal left correction
        setMotors(STOP_SPEED, TURN_SPEED);
        
        // Apply brief backward motion after correction threshold
        if (currentTime - stateChangeTime > MIN_TURN_DURATION) {
          // Check if correction has been active long enough
          if (currentTime - stateChangeTime > MAX_TURN_DURATION) {
            readSensors();
            determineLineState();
          }
        }
      }
      break;
      
    case LINE_RIGHT:
      if (overshootDetected) {
        // Apply anti-overshoot correction with brief brake
        applyBrake();
        setMotors(TURN_SPEED, SLOW_SPEED);
      } else {
        // Normal right correction
        setMotors(TURN_SPEED, STOP_SPEED);
        
        // Apply brief backward motion after correction threshold
        if (currentTime - stateChangeTime > MIN_TURN_DURATION) {
          // Check if correction has been active long enough
          if (currentTime - stateChangeTime > MAX_TURN_DURATION) {
            readSensors();
            determineLineState();
          }
        }
      }
      break;
      
    case SHARP_LEFT:
      // Execute a sharp left turn with active braking to prevent overshoot
      applyBrake();
      setMotors(-BRAKE_SPEED, TURN_SPEED); // Apply backward motion to left motor
      delay(50); // Allow time for the turn
      break;
      
    case SHARP_RIGHT:
      // Execute a sharp right turn with active braking to prevent overshoot
      applyBrake();
      setMotors(TURN_SPEED, -BRAKE_SPEED); // Apply backward motion to right motor
      delay(50); // Allow time for the turn
      break;
      
    case JUNCTION:
      // At a junction, slow down briefly then continue forward
      setMotors(SLOW_SPEED, SLOW_SPEED);
      delay(100);
      setMotors(MAX_SPEED, MAX_SPEED);
      break;
      
    case LINE_LOST:
      // Start recovery process
      if (recoveryPhase == 0) {
        recoveryStartTime = currentTime;
        recoveryPhase = 1;
      }
      advancedRecovery(currentTime);
      break;
  }
}

/**
 * Apply a quick brake by briefly reversing the motors
 */
void applyBrake() {
  // Apply reverse power to both motors briefly to counter momentum
  setMotors(-BRAKE_SPEED, -BRAKE_SPEED);
  delay(QUICK_BRAKE_DURATION);
  setMotors(STOP_SPEED, STOP_SPEED);
  delay(20); // Brief pause after braking
}

/**
 * Advanced recovery process with multiple strategies
 */
void advancedRecovery(unsigned long currentTime) {
  // If we just lost the line, first try to find it based on the last known direction
  if (recoveryPhase == 1) {
    if (DEBUG_MODE) {
      Serial.println(F("Recovery Phase 1: Initial Direction Search"));
    }
    
    // First try a gentle direction search
    if (lastTurnDirection == 'L') {
      setMotors(APPROACH_SPEED, RECOVERY_SPEED);
    } else if (lastTurnDirection == 'R') {
      setMotors(RECOVERY_SPEED, APPROACH_SPEED);
    } else {
      // If no known direction, try a small back-and-forth movement
      setMotors(-SLOW_SPEED, SLOW_SPEED);
      delay(50);
      setMotors(SLOW_SPEED, -SLOW_SPEED);
      delay(50);
    }
    
    // Check if line is found
    readSensors();
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (sensorDigital[i] == 1) {
        if (DEBUG_MODE) {
          Serial.println(F("Line found in Phase 1"));
        }
        return;
      }
    }
    
    // If we've spent enough time in phase 1, move to phase 2
    if (currentTime - recoveryStartTime > 500) {
      recoveryPhase = 2;
      if (DEBUG_MODE) {
        Serial.println(F("Moving to Recovery Phase 2"));
      }
    }
  }
  
  // Phase 2: Try backtracking to last known good position
  else if (recoveryPhase == 2) {
    if (DEBUG_MODE) {
      Serial.println(F("Recovery Phase 2: Backtracking"));
    }
    
    // Back up slightly
    setMotors(-SLOW_SPEED, -SLOW_SPEED);
    delay(200);
    setMotors(STOP_SPEED, STOP_SPEED);
    
    // Try rotating based on the last known position
    float lastGoodPosition = positionHistory[(historyIndex + HISTORY_SIZE - 1) % HISTORY_SIZE];
    
    if (lastGoodPosition < IDEAL_POSITION) {
      // Line was to the left
      setMotors(-SLOW_SPEED, RECOVERY_SPEED);
    } else {
      // Line was to the right
      setMotors(RECOVERY_SPEED, -SLOW_SPEED);
    }
    
    // Check sensors periodically
    delay(100);
    readSensors();
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (sensorDigital[i] == 1) {
        if (DEBUG_MODE) {
          Serial.println(F("Line found in Phase 2"));
        }
        return;
      }
    }
    
    // If we've spent enough time in phase 2, move to phase 3
    if (currentTime - recoveryStartTime > 1500) {
      recoveryPhase = 3;
      if (DEBUG_MODE) {
        Serial.println(F("Moving to Recovery Phase 3"));
      }
    }
  }
  
  // Phase 3: Systematic sweep
  else if (recoveryPhase == 3) {
    if (DEBUG_MODE) {
      Serial.println(F("Recovery Phase 3: Systematic Sweep"));
    }
    
    // Perform a systematic full rotation to find the line
    // Start with the last known direction
    if (lastTurnDirection == 'L' || currentTime % 2000 < 1000) {
      setMotors(-RECOVERY_SPEED, RECOVERY_SPEED);
    } else {
      setMotors(RECOVERY_SPEED, -RECOVERY_SPEED);
    }
    
    // Check sensors periodically
    if (currentTime % RECOVERY_CHECK_INTERVAL < 20) {
      readSensors();
      for (int i = 0; i < NUM_SENSORS; i++) {
        if (sensorDigital[i] == 1) {
          if (DEBUG_MODE) {
            Serial.println(F("Line found in Phase 3"));
          }
          // Apply brake to stop rotation when line is found
          applyBrake();
          return;
        }
      }
    }
    
    // If we've spent too long in recovery, reset and try a different strategy
    if (currentTime - recoveryStartTime > MAX_RECOVERY_TIME) {
      // Reset recovery and change the direction
      recoveryPhase = 1;
      recoveryStartTime = currentTime;
      if (lastTurnDirection == 'L') {
        lastTurnDirection = 'R';
      } else {
        lastTurnDirection = 'L';
      }
      
      if (DEBUG_MODE) {
        Serial.println(F("Reset recovery - changing direction"));
      }
    }
  }
}

/**
 * Calibrate sensors to determine appropriate threshold
 */
void calibrateSensors() {
  if (DEBUG_MODE) {
    Serial.println(F("Starting sensor calibration..."));
  }
  
  int minValues[NUM_SENSORS];
  int maxValues[NUM_SENSORS];
  
  // Initialize min values high and max values low
  for (int i = 0; i < NUM_SENSORS; i++) {
    minValues[i] = 1023;
    maxValues[i] = 0;
  }
  
  // Calibrate for 2.5 seconds
  for (int i = 0; i < 100; i++) {
    // Read each sensor
    for (int j = 0; j < NUM_SENSORS; j++) {
      int value = analogRead(j);
      
      // Update min and max values
      if (value < minValues[j]) {
        minValues[j] = value;
      }
      if (value > maxValues[j]) {
        maxValues[j] = value;
      }
    }
    delay(25);
  }
  
  // Calculate and set threshold (average of min and max)
  int sum = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    int threshold = (minValues[i] + maxValues[i]) / 2;
    sum += threshold;
    
    if (DEBUG_MODE) {
      Serial.print(F("Sensor "));
      Serial.print(i);
      Serial.print(F(" min: "));
      Serial.print(minValues[i]);
      Serial.print(F(" max: "));
      Serial.print(maxValues[i]);
      Serial.print(F(" threshold: "));
      Serial.println(threshold);
    }
  }
  
  // Set global threshold to average of all sensors
  SENSOR_THRESHOLD = sum / NUM_SENSORS;
  
  if (DEBUG_MODE) {
    Serial.print(F("Global threshold set to: "));
    Serial.println(SENSOR_THRESHOLD);
    Serial.println(F("Calibration complete!"));
  }
}

/**
 * Set motor speeds with safety checks
 */
void setMotors(int leftSpeed, int rightSpeed) {
  // Constrain speeds to prevent overflow
  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);
  
  // Save current motor speeds for debugging
  leftMotorSpeed = leftSpeed;
  rightMotorSpeed = rightSpeed;
  
  // Set left motor
  if (leftSpeed >= 0) {
    analogWrite(LEFT_MOTOR_FORWARD, leftSpeed);
    analogWrite(LEFT_MOTOR_BACKWARD, 0);
  } else {
    analogWrite(LEFT_MOTOR_FORWARD, 0);
    analogWrite(LEFT_MOTOR_BACKWARD, -leftSpeed);
  }
  
  // Set right motor
  if (rightSpeed >= 0) {
    analogWrite(RIGHT_MOTOR_FORWARD, rightSpeed);
    analogWrite(RIGHT_MOTOR_BACKWARD, 0);
  } else {
    analogWrite(RIGHT_MOTOR_FORWARD, 0);
    analogWrite(RIGHT_MOTOR_BACKWARD, -rightSpeed);
  }
}

/**
 * Print debug information to Serial
 */
void printDebugInfo() {
  // Limit how often we print debug information
  unsigned long currentTime = millis();
  if (currentTime - lastDebugTime < DEBUG_INTERVAL) {
    return;
  }
  lastDebugTime = currentTime;
  
  // Print sensor values
  Serial.print(F("Sensors: "));
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorValues[i]);
    Serial.print("(");
    Serial.print(sensorDigital[i]);
    Serial.print(") ");
  }
  Serial.println();
  
  // Print position and line state
  Serial.print(F("Position: "));
  Serial.print(position);
  Serial.print(F(" Line State: "));
  
  String stateName;
  switch (lineState) {
    case ON_LINE:
      stateName = "ON_LINE";
      break;
    case LINE_LEFT:
      stateName = "LINE_LEFT";
      break;
    case LINE_RIGHT:
      stateName = "LINE_RIGHT";
      break;
    case LINE_LOST:
      stateName = "LINE_LOST (Phase ";
      stateName += recoveryPhase;
      stateName += ")";
      break;
    case SHARP_LEFT:
      stateName = "SHARP_LEFT";
      break;
    case SHARP_RIGHT:
      stateName = "SHARP_RIGHT";
      break;
    case JUNCTION:
      stateName = "JUNCTION";
      break;
  }
  Serial.println(stateName);
  
  // Print motor speeds
  Serial.print(F("Motors L/R: "));
  Serial.print(leftMotorSpeed);
  Serial.print("/");
  Serial.println(rightMotorSpeed);
  
  // Add separator for readability
  Serial.println(F("-----------------------------"));
}

