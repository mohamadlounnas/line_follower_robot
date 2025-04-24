/**
 * Simple Line Follower Robot
 * 
 * This code provides a robust implementation of a line follower robot
 * using a bang-bang control strategy with max speed pulses.
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

// Motor parameters
#define MAX_SPEED 120     // Maximum speed for motors
#define STOP_SPEED 0      // Speed when stopped

// Sensor parameters
int SENSOR_THRESHOLD = 700;  // Threshold to distinguish black from white (adjust based on your sensors)
#define NUM_SENSORS 8        // Number of sensors being used
#define IDEAL_POSITION 3.5   // Ideal position (center point for 8 sensors)
#define MARGIN 0.5          // How far from ideal position before correction

// Line tracking states
#define ON_LINE 0           // Robot is on the line
#define LINE_LEFT 1         // Line is to the left
#define LINE_RIGHT 2        // Line is to the right
#define LINE_LOST 3         // Line is lost

// Global variables
int lineState = ON_LINE;    // Current line state
int leftMotorSpeed, rightMotorSpeed;
int sensorValues[NUM_SENSORS];   // Raw analog values
int sensorDigital[NUM_SENSORS];  // Converted to 0 or 1
float position = 0;              // Weighted average of sensor positions
char lastTurnDirection = 'N';    // 'L' for left, 'R' for right, 'N' for not set

// Pulse timing
unsigned long lastDirectionChange = 0;
#define MIN_PULSE_DURATION 30  // Minimum duration for a correction pulse (ms)

// Debug variables
unsigned long lastDebugTime = 0;
#define DEBUG_INTERVAL 100  // Print debug info every 100ms

void setup() {
  // Initialize motor control pins as outputs
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
  
  // Initialize serial communication for debugging
  if (DEBUG_MODE) {
    Serial.begin(9600);
    Serial.println(F("Line Follower Robot - Debug Mode"));
    Serial.println(F("-----------------------------"));
    Serial.println(F("Wait 5 seconds before calibration..."));
    Serial.println(F("BANG-BANG CONTROL WITH MAX SPEED PULSES"));
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
  
  // Run the main line following algorithm
  followLineBangBang();
  
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
  } else {
    // No line detected - use last known position
    // Position remains unchanged
  }
}

/**
 * Main line following algorithm using bang-bang control with pulses
 */
void followLineBangBang() {
  // Check if any sensor sees the line
  bool lineDetected = false;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorDigital[i] == 1) {
      lineDetected = true;
      break;
    }
  }
  
  if (lineDetected) {
    // Determine line position state
    if (position < (IDEAL_POSITION - MARGIN)) {
      // Line is to the left
      lineState = LINE_LEFT;
      lastTurnDirection = 'L';
    } 
    else if (position > (IDEAL_POSITION + MARGIN)) {
      // Line is to the right
      lineState = LINE_RIGHT;
      lastTurnDirection = 'R';
    } 
    else {
      // Line is centered
      lineState = ON_LINE;
    }
    
    // Apply bang-bang control with pulses
    applyBangBangControl();
  } 
  else {
    // Line is lost
    lineState = LINE_LOST;
    recoverLine();
  }
}

/**
 * Apply bang-bang control strategy based on line state
 */
void applyBangBangControl() {
  unsigned long currentTime = millis();
  
  switch (lineState) {
    case ON_LINE:
      // Line is centered - go straight at max speed
      setMotors(MAX_SPEED, MAX_SPEED);
      break;
      
    case LINE_LEFT:
      // Line is to the left - turn left
      // Stop left motor, run right motor at max speed
      setMotors(STOP_SPEED, MAX_SPEED);
      lastDirectionChange = currentTime;
      break;
      
    case LINE_RIGHT:
      // Line is to the right - turn right
      // Run left motor at max speed, stop right motor
      setMotors(MAX_SPEED, STOP_SPEED);
      lastDirectionChange = currentTime;
      break;
  }
  
  // Ensure minimum pulse duration for corrections
  if ((lineState == LINE_LEFT || lineState == LINE_RIGHT) && 
      (currentTime - lastDirectionChange < MIN_PULSE_DURATION)) {
    // Continue the correction for at least MIN_PULSE_DURATION milliseconds
    delay(MIN_PULSE_DURATION - (currentTime - lastDirectionChange));
  }
}

/**
 * Attempt to recover the line when it's lost
 */
void recoverLine() {
  if (DEBUG_MODE) {
    Serial.println(F("Line lost! Attempting recovery..."));
  }
  
  // First check if all sensors are on the line (junction or cross)
  bool allOnLine = true;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorDigital[i] == 0) {
      allOnLine = false;
      break;
    }
  }
  
  if (allOnLine) {
    // All sensors see black - could be a junction or cross
    // Stop briefly and then continue forward
    setMotors(STOP_SPEED, STOP_SPEED);
    delay(50);
    setMotors(MAX_SPEED, MAX_SPEED);
    return;
  }
  
  // Turn in the last known direction of the line
  if (lastTurnDirection == 'L') {
    // Turn left to find the line
    setMotors(-MAX_SPEED, MAX_SPEED);
  } else if (lastTurnDirection == 'R') {
    // Turn right to find the line
    setMotors(MAX_SPEED, -MAX_SPEED);
  } else {
    // No last direction known, rotate clockwise
    setMotors(MAX_SPEED, -MAX_SPEED);
  }
  
  // Keep turning until a sensor detects the line
  while (true) {
    readSensors();
    
    // Check if any middle sensor sees the line
    if (sensorDigital[3] == 1 || sensorDigital[4] == 1) {
      // Line found, resume normal following
      break;
    }
    
    // Check if any sensor sees the line
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (sensorDigital[i] == 1) {
        // Line found, resume normal following
        if (DEBUG_MODE) {
          Serial.println(F("Line recovered!"));
        }
        return;
      }
    }
    
    // Avoid blocking for too long
    delay(10);
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
  switch (lineState) {
    case ON_LINE:
      Serial.println(F("ON_LINE"));
      break;
    case LINE_LEFT:
      Serial.println(F("LINE_LEFT"));
      break;
    case LINE_RIGHT:
      Serial.println(F("LINE_RIGHT"));
      break;
    case LINE_LOST:
      Serial.println(F("LINE_LOST"));
      break;
  }
  
  // Print motor speeds
  Serial.print(F("Motors L/R: "));
  Serial.print(leftMotorSpeed);
  Serial.print("/");
  Serial.println(rightMotorSpeed);
  
  // Add separator for readability
  Serial.println(F("-----------------------------"));
}

