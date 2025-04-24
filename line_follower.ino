/**
 * Advanced Line Follower Robot
 * 
 * This implementation includes:
 * - Sensor calibration for reliable readings
 * - PID control for smooth line following
 * - Memory of last detected line position for recovery
 * - Robust line loss handling
 * 
 * Hardware Configuration:
 * - 8 IR sensors on analog pins A0-A7
 * - Motor pins: lmf(9), lmb(6), rmf(10), rmb(11)
 */

// Motor pins
#define rmf 10 // Right motor forward 
#define rmb 11 // Right motor backward
#define lmf 9  // Left motor forward
#define lmb 6  // Left motor backward

// PID control constants
#define Kp 0.2       // Proportional constant
#define Kd 2.0       // Derivative constant
#define BaseSpeed 80 // Base motor speed
#define MaxSpeed 120 // Maximum motor speed
#define TurnSpeed 100 // Speed for turning

// Sensor array configuration
#define NUM_SENSORS 8
#define LINE_POSITION_CENTER 3500 // Center position (3.5 * 1000)

// Thresholds for line detection
#define WHITE_THRESHOLD 100  // Low reading for white surface
#define BLACK_THRESHOLD 900  // High reading for black line
#define MIN_SENSORS_ON_LINE 1 // Minimum sensors required to consider on-line

// Calibration samples
#define CALIBRATION_SAMPLES 100

// Global variables
int sensorValues[NUM_SENSORS];     // Raw sensor readings
int sensorMin[NUM_SENSORS];        // Calibration - minimum values
int sensorMax[NUM_SENSORS];        // Calibration - maximum values
int lastPosition = LINE_POSITION_CENTER; // Last known line position
int lastError = 0;                 // Previous error for derivative term
char lastDirection = 'c';          // Last turning direction (l=left, r=right, c=center)
bool isCalibrated = false;         // Calibration flag

/**
 * Setup function - runs once at startup
 */
void setup() {
  // Initialize motor control pins
  pinMode(lmf, OUTPUT);
  pinMode(lmb, OUTPUT);
  pinMode(rmf, OUTPUT);
  pinMode(rmb, OUTPUT);
  
  // Initialize serial for debugging
  Serial.begin(9600);
  
  // Run calibration routine
  calibrateSensors();
}

/**
 * Main program loop
 */
void loop() {
  followLine();
}

/**
 * Calibrate sensors to adjust for ambient light conditions
 * Rotates robot during calibration to capture full sensor range
 */
void calibrateSensors() {
  Serial.println("Starting calibration...");
  
  // Initialize calibration values
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }
  
  // Rotate left and right to calibrate sensors
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    // Zigzag motion during calibration
    if (i < CALIBRATION_SAMPLES/2) {
      motor(TurnSpeed, -TurnSpeed); // Turn left
    } else {
      motor(-TurnSpeed, TurnSpeed); // Turn right
    }
    
    // Read sensors and update min/max values
    readSensors(false);
    for (int j = 0; j < NUM_SENSORS; j++) {
      if (sensorValues[j] < sensorMin[j]) {
        sensorMin[j] = sensorValues[j];
      }
      if (sensorValues[j] > sensorMax[j]) {
        sensorMax[j] = sensorValues[j];
      }
    }
    
    delay(10);
  }
  
  // Stop motors
  motor(0, 0);
  
  // Debug output of calibration values
  Serial.println("Calibration complete!");
  Serial.println("Min values:");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorMin[i]);
    Serial.print(" ");
  }
  Serial.println();
  
  Serial.println("Max values:");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorMax[i]);
    Serial.print(" ");
  }
  Serial.println();
  
  isCalibrated = true;
  
  // Wait for a moment after calibration
  delay(1000);
}

/**
 * Read all sensor values
 * @param normalize If true, normalize readings based on calibration data
 * @return true if at least one sensor detects the line
 */
bool readSensors(bool normalize) {
  int sensorsOnLine = 0;
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    // Read raw analog values, A0 is sensor[0], A7 is sensor[7]
    sensorValues[i] = analogRead(NUM_SENSORS - 1 - i);
    
    // Normalize based on calibration if needed
    if (normalize && isCalibrated) {
      // Constrain readings to calibration range
      sensorValues[i] = constrain(sensorValues[i], sensorMin[i], sensorMax[i]);
      // Map to 0-1000 range
      sensorValues[i] = map(sensorValues[i], sensorMin[i], sensorMax[i], 0, 1000);
      
      // Count sensors that detect the line
      if (sensorValues[i] > 500) {
        sensorsOnLine++;
      }
    }
    
    // Print sensor values for debugging
    Serial.print(sensorValues[i]);
    Serial.print("\t");
  }
  
  return (sensorsOnLine >= MIN_SENSORS_ON_LINE);
}

/**
 * Calculate the weighted position of the line
 * @return Position value from 0 to 7000, or last position if line is lost
 */
int getLinePosition() {
  long sum = 0;
  int weightedSum = 0;
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    int value = sensorValues[i];
    
    // Only consider readings that likely indicate line presence
    if (value > 200) {
      weightedSum += value * (i * 1000);
      sum += value;
    }
  }
  
  // Check if line is detected
  if (sum > 50) {
    lastPosition = weightedSum / sum;
    
    // Update direction memory based on line position
    if (lastPosition < LINE_POSITION_CENTER - 1000) {
      lastDirection = 'l'; // Line is on the left
    } else if (lastPosition > LINE_POSITION_CENTER + 1000) {
      lastDirection = 'r'; // Line is on the right
    }
    
    Serial.print("Line Position: ");
    Serial.println(lastPosition);
    return lastPosition;
  } else {
    // Line not found, return last known position
    Serial.print("Line lost! Last position: ");
    Serial.println(lastPosition);
    return lastPosition;
  }
}

/**
 * Main line following function using PID control
 */
void followLine() {
  // Read sensor values (normalized)
  bool onLine = readSensors(true);
  int position = getLinePosition();
  
  // Calculate error from center position
  int error = position - LINE_POSITION_CENTER;
  
  // Calculate PID components
  int proportional = error;
  int derivative = error - lastError;
  lastError = error;
  
  // Calculate PID output
  int pidOutput = (Kp * proportional) + (Kd * derivative);
  
  // Display PID information for debugging
  Serial.print("Error: ");
  Serial.print(error);
  Serial.print(" PID: ");
  Serial.println(pidOutput);
  
  // Determine motor speeds based on PID output
  int leftMotorSpeed, rightMotorSpeed;
  
  if (onLine) {
    // Line detected - normal PID control
    leftMotorSpeed = BaseSpeed + pidOutput;
    rightMotorSpeed = BaseSpeed - pidOutput;
    
    // Constrain speeds to valid range
    leftMotorSpeed = constrain(leftMotorSpeed, -MaxSpeed, MaxSpeed);
    rightMotorSpeed = constrain(rightMotorSpeed, -MaxSpeed, MaxSpeed);
    
    motor(leftMotorSpeed, rightMotorSpeed);
  } else {
    // Line lost - recovery behavior
    recoverLine();
  }
}

/**
 * Recovery behavior when line is lost
 */
void recoverLine() {
  // Stop briefly to stabilize
  motor(0, 0);
  delay(10);
  
  // Turn in the direction of the last known line position
  if (lastDirection == 'l') {
    // Line was on the left
    Serial.println("Recovering - turning left");
    motor(-TurnSpeed, TurnSpeed);
  } else if (lastDirection == 'r') {
    // Line was on the right
    Serial.println("Recovering - turning right");
    motor(TurnSpeed, -TurnSpeed);
  } else {
    // Unknown direction, search in a spiral pattern
    Serial.println("Recovering - searching pattern");
    
    // Alternate turning directions in a widening pattern
    static int searchStage = 0;
    static int searchTime = 100;
    
    if (searchStage % 2 == 0) {
      motor(TurnSpeed, -TurnSpeed);
    } else {
      motor(-TurnSpeed, TurnSpeed);
    }
    
    delay(searchTime);
    searchStage++;
    
    // Increase search radius over time
    if (searchStage % 2 == 0) {
      searchTime += 50;
    }
    
    // Reset if search gets too wide
    if (searchTime > 500) {
      searchTime = 100;
    }
  }
  
  // Check if we found the line again
  readSensors(true);
  getLinePosition();
}

/**
 * Control both motors
 * @param leftSpeed Left motor speed (-MaxSpeed to MaxSpeed)
 * @param rightSpeed Right motor speed (-MaxSpeed to MaxSpeed)
 */
void motor(int leftSpeed, int rightSpeed) {
  // Control left motor
  if (leftSpeed >= 0) {
    // Forward motion
    analogWrite(lmf, leftSpeed);
    analogWrite(lmb, 0);
  } else {
    // Backward motion
    analogWrite(lmf, 0);
    analogWrite(lmb, -leftSpeed);
  }
  
  // Control right motor
  if (rightSpeed >= 0) {
    // Forward motion
    analogWrite(rmf, rightSpeed);
    analogWrite(rmb, 0);
  } else {
    // Backward motion
    analogWrite(rmf, 0);
    analogWrite(rmb, -rightSpeed);
  }
}

