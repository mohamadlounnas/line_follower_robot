/**
 * Simple Line Follower Robot
 * 
 * This implementation includes:
 * - Fixed threshold for line detection (800)
 * - PID control for smooth line following
 * - Memory of last detected line position for recovery
 * - Forward-only motor movement
 * 
 * Hardware Configuration:
 * - 8 IR sensors on analog pins A0-A7
 * - Motor pins: lmf(10), lmb(11), rmf(9), rmb(6)
 */

// Motor pins
#define rmf 9  // Right motor forward 
#define rmb 6  // Right motor backward
#define lmf 10 // Left motor forward
#define lmb 11 // Left motor backward

// PID control constants
#define Kp 0.2       // Proportional constant
#define Kd 2.0       // Derivative constant
#define BaseSpeed 80 // Base motor speed
#define MaxSpeed 120 // Maximum motor speed
#define TurnSpeed 100 // Speed for turning

// Sensor configuration
#define NUM_SENSORS 8
#define LINE_POSITION_CENTER 3500 // Center position (3.5 * 1000)
#define THRESHOLD 800           // Fixed threshold for line detection
#define MIN_SENSORS_ON_LINE 1   // Minimum sensors required to consider on-line

// Global variables
int sensorValues[NUM_SENSORS];     // Raw sensor readings
int lastPosition = LINE_POSITION_CENTER; // Last known line position
int lastError = 0;                 // Previous error for derivative term
char lastDirection = 'c';          // Last turning direction (l=left, r=right, c=center)

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
  
  // Briefly pause before starting
  delay(1000);
  Serial.println("Line follower initialized with threshold: 800");
}

/**
 * Main program loop
 */
void loop() {
  followLine();
}

/**
 * Read all sensor values using the fixed threshold
 * @return true if at least one sensor detects the line
 */
bool readSensors() {
  int sensorsOnLine = 0;
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    // Read raw analog values, A0 is sensor[0], A7 is sensor[7]
    sensorValues[i] = analogRead(NUM_SENSORS - 1 - i);
    
    // Print sensor values for debugging
    Serial.print(sensorValues[i]);
    Serial.print("\t");
    
    // Count sensors that detect the line (above threshold)
    if (sensorValues[i] > THRESHOLD) {
      sensorsOnLine++;
    }
  }
  
  Serial.println();
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
    
    // Only consider readings above the threshold
    if (value > THRESHOLD) {
      weightedSum += value * (i * 1000);
      sum += value;
    }
  }
  
  // Check if line is detected
  if (sum > 0) {
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
  // Read sensor values and determine line position
  bool onLine = readSensors();
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
    
    // Constrain speeds to valid range (forward motion only)
    leftMotorSpeed = constrain(leftMotorSpeed, 0, MaxSpeed);
    rightMotorSpeed = constrain(rightMotorSpeed, 0, MaxSpeed);
    
    motor(leftMotorSpeed, rightMotorSpeed);
  } else {
    // Line lost - recovery behavior
    recoverLine();
  }
}

/**
 * Recovery behavior when line is lost
 * Only uses forward motion for recovery
 */
void recoverLine() {
  // Turn in the direction of the last known line position (forward-only turning)
  if (lastDirection == 'l') {
    // Line was on the left - turn left by slowing right motor
    Serial.println("Recovering - turning left");
    motor(0, TurnSpeed);
  } else if (lastDirection == 'r') {
    // Line was on the right - turn right by slowing left motor
    Serial.println("Recovering - turning right");
    motor(TurnSpeed, 0);
  } else {
    // Unknown direction, use right-biased search
    Serial.println("Recovering - searching");
    
    // Alternate search directions with varying intensities
    static int searchStage = 0;
    
    if (searchStage % 4 == 0) {
      motor(TurnSpeed, TurnSpeed/4);  // Gentle right turn
    } else if (searchStage % 4 == 1) {
      motor(TurnSpeed/4, TurnSpeed);  // Gentle left turn
    } else if (searchStage % 4 == 2) {
      motor(TurnSpeed, 0);           // Sharp right turn
    } else {
      motor(0, TurnSpeed);           // Sharp left turn
    }
    
    delay(100);
    searchStage++;
    
    // Reset search stage after a while to prevent getting stuck
    if (searchStage > 20) {
      searchStage = 0;
    }
  }
  
  // Check if we found the line again
  readSensors();
  getLinePosition();
}

/**
 * Control both motors (forward motion only)
 * @param leftSpeed Left motor speed (0 to MaxSpeed)
 * @param rightSpeed Right motor speed (0 to MaxSpeed)
 */
void motor(int leftSpeed, int rightSpeed) {
  // Ensure speeds are non-negative (forward motion only)
  leftSpeed = max(0, leftSpeed);
  rightSpeed = max(0, rightSpeed);
  
  // Apply motor speeds
  analogWrite(lmf, leftSpeed);
  analogWrite(lmb, 0);  // Always 0 for forward-only motion
  
  analogWrite(rmf, rightSpeed);
  analogWrite(rmb, 0);  // Always 0 for forward-only motion
  
  Serial.print("Motors: L=");
  Serial.print(leftSpeed);
  Serial.print(" R=");
  Serial.println(rightSpeed);
}

