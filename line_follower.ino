/**
 * Line Follower Robot
 * 
 * This code implements a PID-based line following algorithm
 * for a robot with 8 IR reflectance sensors.
 * 
 * Hardware:
 * - 8 IR reflectance sensors on analog pins A0-A7
 * - Left motor: PWM on pins 9 (forward) and 6 (backward)
 * - Right motor: PWM on pins 10 (forward) and 11 (backward)
 */

// Motor pins
#define LEFT_MOTOR_FORWARD 9
#define LEFT_MOTOR_BACKWARD 6
#define RIGHT_MOTOR_FORWARD 10
#define RIGHT_MOTOR_BACKWARD 11

// PID Constants
#define KP 35        // Proportional constant
#define KD 20        // Derivative constant - REDUCED to prevent overcompensation
#define BASE_SPEED 80 // Base motor speed
#define MAX_SPEED 120 // Maximum motor speed
#define TURN_SPEED 100 // Speed when making recovery turns

// Sensor settings
#define NUM_SENSORS 8
#define SENSOR_THRESHOLD 760
#define IDEAL_POSITION 3.5  // Center position (for 8 sensors indexed 0-7)

// Line detection thresholds
#define LINE_THRESHOLD_LOW 1.0   // Below this is considered far left
#define LINE_THRESHOLD_HIGH 6.0  // Above this is considered far right

// Variables for PID control
float lastError = 0;
float position = IDEAL_POSITION;
int lastDirection = 0;  // Direction memory: -1 for left, 1 for right, 0 for straight
unsigned long lastLineTime = 0; // Time when line was last detected

// Function prototypes
void readSensors(int *sensorValues, float *position);
void calculatePID(float currentPosition, float *pidValue);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void recoverLine();
void printSensorValues(int *sensorValues);

void setup() {
  // Initialize motor pins as outputs
  pinMode(LEFT_MOTOR_FORWARD, OUTPUT);
  pinMode(LEFT_MOTOR_BACKWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_FORWARD, OUTPUT);
  pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT);
  
  // Setup serial communication for debugging
  Serial.begin(9600);
  
  // Stop motors initially
  setMotorSpeeds(0, 0);
  
  // Delay to allow time to position the robot
  delay(2000);
  
  Serial.println("Line Follower Ready!");
}

void loop() {
  int sensorValues[NUM_SENSORS];
  float position;
  float pidValue;
  
  // Read sensors and get line position
  readSensors(sensorValues, &position);
  
  // Print sensor values for debugging
  printSensorValues(sensorValues);
  
  // Check if line is detected
  if (position >= 0) {
    // Update last line detected time
    lastLineTime = millis();
    
    // Calculate PID value
    calculatePID(position, &pidValue);
    
    // Record direction for line recovery
    if (position < IDEAL_POSITION - 0.5) {
      // Line is to the left
      lastDirection = -1;
    } else if (position > IDEAL_POSITION + 0.5) {
      // Line is to the right
      lastDirection = 1;
    }
    
    // Handle extreme positions differently to prevent oscillation
    if (position < LINE_THRESHOLD_LOW) {
      // Far left - make a sharper left turn
      setMotorSpeeds(BASE_SPEED/2, BASE_SPEED*1.5);
      Serial.println("Sharp Left Turn");
    } else if (position > LINE_THRESHOLD_HIGH) {
      // Far right - make a sharper right turn
      setMotorSpeeds(BASE_SPEED*1.5, BASE_SPEED/2);
      Serial.println("Sharp Right Turn");
    } else {
      // Normal PID control
      int leftSpeed = BASE_SPEED + pidValue;
      int rightSpeed = BASE_SPEED - pidValue;
      
      // Set motor speeds
      setMotorSpeeds(leftSpeed, rightSpeed);
      
      // Debug output
      Serial.print("Position: ");
      Serial.print(position);
      Serial.print(" PID: ");
      Serial.print(pidValue);
      Serial.print(" Motors L/R: ");
      Serial.print(leftSpeed);
      Serial.print("/");
      Serial.println(rightSpeed);
    }
  } else {
    // Line lost - try to recover
    recoverLine();
  }
  
  // Short delay for stability
  delay(10);
}

/**
 * Reads sensor values and calculates the line position.
 * Returns a weighted average of the line position between 0 and 7.
 * Returns -1 if no line is detected.
 */
void readSensors(int *sensorValues, float *position) {
  float weightedSum = 0;
  int sum = 0;
  bool lineDetected = false;
  
  // Read all sensors
  for (int i = 0; i < NUM_SENSORS; i++) {
    // Read analog value and invert the threshold comparison 
    // (this works better with some sensor types)
    int rawValue = analogRead(i);
    sensorValues[i] = rawValue;
    
    // Convert to binary (0 or 1)
    if (rawValue > SENSOR_THRESHOLD) {
      sensorValues[i] = 1;
      lineDetected = true;
    } else {
      sensorValues[i] = 0;
    }
    
    // Calculate weighted sum
    weightedSum += sensorValues[i] * i;
    sum += sensorValues[i];
  }
  
  // Calculate position
  if (sum > 0) {
    *position = weightedSum / sum;
  } else {
    *position = -1; // No line detected
  }
}

/**
 * Calculates PID value based on current position and error.
 */
void calculatePID(float currentPosition, float *pidValue) {
  // Calculate error (how far from ideal position)
  float error = IDEAL_POSITION - currentPosition;
  
  // Calculate PD components
  float proportional = KP * error;
  float derivative = KD * (error - lastError);
  
  // Calculate total PID value (we're only using P and D)
  *pidValue = proportional + derivative;
  
  // Update last error for next iteration
  lastError = error;
}

/**
 * Print sensor values for debugging
 */
void printSensorValues(int *sensorValues) {
  Serial.print("Sensors: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorValues[i]);
    Serial.print(" ");
  }
  Serial.println();
}

/**
 * Try to recover the line when it's lost
 */
void recoverLine() {
  // Check how long we've been without a line
  unsigned long currentTime = millis();
  unsigned long timeSinceLastLine = currentTime - lastLineTime;
  
  if (timeSinceLastLine > 2000) {
    // If it's been more than 2 seconds, stop the robot
    setMotorSpeeds(0, 0);
    Serial.println("Line completely lost - stopping");
    return;
  }
  
  // Turn in the last known direction of the line
  Serial.print("Recovering line - turning ");
  
  if (lastDirection <= 0) { // Left or unknown
    Serial.println("left");
    setMotorSpeeds(-TURN_SPEED/2, TURN_SPEED);
  } else { // Right
    Serial.println("right");
    setMotorSpeeds(TURN_SPEED, -TURN_SPEED/2);
  }
}

/**
 * Sets the speed of both motors.
 */
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Constrain speeds to valid range
  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);
  
  // Set left motor speed and direction
  if (leftSpeed >= 0) {
    analogWrite(LEFT_MOTOR_FORWARD, leftSpeed);
    analogWrite(LEFT_MOTOR_BACKWARD, 0);
  } else {
    analogWrite(LEFT_MOTOR_FORWARD, 0);
    analogWrite(LEFT_MOTOR_BACKWARD, -leftSpeed);
  }
  
  // Set right motor speed and direction
  if (rightSpeed >= 0) {
    analogWrite(RIGHT_MOTOR_FORWARD, rightSpeed);
    analogWrite(RIGHT_MOTOR_BACKWARD, 0);
  } else {
    analogWrite(RIGHT_MOTOR_FORWARD, 0);
    analogWrite(RIGHT_MOTOR_BACKWARD, -rightSpeed);
  }
}

