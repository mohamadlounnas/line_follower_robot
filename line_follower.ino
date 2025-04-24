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
#define KD 40        // Derivative constant
#define BASE_SPEED 80 // Base motor speed
#define MAX_SPEED 120 // Maximum motor speed

// Sensor settings
#define NUM_SENSORS 8
#define SENSOR_THRESHOLD 760
#define IDEAL_POSITION 3.5  // Center position (for 8 sensors indexed 0-7)

// Variables for PID control
float lastError = 0;
float position = IDEAL_POSITION;

// Function prototypes
void readSensors(int *sensorValues, float *position);
void calculatePID(float currentPosition, float *pidValue);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void calibrateSensors();

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
  
  // Check if line is detected
  if (position >= 0) {
    // Calculate PID value
    calculatePID(position, &pidValue);
    
    // Calculate motor speeds
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
  } else {
    // Line lost - implement recovery behavior
    // For now, just stop
    setMotorSpeeds(0, 0);
    Serial.println("Line lost!");
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
    sensorValues[i] = analogRead(i);
    
    // Convert to binary (0 or 1)
    if (sensorValues[i] > SENSOR_THRESHOLD) {
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

