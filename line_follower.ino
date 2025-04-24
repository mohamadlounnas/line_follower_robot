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

// Turn handling
#define LEFT_TURN 0
#define RIGHT_TURN 1
#define NO_TURN 2
#define TURN_THRESHOLD 2.0  // How far from center indicates a turn
#define RECOVERY_SPEED 100  // Speed for recovery turns

// Sensor settings
#define NUM_SENSORS 8
#define SENSOR_THRESHOLD 760
#define IDEAL_POSITION 3.5  // Center position (for 8 sensors indexed 0-7)

// Variables for PID control
float lastError = 0;
float position = IDEAL_POSITION;
int lastTurn = NO_TURN;           // Remembers the last turn direction
unsigned long lineLastSeen = 0;   // Time when line was last seen
bool inRecoveryMode = false;      // Flag for recovery mode
int recoveryAttempts = 0;         // Count of recovery attempts

// Function prototypes
void readSensors(int *sensorValues, float *position);
void calculatePID(float currentPosition, float *pidValue);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
void recoverLine(int *sensorValues);
bool detectLShapeTurn(int *sensorValues, float position);

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
    // Reset recovery mode if we found the line
    if (inRecoveryMode) {
      inRecoveryMode = false;
      recoveryAttempts = 0;
      Serial.println("Line found during recovery!");
    }
    
    // Update time when line was last seen
    lineLastSeen = millis();
    
    // Check for L-shape turn
    if (detectLShapeTurn(sensorValues, position)) {
      // Handle turn specifically
      if (position < IDEAL_POSITION) {
        // L-turn to the left
        lastTurn = LEFT_TURN;
        Serial.println("L-Turn LEFT detected");
        setMotorSpeeds(0, RECOVERY_SPEED); // Sharp left turn
        delay(50); // Give time to make the turn
      } else {
        // L-turn to the right
        lastTurn = RIGHT_TURN;
        Serial.println("L-Turn RIGHT detected");
        setMotorSpeeds(RECOVERY_SPEED, 0); // Sharp right turn
        delay(50); // Give time to make the turn
      }
    } else {
      // Normal line following
      // Update last turn direction based on position (for recovery if needed later)
      if (position < IDEAL_POSITION - 1.0) {
        lastTurn = LEFT_TURN; // Line is on the left
      } else if (position > IDEAL_POSITION + 1.0) {
        lastTurn = RIGHT_TURN; // Line is on the right
      }
      
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
    }
  } else {
    // Line lost - enter recovery mode
    // Only enter recovery if the line was recently seen (avoid continuous recovery)
    if (!inRecoveryMode && millis() - lineLastSeen < 2000) {
      inRecoveryMode = true;
      Serial.println("Entering line recovery mode...");
    }
    
    if (inRecoveryMode) {
      recoverLine(sensorValues);
    } else {
      // Lost for too long, just stop
      setMotorSpeeds(0, 0);
      Serial.println("Line completely lost, stopping.");
    }
  }
  
  // Short delay for stability
  delay(10);
}

/**
 * Detects if the robot is encountering an L-shaped turn
 */
bool detectLShapeTurn(int *sensorValues, float position) {
  // Check for L-shape turn pattern
  // For L-turn to the left: only leftmost sensors see the line
  // For L-turn to the right: only rightmost sensors see the line
  
  // Count active sensors on left and right sides
  int leftActive = sensorValues[0] + sensorValues[1] + sensorValues[2];
  int rightActive = sensorValues[5] + sensorValues[6] + sensorValues[7];
  int centerActive = sensorValues[3] + sensorValues[4];
  
  // L-turn to the left: left sensors active, center/right inactive
  if (leftActive >= 2 && centerActive == 0 && rightActive == 0) {
    return true;
  }
  
  // L-turn to the right: right sensors active, center/left inactive
  if (rightActive >= 2 && centerActive == 0 && leftActive == 0) {
    return true;
  }
  
  // Check for extreme position (very far from center)
  if (position < 1.0 || position > 6.0) {
    return true;
  }
  
  return false;
}

/**
 * Attempts to recover the line when it's lost
 */
void recoverLine(int *sensorValues) {
  // Check if we reached maximum recovery attempts
  if (recoveryAttempts > 20) {
    setMotorSpeeds(0, 0);
    Serial.println("Max recovery attempts reached. Stopping.");
    inRecoveryMode = false;
    return;
  }
  
  recoveryAttempts++;
  
  // Use last turn direction to guide recovery
  if (lastTurn == LEFT_TURN) {
    // Turn left to find the line
    Serial.println("Recovering - turning LEFT");
    setMotorSpeeds(-RECOVERY_SPEED/2, RECOVERY_SPEED);
  } else if (lastTurn == RIGHT_TURN) {
    // Turn right to find the line
    Serial.println("Recovering - turning RIGHT");
    setMotorSpeeds(RECOVERY_SPEED, -RECOVERY_SPEED/2);
  } else {
    // No known direction, small clockwise rotation or backward
    Serial.println("Recovering - no direction");
    setMotorSpeeds(-RECOVERY_SPEED/2, RECOVERY_SPEED/2);
  }
  
  // Short delay to allow movement
  delay(50);
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

