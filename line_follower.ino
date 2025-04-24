  //#define led_r 2
  //#define led_b 4
  //#define buzzer 3
  //Motor pins
  #define rmf 10 //IN1
  #define rmb 11 //IN2
  //#define rms 6  //EnA

  #define lmf 9  //IN3
  #define lmb 6  //IN2
  //#define lms 11//EnB

  // Sensor array with 8 IR sensors (A0-A7)
  int sensor[8];  //to store the sensor value
  int thresholds = 760; //threshold = (minimum analog value + Maximum Analog Value) / 2
  
  float c; // Position value (weighted average of sensor readings)
  int left_motor_speed = 80, right_motor_speed = 80;
  int left_motor, right_motor;
  int kp = 30, kd = 35; // PID constants for line following
  int PID_value;
  float current_error, previous_error;
  int turn_speed = 120;
  char last_direction = 'n'; // 'n' for none, 'l' for left, 'r' for right
  int maxs = 120; // Maximum motor speed
  
  // Variables to track last line position
  float last_position = 4.5; // Default center position
  unsigned long last_line_detected = 0; // Time when line was last detected
  unsigned long line_lost_timeout = 100; // Time in ms before considering line lost

 
  void setup() {
    // Set motor driver pins as output
    pinMode(lmf, OUTPUT);
    pinMode(lmb, OUTPUT);
    pinMode(rmf, OUTPUT);
    pinMode(rmb, OUTPUT);
    
    Serial.begin(9600);
  }


  void loop() {
    Line_Follow();  // Line follow using PID
  }

  /**
   * Reads sensor values and calculates the weighted position of the line
   * Maps analog readings to digital (0/1) values based on threshold
   * Calculates weighted average position from 1 to 8 (c variable)
   */
  void sensor_reading() {
    float a;
    float b;
        
    // Read all 8 IR sensors
    sensor[7] = analogRead(A7);
    sensor[6] = analogRead(A6);
    sensor[5] = analogRead(A5);
    sensor[4] = analogRead(A4);
    sensor[3] = analogRead(A3);
    sensor[2] = analogRead(A2);
    sensor[1] = analogRead(A1);
    sensor[0] = analogRead(A0);
        
    // Convert analog readings to digital (0/1)
    for (int i = 7; i >= 0; i--) {
      if (sensor[i] > thresholds) {
        sensor[i] = 1;
      } else {
        sensor[i] = 0;
      }
    }
    
    // Calculate weighted position (1 to 8)
    a = (sensor[0] * 1 + sensor[1] * 2 + sensor[2] * 3 + sensor[3] * 4 + sensor[4] * 5 + sensor[5] * 6 + sensor[6] * 7 + sensor[7] * 8);
    b = (sensor[0] + sensor[1] + sensor[2] + sensor[3] + sensor[4] + sensor[5] + sensor[6] + sensor[7]);
    
    if (b > 0) {
      c = a / b;
      last_position = c; // Remember the last valid line position
      last_line_detected = millis(); // Update time when line was last detected
    }
  }

  /**
   * Main line following function with PID control and decision making
   * Handles different cases based on sensor readings
   */
  void Line_Follow() {
    while (1) {
      sensor_reading();
      
      // Check if any sensor detects the line
      bool line_detected = false;
      for (int i = 0; i < 8; i++) {
        if (sensor[i] == 1) {
          line_detected = true;
          break;
        }
      }
      
      // Line is detected - use PID control for line following
      if (line_detected) {
        current_error = 4.5 - c;
        Serial.print(current_error);
        Serial.print("  ");
        PID_value = current_error * kp + kd * (current_error - previous_error);
        previous_error = current_error;
        Serial.println(PID_value);
        
        right_motor = right_motor_speed - PID_value;
        left_motor = left_motor_speed + PID_value;
        motor(left_motor, right_motor);
        
        // Update last direction based on the current position
        if (c < 4) {
          last_direction = 'l'; // Line is on the left
        } else if (c > 5) {
          last_direction = 'r'; // Line is on the right
        }
      }
      
      // All sensors on white surface (lost the line)
      else if (sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 0 && 
               sensor[4] == 0 && sensor[5] == 0 && sensor[6] == 0 && sensor[7] == 0) {
        
        // Check if we just recently lost the line
        if (millis() - last_line_detected < line_lost_timeout) {
          // Continue in the same direction as before (short-term memory)
          if (last_position < 4) {
            motor(turn_speed/2, turn_speed);  // Slight left turn
          } else if (last_position > 5) {
            motor(turn_speed, turn_speed/2);  // Slight right turn
          } else {
            motor(turn_speed, turn_speed);  // Go straight
          }
        }
        // Line has been lost for a while, use last_direction for recovery
        else {
          if (last_direction == 'r') {
            right();
          } 
          else if (last_direction == 'l') {
            left();
          }
          else {
            U_turn(); // If no memory of direction, do a U-turn
          }
        }
      }

      // Rightmost sensor is detecting and leftmost is not - prepare for right turn
      if (sensor[7] == 1 && sensor[0] == 0) {
        last_direction = 'r';
      }
      
      // Leftmost sensor is detecting and rightmost is not - prepare for left turn
      if (sensor[0] == 1 && sensor[7] == 0) {
        last_direction = 'l';
      }

      // All sensors detecting black - could be a junction or end of track
      if (sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 1 && 
          sensor[4] == 1 && sensor[5] == 1 && sensor[6] == 1 && sensor[7] == 1) {
        
        delay(30); // Short delay to confirm reading
        sensor_reading();
        
        // Still all black - stop and wait
        if ((sensor[0] + sensor[1] + sensor[2] + sensor[3] + sensor[4] + sensor[5] + sensor[6] + sensor[7]) == 8) {
          motor(0, 0); // Stop
          while ((sensor[0] + sensor[1] + sensor[2] + sensor[3] + sensor[4] + sensor[5] + sensor[6] + sensor[7]) == 8) {
            sensor_reading(); // Wait until conditions change
          }
        } 
        // If middle sensors show white but outer sensors show black, turn based on last direction
        else if (sensor[0] == 0 && sensor[7] == 0) {
          if (last_direction == 'l') {
            left();
          } else {
            right();
          }
        }
      }
    }
  }

  /**
   * Turn right until the center sensors detect the line
   */
  void right() {
    while (1) {
      motor(turn_speed, 0); // Turn right
      sensor_reading();
      // Wait until center sensors detect the line
      while (sensor[3] == 0 && sensor[4] == 0) {
        sensor_reading();
      }
      motor(0, turn_speed); // Apply small correction
      delay(35);
      break;
    }
  }

  /**
   * Turn left until the center sensors detect the line
   */
  void left() {
    while (1) {
      motor(0, turn_speed); // Turn left
      sensor_reading();
      // Wait until center sensors detect the line
      while (sensor[3] == 0 && sensor[4] == 0) {
        sensor_reading();
      }
      motor(turn_speed, 0); // Apply small correction
      delay(35);
      break;
    }
  }
  
  /**
   * Perform a 180-degree turn to find the line again
   */
  void U_turn() {
    while (1) {
      delay(120);
      motor(-turn_speed, turn_speed); // Rotate in place
      // Wait until center sensors detect the line
      while (sensor[3] == 0 && sensor[4] == 0) {
        sensor_reading();
      }
      motor(turn_speed, -turn_speed); // Apply small correction
      delay(20);
      break;
    }
  }
 
  /**
   * Control both motors with speed values
   * @param a Left motor speed (positive = forward, negative = backward)
   * @param b Right motor speed (positive = forward, negative = backward)
   */
  void motor(int a, int b) {
    // Left motor control
    if (a >= 0) {
      analogWrite(lmf, a > maxs ? maxs : a);
      analogWrite(lmb, 0);
    } else {
      analogWrite(lmf, 0);
      analogWrite(lmb, abs(a) > maxs ? maxs : abs(a));
    }
    
    // Right motor control
    if (b >= 0) {
      analogWrite(rmf, b > maxs ? maxs : b);
      analogWrite(rmb, 0);
    } else {
      analogWrite(rmf, 0);
      analogWrite(rmb, abs(b) > maxs ? maxs : abs(b));
    }
  }

  /**
   * Debug function to show analog sensor values
   */
  void show_analog_value() {
    for (short int i = 7; i >= 0; i--) {
      Serial.print(String(analogRead(i)) + " ");
    }
    delay(100);
    Serial.println();
  }

  /**
   * Debug function to show digital sensor values
   */
  void show_digital_value() {
    for (short int i = 7; i >= 0; i--) {
      if (analogRead(i) > thresholds) {
        sensor[i] = 1;
      } else {
        sensor[i] = 0;
      }
      Serial.print(sensor[i]);
      Serial.print("   ");
    }
    delay(100);
    Serial.println();
  }

