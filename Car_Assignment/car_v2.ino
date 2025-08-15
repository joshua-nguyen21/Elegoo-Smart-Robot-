#include <avr/io.h>
#include <Servo.h>

#define LEFT_SENSOR_PIN A2    // Left sensor
#define CENTER_SENSOR_PIN A1  // Center sensor
#define RIGHT_SENSOR_PIN A0   // Right sensor
#define NUM_SENSORS 3         // Number of sensors
#define echo 12               // Echo input to read pulse
#define trigger 13            // Trigger output to send pulse
#define ENA 5                 // Enable A (PWM)
#define ENB 6                 // Enable B (PWM)
#define INB 7                 // Right Motors (Direction)
#define INA 8                 // Left Motors (Direction)
#define SRV 10                // Servo motor pin
#define STBY 3                // Must be on to drive

// Distance variables
int rd = 0; // right distance
int ld = 0; // left distance
int fd = 0; // forward distance

// Timing and thresholds
int rtdelay = 610;       // Right turn delay (ms)
int ltdelay = 600;       // Left turn delay (ms)
int stop_distance = 11;  // Stop distance (cm) - Increased from 11
int rdelay = 600;        // Reverse delay (ms)
Servo servo;
int line_state = 1;      // 1 = line following, 0 = obstacle avoidance

// PID Constants (tune these for your car)
#define KP 0.2   // Proportional gain
#define KI 0.0   // Integral gain
#define KD 2.5   // Derivative gain

// Speed settings
#define BASE_SPEED 50  // Base speed (0-255)
#define MAX_SPEED 90   // Maximum speed
int rightSpeed = 0;
int leftSpeed = 0;

// Threshold for line detection (adjust based on surface)
#define LINE_THRESHOLD 500  // Higher = more sensitive to dark lines

// Global variables
int lastError = 0;
int integral = 0;

// ---- Helper Functions ----

// Check if any sensor detects the line
bool checkForLine() {
  int leftVal = analogRead(LEFT_SENSOR_PIN);
  int centerVal = analogRead(CENTER_SENSOR_PIN);
  int rightVal = analogRead(RIGHT_SENSOR_PIN);
  return (leftVal > LINE_THRESHOLD || centerVal > LINE_THRESHOLD || rightVal > LINE_THRESHOLD);
}

// Calculate line position (weighted average)
int calculateLinePosition(int sensorValues[]) {
  int position = 0;
  int sum = 0;
  int onCount = 0;
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorValues[i] > LINE_THRESHOLD) {  // Sensor sees line
      position += (i * 1000);  // Weight positions (0, 1000, 2000)
      onCount++;
    }
  }

  if (onCount > 0) {
    position /= onCount;  // Average of active sensors
  }

  // If all sensors see the line (junction detected)
  if (onCount == 3) {
    stop();
    delay(3000);
    forward();
    delay(500);  // Increased forward time after junction
  }
  else {
    line_state = 1;  // Stay in line-following mode
    forward();
    delay(500);
  }
  
  return position;
}

// PID Controller
int calculatePID(int error) {
  // Proportional term
  int proportional = KP * error;
  
  // Integral term (with anti-windup)
  integral += error;
  integral = constrain(integral, -1000, 1000);  // Prevent integral windup
  int integralTerm = KI * integral;
  
  // Derivative term
  int derivative = error - lastError;
  int derivativeTerm = KD * derivative;
  lastError = error;
  
  return proportional + integralTerm + derivativeTerm;
}

// ---- Motor Control Functions ----

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  digitalWrite(STBY, HIGH);
  // Left motor
  digitalWrite(INA, leftSpeed > 0 ? HIGH : LOW);
  analogWrite(ENA, abs(leftSpeed));
  // Right motor
  digitalWrite(INB, rightSpeed > 0 ? HIGH : LOW);
  analogWrite(ENB, abs(rightSpeed));
}

void stop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(STBY, LOW);
}

void forward() {
  digitalWrite(STBY, HIGH);
  analogWrite(ENA, BASE_SPEED);
  analogWrite(ENB, BASE_SPEED);
  digitalWrite(INA, HIGH);
  digitalWrite(INB, HIGH);
}

void left() {
  digitalWrite(STBY, HIGH);
  analogWrite(ENA, 100);
  analogWrite(ENB, 100);
  digitalWrite(INA, LOW);
  digitalWrite(INB, HIGH);
  delay(ltdelay);
  stop();
}

void right() {
  digitalWrite(STBY, HIGH);
  analogWrite(ENA, 100);
  analogWrite(ENB, 100);
  digitalWrite(INA, HIGH);
  digitalWrite(INB, LOW);
  delay(rtdelay);
  stop();
}

void reverse() {
  digitalWrite(STBY, HIGH);
  analogWrite(ENA, 50);
  analogWrite(ENB, 50);
  digitalWrite(INA, LOW);
  digitalWrite(INB, LOW);
  delay(rdelay);
  stop();
}

// ---- Ultrasonic Sensor Functions ----

int detect_forward() {
  servo.write(90);
  delay(300);  // Reduced delay for faster response
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  return pulseIn(echo, HIGH) / 58;
}

int detect_right() {
  servo.write(0);
  delay(300);
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  int dist = pulseIn(echo, HIGH) / 58;
  servo.write(90);
  return dist;
}

int detect_left() {
  servo.write(180);
  delay(300);
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  int dist = pulseIn(echo, HIGH) / 58;
  servo.write(90);
  return dist;
}

// ---- Setup & Main Loop ----

void setup() {
  servo.attach(SRV);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);
  stop();
}

void loop() {
  if (line_state == 1) {
    // --- Line Following Mode ---
    int sensorValues[NUM_SENSORS];
    sensorValues[0] = analogRead(LEFT_SENSOR_PIN);
    sensorValues[1] = analogRead(CENTER_SENSOR_PIN);
    sensorValues[2] = analogRead(RIGHT_SENSOR_PIN);

    int position = calculateLinePosition(sensorValues);
    int error = position - 1000;  // Center is 1000
    int pidValue = calculatePID(error);
    
    leftSpeed = BASE_SPEED + pidValue;
    rightSpeed = BASE_SPEED - pidValue;
    
    leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);
    
    setMotorSpeed(leftSpeed, rightSpeed);
    delay(50);
  }
  else {
    // --- Obstacle Avoidance Mode (with line recovery attempts) ---
    bool lineFound = false;
    
    // Try to re-find the line (5 attempts)
    for (int attempt = 0; attempt < 5; attempt++) {
      forward();
      delay(200);  // Move forward a bit
      if (checkForLine()) {
        line_state = 1;  // Switch back to line-following
        lineFound = true;
        break;
      }
    }

    // If line not found, do obstacle avoidance
    if (!lineFound) {
      fd = detect_forward();
      if (fd < stop_distance) {
        stop();
        delay(500);
        rd = detect_right();
        ld = detect_left();
        
        // If both sides blocked, reverse
        if (rd < stop_distance && ld < stop_distance) {
          reverse();
        }
        // Otherwise, turn toward clearer path
        else if (rd > ld) {
          right();
        }
        else {
          left();
        }
      }
      else {
        forward();
        delay(200);  // Move forward longer
      }
    }
  }
}