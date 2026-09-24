#include <Arduino.h>
#include <math.h>

// Live Odometry updated via Serial from Python
volatile float robotX = 0.0;
volatile float robotY = 0.0;
volatile float robotHeading = 0.0; // Radians

// Current Goal Target sent from Python
volatile float targetX = 0.0;
volatile float targetY = 0.0;
volatile bool hasGoal = false;

// Navigation Tuning
const float DISTANCE_THRESHOLD = 5.0; // 5 cm tolerance
const float K_LINEAR = 12.0;
const float K_ANGULAR = 45.0;
const int MAX_PWM = 255;
const int MIN_PWM = 60;

// Pins
const int LEFT_MOTOR_PWM = 25, LEFT_MOTOR_DIR = 26;
const int RIGHT_MOTOR_PWM = 27, RIGHT_MOTOR_DIR = 14;

void setMotors(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -MAX_PWM, MAX_PWM);
  rightSpeed = constrain(rightSpeed, -MAX_PWM, MAX_PWM);

  digitalWrite(LEFT_MOTOR_DIR, leftSpeed >= 0 ? HIGH : LOW);
  analogWrite(LEFT_MOTOR_PWM, abs(leftSpeed));

  digitalWrite(RIGHT_MOTOR_DIR, rightSpeed >= 0 ? HIGH : LOW);
  analogWrite(RIGHT_MOTOR_PWM, abs(rightSpeed));
}

void processSerialData() {
  while (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    // Parse: POS,x,y,heading
    if (input.startsWith("POS,")) {
      int c1 = input.indexOf(',');
      int c2 = input.indexOf(',', c1 + 1);
      int c3 = input.indexOf(',', c2 + 1);
      if (c1 != -1 && c2 != -1 && c3 != -1) {
        robotX = input.substring(c1 + 1, c2).toFloat();
        robotY = input.substring(c2 + 1, c3).toFloat();
        robotHeading = input.substring(c3 + 1).toFloat();
      }
    }
    // Parse: SET_GOAL,x,y
    else if (input.startsWith("SET_GOAL,")) {
      int c1 = input.indexOf(',');
      int c2 = input.indexOf(',', c1 + 1);
      if (c1 != -1 && c2 != -1) {
        targetX = input.substring(c1 + 1, c2).toFloat();
        targetY = input.substring(c2 + 1).toFloat();
        hasGoal = true;
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_MOTOR_PWM, OUTPUT); pinMode(LEFT_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT); pinMode(RIGHT_MOTOR_DIR, OUTPUT);
}

void loop() {
  processSerialData();

  if (!hasGoal) {
    setMotors(0, 0);
    delay(20);
    return;
  }

  float deltaX = targetX - robotX;
  float deltaY = targetY - robotY;
  float distance = sqrt(deltaX * deltaX + deltaY * deltaY);

  if (distance < DISTANCE_THRESHOLD) {
    setMotors(0, 0);
    Serial.println("ARRIVED");
    hasGoal = false;
    return;
  }

  float desiredHeading = atan2(deltaY, deltaX);
  float headingError = desiredHeading - robotHeading;

  while (headingError > M_PI)  headingError -= 2 * M_PI;
  while (headingError < -M_PI) headingError += 2 * M_PI;

  float linearSpeed = distance * K_LINEAR;
  float angularSpeed = headingError * K_ANGULAR;

  if (linearSpeed < MIN_PWM && distance > DISTANCE_THRESHOLD) {
    linearSpeed = MIN_PWM;
  }

  int leftOutput = (int)(linearSpeed - angularSpeed);
  int rightOutput = (int)(linearSpeed + angularSpeed);

  setMotors(leftOutput, rightOutput);
  delay(20);
}