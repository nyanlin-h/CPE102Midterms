#include <Arduino.h>
#include <math.h>

volatile float robotX = 0.0, robotY = 0.0, robotHeading = 0.0;
volatile float targetX = 0.0, targetY = 0.0;
volatile bool hasGoal = false;

const float DISTANCE_THRESHOLD = 5.0; // 5 cm threshold
const int LEFT_MOTOR_PWM = 25, LEFT_MOTOR_DIR = 26;
const int RIGHT_MOTOR_PWM = 27, RIGHT_MOTOR_DIR = 14;

void setMotors(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  digitalWrite(LEFT_MOTOR_DIR, leftSpeed >= 0 ? HIGH : LOW);
  analogWrite(LEFT_MOTOR_PWM, abs(leftSpeed));
  digitalWrite(RIGHT_MOTOR_DIR, rightSpeed >= 0 ? HIGH : LOW);
  analogWrite(RIGHT_MOTOR_PWM, abs(rightSpeed));
}

void processSerialCommands() {
  while (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    
    if (line.startsWith("POS,")) {
      int c1 = line.indexOf(','), c2 = line.indexOf(',', c1 + 1), c3 = line.indexOf(',', c2 + 1);
      robotX = line.substring(c1 + 1, c2).toFloat();
      robotY = line.substring(c2 + 1, c3).toFloat();
      robotHeading = line.substring(c3 + 1).toFloat();
    } 
    else if (line.startsWith("SET_GOAL,")) {
      int c1 = line.indexOf(','), c2 = line.indexOf(',', c1 + 1);
      targetX = line.substring(c1 + 1, c2).toFloat();
      targetY = line.substring(c2 + 1).toFloat();
      hasGoal = true;
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_MOTOR_PWM, OUTPUT); pinMode(LEFT_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT); pinMode(RIGHT_MOTOR_DIR, OUTPUT);
}

void loop() {
  processSerialCommands();

  if (!hasGoal) {
    setMotors(0, 0);
    delay(20);
    return;
  }

  float deltaX = targetX - robotX;
  float deltaY = targetY - robotY;
  float distance = sqrt(deltaX * deltaX + deltaY * deltaY);

  if (distance > DISTANCE_THRESHOLD) {
    float desiredHeading = atan2(deltaY, deltaX);
    float headingError = desiredHeading - robotHeading;
    while (headingError > M_PI)  headingError -= 2 * M_PI;
    while (headingError < -M_PI) headingError += 2 * M_PI;

    int linear = constrain((int)(distance * 10.0), 60, 200);
    int angular = (int)(headingError * 40.0);

    setMotors(linear - angular, linear + angular);
  } else {
    setMotors(0, 0);
    hasGoal = false;
    Serial.println("WAYPOINT_REACHED"); // Notify Python that the target was reached
  }

  delay(20);
}