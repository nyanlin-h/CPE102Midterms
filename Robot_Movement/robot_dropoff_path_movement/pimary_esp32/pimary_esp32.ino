#include <Arduino.h>
#include <math.h>

// --- Pin Definitions ---
const int LEFT_MOTOR_PWM = 25, LEFT_MOTOR_DIR = 26;
const int RIGHT_MOTOR_PWM = 27, RIGHT_MOTOR_DIR = 14;

// TCS3200 Color Sensor Pins , check and adjust the nums
const int S2 = 18;
const int S3 = 19;
const int OUT_PIN = 21;

// --- Pose and Target Variables ---
volatile float robotX = 0.0, robotY = 0.0, robotHeading = 0.0;
volatile float targetX = 0.0, targetY = 0.0;
volatile bool hasGoal = false;
const float DISTANCE_THRESHOLD = 5.0; // 5 cm threshold

String lastDetectedColor = "UNKNOWN";

void setMotors(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);
  digitalWrite(LEFT_MOTOR_DIR, leftSpeed >= 0 ? HIGH : LOW);
  analogWrite(LEFT_MOTOR_PWM, abs(leftSpeed));
  digitalWrite(RIGHT_MOTOR_DIR, rightSpeed >= 0 ? HIGH : LOW);
  analogWrite(RIGHT_MOTOR_PWM, abs(rightSpeed));
}


String readTCS3200Color() {
  // Red
  digitalWrite(S2, LOW); digitalWrite(S3, LOW);
  int redPW = pulseIn(OUT_PIN, LOW, 20000);

  // Blue
  digitalWrite(S2, LOW); digitalWrite(S3, HIGH);
  int bluePW = pulseIn(OUT_PIN, LOW, 20000);

  // Green
  digitalWrite(S2, HIGH); digitalWrite(S3, HIGH);
  int greenPW = pulseIn(OUT_PIN, LOW, 20000);

  // If no object is close enough to trigger pulse
  if (redPW == 0 || bluePW == 0 || greenPW == 0) return "UNKNOWN";

  // Simple calibration thresholds (Adjust based on your sensor readings)
  if (redPW < bluePW && redPW < greenPW && redPW < 100) return "Crimson";
  if (bluePW < redPW && bluePW < greenPW && bluePW < 100) return "Cyan";
  if (greenPW < redPW && greenPW < bluePW && greenPW < 100) return "Lime_Green";
  if (redPW < 120 && greenPW < 120 && bluePW > 150) return "Marigold";
  if (bluePW < 120 && greenPW < 120 && redPW > 150) return "Sky_Blue";
  if (redPW < 150 && bluePW < 150 && greenPW > 180) return "Violet";

  return "UNKNOWN";
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

  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT_PIN, INPUT);
}

void loop() {
  processSerialCommands();

  // 1. Read Color Sensor continuously
  String currentColor = readTCS3200Color();
  if (currentColor != "UNKNOWN" && currentColor != lastDetectedColor) {
    Serial.print("DETECTED_COLOR,");
    Serial.println(currentColor);
    lastDetectedColor = currentColor;
  }

  //  Drive to target position if active
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
    Serial.println("WAYPOINT_REACHED");
  }

  delay(20);
}