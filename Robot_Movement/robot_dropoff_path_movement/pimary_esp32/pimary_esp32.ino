#include <Arduino.h>
#include <math.h>
#include <ESP32Servo.h>

// --- Pin Assignments ---
const int LEFT_MOTOR_PWM = 25, LEFT_MOTOR_DIR = 26;
const int RIGHT_MOTOR_PWM = 27, RIGHT_MOTOR_DIR = 14;

const int INTAKE_MOTOR_PWM = 32;
const int INTAKE_MOTOR_DIR = 33;

const int SERVO_PIN = 13;
Servo gateServo;

const int GATE_CLOSED_ANGLE = 0;   
const int GATE_SMALL_ANGLE = 90;   
const int GATE_BIG_ANGLE = 180;    

const int S2 = 18, S3 = 19, OUT_PIN = 21;

// --- Motion Control & Calibration Parameters ---
volatile float robotX = 0.0, robotY = 0.0, robotHeading = 0.0;
volatile float targetX = 0.0, targetY = 0.0;
volatile bool hasGoal = false;

int currentLeftSpeed = 0, currentRightSpeed = 0;
const int MAX_PWM_STEP = 6; 

// 5.0 cm stopping distance -> 5.0 * 3.74 = 18.7 pixels
const float DISTANCE_THRESHOLD = 18.7; 

// Telemetry Watchdog Timer
unsigned long lastTelemetryTime = 0;
const unsigned long TIMEOUT_MS = 1000; // Stop drive if camera link drops >1sec

String lastDetectedColor = "UNKNOWN";
String rxBuffer = "";

void setMotorsSmooth(int targetLeft, int targetRight) {
  targetLeft = constrain(targetLeft, -255, 255);
  targetRight = constrain(targetRight, -255, 255);

  if (currentLeftSpeed < targetLeft) currentLeftSpeed = min(currentLeftSpeed + MAX_PWM_STEP, targetLeft);
  else if (currentLeftSpeed > targetLeft) currentLeftSpeed = max(currentLeftSpeed - MAX_PWM_STEP, targetLeft);

  if (currentRightSpeed < targetRight) currentRightSpeed = min(currentRightSpeed + MAX_PWM_STEP, targetRight);
  else if (currentRightSpeed > targetRight) currentRightSpeed = max(currentRightSpeed - MAX_PWM_STEP, targetRight);

  digitalWrite(LEFT_MOTOR_DIR, currentLeftSpeed >= 0 ? HIGH : LOW);
  analogWrite(LEFT_MOTOR_PWM, abs(currentLeftSpeed));
  digitalWrite(RIGHT_MOTOR_DIR, currentRightSpeed >= 0 ? HIGH : LOW);
  analogWrite(RIGHT_MOTOR_PWM, abs(currentRightSpeed));
}

void setIntakeMotor(int speed) {
  speed = constrain(speed, -255, 255);
  digitalWrite(INTAKE_MOTOR_DIR, speed >= 0 ? HIGH : LOW);
  analogWrite(INTAKE_MOTOR_PWM, abs(speed));
}

void releaseStoneSequence() {
  setMotorsSmooth(0, 0);
  setIntakeMotor(0);
  delay(300);

  gateServo.write(GATE_SMALL_ANGLE);
  delay(800); 
  gateServo.write(GATE_CLOSED_ANGLE);
  delay(800); 

  if (lastDetectedColor != "UNKNOWN" && lastDetectedColor != "NONE") {
    gateServo.write(GATE_BIG_ANGLE);
    delay(800); 
    gateServo.write(GATE_SMALL_ANGLE);
    delay(800); 
    gateServo.write(GATE_CLOSED_ANGLE);
    delay(500); 
  }
  
  lastDetectedColor = "UNKNOWN";
}

String readTCS3200Color() {
  digitalWrite(S2, LOW); digitalWrite(S3, LOW);
  int redPW = pulseIn(OUT_PIN, LOW, 20000);

  digitalWrite(S2, LOW); digitalWrite(S3, HIGH);
  int bluePW = pulseIn(OUT_PIN, LOW, 20000);

  digitalWrite(S2, HIGH); digitalWrite(S3, HIGH);
  int greenPW = pulseIn(OUT_PIN, LOW, 20000);

  if (redPW == 0 || bluePW == 0 || greenPW == 0) return "UNKNOWN";

  if (redPW < bluePW && redPW < greenPW && redPW < 100) return "Crimson";
  if (bluePW < redPW && bluePW < greenPW && bluePW < 100) return "Cyan";
  if (greenPW < redPW && greenPW < bluePW && greenPW < 100) return "Lime_Green";
  if (redPW < 120 && greenPW < 120 && bluePW > 150) return "Marigold";
  if (bluePW < 120 && greenPW < 120 && redPW > 150) return "Sky_Blue";
  if (redPW < 150 && bluePW < 150 && greenPW > 180) return "Violet";

  return "UNKNOWN";
}

void parseCommand(String line) {
  line.trim();
  if (line.startsWith("POS,")) {
    int c1 = line.indexOf(','), c2 = line.indexOf(',', c1 + 1), c3 = line.indexOf(',', c2 + 1);
    robotX = line.substring(c1 + 1, c2).toFloat();
    robotY = line.substring(c2 + 1, c3).toFloat();
    robotHeading = line.substring(c3 + 1).toFloat();
    lastTelemetryTime = millis(); // Refresh watchdog timer
  } 
  else if (line.startsWith("SET_GOAL,")) {
    int c1 = line.indexOf(','), c2 = line.indexOf(',', c1 + 1);
    targetX = line.substring(c1 + 1, c2).toFloat();
    targetY = line.substring(c2 + 1).toFloat();
    hasGoal = true;
  }
  else if (line == "START_INTAKE") {
    setIntakeMotor(220);
  }
  else if (line == "STOP_INTAKE") {
    setIntakeMotor(0);
  }
  else if (line == "RELEASE_STONE") {
    releaseStoneSequence();
  }
}

// Non-blocking serial accumulator
void processSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      parseCommand(rxBuffer);
      rxBuffer = "";
    } else {
      rxBuffer += c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  
  pinMode(LEFT_MOTOR_PWM, OUTPUT); pinMode(LEFT_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT); pinMode(RIGHT_MOTOR_DIR, OUTPUT);
  pinMode(INTAKE_MOTOR_PWM, OUTPUT); pinMode(INTAKE_MOTOR_DIR, OUTPUT);

  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT); pinMode(OUT_PIN, INPUT);

  gateServo.attach(SERVO_PIN);
  gateServo.write(GATE_CLOSED_ANGLE);
  lastTelemetryTime = millis();
}

void loop() {
  processSerialCommands();

  // Watchdog Safety Check: Stop motors if tracking link stalls
  if (hasGoal && (millis() - lastTelemetryTime > TIMEOUT_MS)) {
    setMotorsSmooth(0, 0);
    hasGoal = false;
    Serial.println("NAV_TIMEOUT_SAFETY_STOP");
  }

  String currentColor = readTCS3200Color();
  if (currentColor != "UNKNOWN" && currentColor != lastDetectedColor) {
    Serial.print("DETECTED_COLOR,");
    Serial.println(currentColor);
    lastDetectedColor = currentColor;
  }

  if (!hasGoal) {
    setMotorsSmooth(0, 0);
    delay(20);
    return;
  }

  float deltaX = targetX - robotX;
  float deltaY = targetY - robotY;
  float distance = sqrt(deltaX * deltaX + deltaY * deltaY);

  if (distance > DISTANCE_THRESHOLD) {
    // Invert deltaY to convert OpenCV downward Y into standard Cartesian orientation
    float desiredHeading = atan2(-deltaY, deltaX);
    
    // Shortest-path angle wrapping using atan2(sin(e), cos(e))
    float headingError = desiredHeading - robotHeading;
    headingError = atan2(sin(headingError), cos(headingError));

    int linear = constrain((int)(distance * 1.8), 35, 140);  
    int angular = (int)(headingError * 25.0);                 

    setMotorsSmooth(linear - angular, linear + angular);
  } else {
    setMotorsSmooth(0, 0);
    hasGoal = false;
    Serial.println("WAYPOINT_REACHED");
  }

  delay(20);
}