#include <Arduino.h>
#include <math.h>
#include <ESP32Servo.h> // Library: ESP32Servo by Kevin Harrington

// --- Pin Assignments ---
// Drivetrain Motors
const int LEFT_MOTOR_PWM = 25, LEFT_MOTOR_DIR = 26;
const int RIGHT_MOTOR_PWM = 27, RIGHT_MOTOR_DIR = 14;

// 12V Intake Motor
const int INTAKE_MOTOR_PWM = 32;
const int INTAKE_MOTOR_DIR = 33;

// Drop Gate Servo
const int SERVO_PIN = 13;
Servo gateServo;

// Servo Angles for Dual-Stage Gate Sequence
const int GATE_CLOSED_ANGLE = 0;   
const int GATE_SMALL_ANGLE = 90;   
const int GATE_BIG_ANGLE = 180;    

// TCS3200 Color Sensor Pins
const int S2 = 18;
const int S3 = 19;
const int OUT_PIN = 21;

// --- Motion Control Variables ---
volatile float robotX = 0.0, robotY = 0.0, robotHeading = 0.0;
volatile float targetX = 0.0, targetY = 0.0;
volatile bool hasGoal = false;

// Dynamic Ramping for Heavy Robot on Slippery Surfaces
int currentLeftSpeed = 0, currentRightSpeed = 0;
const int MAX_PWM_STEP = 6;            // Limits acceleration per loop to prevent wheel spin
const float DISTANCE_THRESHOLD = 5.5;   // Increased stopping distance (5.5cm) to prevent overshooting

String lastDetectedColor = "UNKNOWN";

void setMotorsSmooth(int targetLeft, int targetRight) {
  targetLeft = constrain(targetLeft, -255, 255);
  targetRight = constrain(targetRight, -255, 255);

  // Ramp Left Motor Speed
  if (currentLeftSpeed < targetLeft) currentLeftSpeed = min(currentLeftSpeed + MAX_PWM_STEP, targetLeft);
  else if (currentLeftSpeed > targetLeft) currentLeftSpeed = max(currentLeftSpeed - MAX_PWM_STEP, targetLeft);

  // Ramp Right Motor Speed
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
  setIntakeMotor(0); // Pause intake during discharge
  delay(300);

  // Stage 1: Small Stone Discharge
  gateServo.write(GATE_SMALL_ANGLE);
  delay(800); 
  gateServo.write(GATE_CLOSED_ANGLE);
  delay(800); 

  // Stage 2: Large Stone Clearance Sequence
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
    else if (line == "START_INTAKE") {
      setIntakeMotor(220); // High PWM power for 7cm feeder channel
    }
    else if (line == "STOP_INTAKE") {
      setIntakeMotor(0);
    }
    else if (line == "RELEASE_STONE") {
      releaseStoneSequence();
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
}

void loop() {
  processSerialCommands();

  // Read TCS3200 sensor continuously
  String currentColor = readTCS3200Color();
  if (currentColor != "UNKNOWN" && currentColor != lastDetectedColor) {
    Serial.print("DETECTED_COLOR,");
    Serial.println(currentColor);
    lastDetectedColor = currentColor;
  }

  // Drive Control Logic
  if (!hasGoal) {
    setMotorsSmooth(0, 0);
    delay(20);
    return;
  }

  float deltaX = targetX - robotX;
  float deltaY = targetY - robotY;
  float distance = sqrt(deltaX * deltaX + deltaY * deltaY);

  if (distance > DISTANCE_THRESHOLD) {
    float desiredHeading = atan2(deltaY, deltaX);
    float headingError = desiredHeading - robotHeading;
    
    // Normalize heading error between -PI and +PI
    while (headingError > M_PI)  headingError -= 2 * M_PI;
    while (headingError < -M_PI) headingError += 2 * M_PI;

    // Smooth gain factors tuned for slippery floors & heavy weight
    int linear = constrain((int)(distance * 6.0), 35, 140);  // Low top-speed limit (140 PWM)
    int angular = (int)(headingError * 25.0);                 // Gentle steering gain

    setMotorsSmooth(linear - angular, linear + angular);
  } else {
    setMotorsSmooth(0, 0);
    hasGoal = false;
    Serial.println("WAYPOINT_REACHED");
  }

  delay(20);
}