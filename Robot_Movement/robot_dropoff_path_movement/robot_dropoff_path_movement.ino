#include <Arduino.h>
#include <math.h>

// Current Pose from Python
volatile float robotX = 0.0, robotY = 0.0, robotHeading = 0.0;
// Dynamic Target assigned by master tracking code
volatile float targetX = 60.0, targetY = 50.0; 

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
    
    // Update current location from vision system
    if (line.startsWith("POS,")) {
      int c1 = line.indexOf(','), c2 = line.indexOf(',', c1 + 1), c3 = line.indexOf(',', c2 + 1);
      robotX = line.substring(c1 + 1, c2).toFloat();
      robotY = line.substring(c2 + 1, c3).toFloat();
      robotHeading = line.substring(c3 + 1).toFloat();
    } 
    // Update target waypoint assigned by Python
    else if (line.startsWith("SET_GOAL,")) {
      int c1 = line.indexOf(','), c2 = line.indexOf(',', c1 + 1);
      targetX = line.substring(c1 + 1, c2).toFloat();
      targetY = line.substring(c2 + 1).toFloat();
    }
  }
}

// Trigger function when onboard color sensor detects a stone
void onStoneGrabbed(String colorName) {
  // Transmit color detection string to Python: "DETECTED_COLOR,Marigold"
  Serial.print("DETECTED_COLOR,");
  Serial.println(colorName);
}

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_MOTOR_PWM, OUTPUT); pinMode(LEFT_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT); pinMode(RIGHT_MOTOR_DIR, OUTPUT);
}

void loop() {
  processSerialCommands();

  // Navigation proportional controller driving to active target (targetX, targetY)
  float deltaX = targetX - robotX;
  float deltaY = targetY - robotY;
  float distance = sqrt(deltaX * deltaX + deltaY * deltaY);

  if (distance > 5.0) { // Drive towards goal
    float desiredHeading = atan2(deltaY, deltaX);
    float headingError = desiredHeading - robotHeading;
    while (headingError > M_PI)  headingError -= 2 * M_PI;
    while (headingError < -M_PI) headingError += 2 * M_PI;

    int linear = constrain((int)(distance * 10.0), 60, 200);
    int angular = (int)(headingError * 40.0);

    setMotors(linear - angular, linear + angular);
  } else {
    setMotors(0, 0); // Goal reached
  }

  delay(20);
}