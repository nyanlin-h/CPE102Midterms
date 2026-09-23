#include <Arduino.h>
#include <math.h>

// --- ROBOT POSITION (Global Odometry State) ---
// get values from robot_tracking code
volatile float robotX = 0.0; 
volatile float robotY = 0.0;
volatile float robotHeading = 0.0; // In radians 

// --- PREDETERMINED PATH ANCHORS ---
struct Waypoint {
  float x;
  float y;
};

const int TOTAL_WAYPOINTS = 6;
Waypoint path[TOTAL_WAYPOINTS] = {
  { 30.0,  80.0}, // Green Anchor
  { 32.0,  25.0}, // Violet Anchor
  { 85.0,  15.0}, // Light Blue Anchor    dont forget to fix coord
  {145.0,  20.0}, // Marigold Anchor
  {140.0,  78.0}, // Cyan Anchor
  { 80.0,  95.0}  // Crimson Anchor
};

int currentTargetIndex = 0;
bool pathCompleted = false;

// --- NAVIGATION TUNING PARAMETERS ---
const float DISTANCE_THRESHOLD = 5.0; // Distance tolerance to clear a point (~ 5 cm)
const float K_LINEAR = 12.0;          // Proportional speed multiplier
const float K_ANGULAR = 45.0;         // Proportional steering multiplier
const int MAX_PWM = 255;
const int MIN_PWM = 60;

// --- MOTOR CONTROLLER DRIVER PINS ---
const int LEFT_MOTOR_PWM = 25;
const int LEFT_MOTOR_DIR = 26;
const int RIGHT_MOTOR_PWM = 27;
const int RIGHT_MOTOR_DIR = 14;

//replacement for analogwrite
void ledcAnalogWrite(uint8_t pin, uint32_t value) {
  // fall back wrapper
  analogWrite(pin, value); 
}

void setMotors(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -MAX_PWM, MAX_PWM);
  rightSpeed = constrain(rightSpeed, -MAX_PWM, MAX_PWM);

  // Left Motor Direction Set
  digitalWrite(LEFT_MOTOR_DIR, leftSpeed >= 0 ? HIGH : LOW);
  ledcAnalogWrite(LEFT_MOTOR_PWM, abs(leftSpeed));

  // Right Motor Direction Set
  digitalWrite(RIGHT_MOTOR_DIR, rightSpeed >= 0 ? HIGH : LOW);
  ledcAnalogWrite(RIGHT_MOTOR_PWM, abs(rightSpeed));
}

// --- 2. SENSOR TRACKING PLACEHOLDER ---
void updateRobotOdometry() {
  // TODO: get values from code and update robotx and roboty
  
}

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR, OUTPUT);
  
  Serial.println("System Initialized. Starting Autonomous Path Navigation...");
}

void loop() {
  // Continuously refresh where the robot is located before making driving math decisions
  updateRobotOdometry();

  // 1. Check if complete
  if (pathCompleted) {
    setMotors(0, 0); // Stop completely
    Serial.println("Success: All drop-off areas cleared!");
    while(1) { delay(1000); } // Stop execution loop
  }

  // 2. Fetch active coordinate target
  float targetX = path[currentTargetIndex].x;
  float targetY = path[currentTargetIndex].y;

  // 3. Compute error metrics
  float deltaX = targetX - robotX;
  float deltaY = targetY - robotY;
  float distanceToTarget = sqrt(deltaX * deltaX + deltaY * deltaY);

  // 4. Check if target is successfully reached
  if (distanceToTarget < DISTANCE_THRESHOLD) {
    Serial.print("Reached Anchor: "); Serial.println(currentTargetIndex);
    currentTargetIndex++;
    
    if (currentTargetIndex >= TOTAL_WAYPOINTS) {
      pathCompleted = true;
    }
    return; // Fast forward to the next loop iteration
  }

  // 5. Compute target heading error
  float desiredHeading = atan2(deltaY, deltaX);
  float headingError = desiredHeading - robotHeading;

  // Normalize angle error bounds to (-PI to PI)
  while (headingError > M_PI)  headingError -= 2 * M_PI;
  while (headingError < -M_PI) headingError += 2 * M_PI;

  // 6. Calculate Output Commands (Proportional Regulation)
  float linearSpeed = distanceToTarget * K_LINEAR;
  float angularSpeed = headingError * K_ANGULAR;

  // Prevent stalling by applying minimal operational base power
  if (linearSpeed < MIN_PWM && distanceToTarget > DISTANCE_THRESHOLD) {
    linearSpeed = MIN_PWM;
  }

  // 7. Differential Mixing to Drive Units
  int leftMotorOutput = (int)(linearSpeed - angularSpeed);
  int rightMotorOutput = (int)(linearSpeed + angularSpeed);

  setMotors(leftMotorOutput, rightMotorOutput);

  // Debug monitoring print
  delay(20); 
}
