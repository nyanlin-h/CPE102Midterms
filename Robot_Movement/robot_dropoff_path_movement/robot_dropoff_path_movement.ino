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

// get coords dromthe other code
const int TOTAL_WAYPOINTS = 7;
Waypoint path[TOTAL_WAYPOINTS] = {
  { 30.0,  80.0}, // Green Anchor
  { 32.0,  25.0}, // Violet Anchor
  { 85.0,  15.0}, // Light Blue Anchor    
  {145.0,  20.0}, // Marigold Anchor
  {140.0,  78.0}, // Cyan Anchor
  { 80.0,  95.0}, // Crimson Anchor 
  { 10.0,  10.0}  // PICK-UP ZONE 
};

int currentTargetIndex = 0;
int currentCycleCount = 1; // Track how many round-trips the robot makes

// 2. FIXED: Re-added the missing loop completion variable
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

// replacement for analogwrite
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
  // Parse incoming tracking packets from serial stream here
  if (Serial.available() > 0) {
    String incomingLine = Serial.readStringUntil('\n');
    incomingLine.trim();
    
    if (incomingLine.startsWith("ROBOT,")) {
      incomingLine = incomingLine.substring(6); 
      int comma1 = incomingLine.indexOf(',');
      int comma2 = incomingLine.indexOf(',', comma1 + 1);
      
      if (comma1 != -1 && comma2 != -1) {
        robotX       = incomingLine.substring(0, comma1).toFloat();
        robotY       = incomingLine.substring(comma1 + 1, comma2).toFloat();
        robotHeading = incomingLine.substring(comma2 + 1).toFloat();
      }
    }
  }
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

  // 1. Fetch active coordinate target
  float targetX = path[currentTargetIndex].x;
  float targetY = path[currentTargetIndex].y;

  // 2. Compute error metrics
  float deltaX = targetX - robotX;
  float deltaY = targetY - robotY;
  float distanceToTarget = sqrt(deltaX * deltaX + deltaY * deltaY);

  // 3. Check if target is successfully reached
  if (distanceToTarget < DISTANCE_THRESHOLD) {
    
    // 3. FIXED: Handle the Pick-Up Zone specifically for multi-pass cycling
    if (currentTargetIndex == TOTAL_WAYPOINTS - 1) {
      setMotors(0, 0); // Stop wheels safely
      Serial.print("--- Arrived at Pick-Up Zone! Cycle "); 
      Serial.print(currentCycleCount); Serial.println(" Completed. ---");
      Serial.println("Waiting 5 seconds for loading...");
      
      delay(5000); // 5-second pause to let you drop off new materials
      
      currentCycleCount++;
      currentTargetIndex = 0; // RESET target index to point back to the Green Anchor
      Serial.println(">>> Moving out for next loop sequence! <<<");
    } 
    else {
      Serial.print("Reached Anchor: "); Serial.println(currentTargetIndex);
      currentTargetIndex++;
    }
    return; // Fast forward to the next loop iteration
  }

  // 4. Compute target heading error
  float desiredHeading = atan2(deltaY, deltaX);
  float headingError = desiredHeading - robotHeading;

  // Normalize angle error bounds to (-PI to PI)
  while (headingError > M_PI)  headingError -= 2 * M_PI;
  while (headingError < -M_PI) headingError += 2 * M_PI;

  // 5. Calculate Output Commands (Proportional Regulation)
  float linearSpeed = distanceToTarget * K_LINEAR;
  float angularSpeed = headingError * K_ANGULAR;

  // Prevent stalling by applying minimal operational base power
  if (linearSpeed < MIN_PWM && distanceToTarget > DISTANCE_THRESHOLD) {
    linearSpeed = MIN_PWM;
  }

  // 6. Differential Mixing to Drive Units
  int leftMotorOutput = (int)(linearSpeed - angularSpeed);
  int rightMotorOutput = (int)(linearSpeed + angularSpeed);

  setMotors(leftMotorOutput, rightMotorOutput);

  // Debug monitoring print
  delay(20); 
}
