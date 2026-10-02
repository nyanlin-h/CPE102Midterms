#include <Arduino.h>
#include <math.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "InEngMotor.h"

// =========================================================================
// CONFIGURATION FLAGS
// Set to 'true' to trigger servo release IMMEDIATELY when color is sensed.
// Set to 'false' if Python controls the release via UDP "RELEASE_STONE".
// =========================================================================
const bool AUTO_RELEASE_ON_COLOR = true; 

// --- Servo Configuration ---
#define SERVO_PIN 18

// --- Intake Roller Motor Pins ---
#define MR_IN1 21
#define MR_IN2 22
#define ENA 23

// --- Wi-Fi Setup ---
const char* WIFI_SSID = "Tan";
const char* WIFI_PASS = "02052008";

const unsigned int UDP_PORT = 8888;
WiFiUDP udp;
IPAddress remoteIP;
unsigned int remotePort;
bool hasRemoteHost = false;
bool udpStarted = false;

// --- TCS3200 Color Sensor Pins ---
#define S0 14
#define S1 13
#define S2 4
#define S3 5
#define sensorOut 35

int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;

Servo gateServo;
const int GATE_CLOSED_ANGLE = 0;   
const int GATE_SMALL_ANGLE  = 90;  
const int GATE_BIG_ANGLE    = 180; 

// --- Motor Calibration Scaling (Original Values Restored) ---
const float LEFT_MOTOR_SCALE  = 105.0f / 255.0f;
const float RIGHT_MOTOR_SCALE = 1.000f;          

volatile float robotX = 0.0, robotY = 0.0, robotHeading = 0.0;
volatile float targetX = 0.0, targetY = 0.0;
volatile bool hasGoal = false;

int currentLeftSpeed = 0, currentRightSpeed = 0;

const int MAX_PWM_STEP = 60;          
const float DISTANCE_THRESHOLD = 55.0; 

// --- Speed Scaling (Original Values Restored) ---
const float COLLECTION_SPEED_SCALE = 0.2f; 
const float SLOW_DOWN_ZONE_PX = 120.0f;     

unsigned long intakeDriveStartTime = 0;
bool isIntakeDriving = false;
const unsigned long INTAKE_DRIVE_DURATION = 2000; 

unsigned long lastTelemetryTime = 0;
const unsigned long TIMEOUT_MS = 3000;

String lastDetectedColor = "UNKNOWN";
int colorCheckCounter = 0;

enum ReleaseState { RELEASE_IDLE, STAGE_1, STAGE_2, STAGE_3, STAGE_4, STAGE_5, STAGE_6 };
ReleaseState releaseState = RELEASE_IDLE;
unsigned long releaseTimer = 0;

void sendUDP(String message) {
  if (hasRemoteHost) {
    udp.beginPacket(remoteIP, remotePort);
    udp.print(message + "\n");
    udp.endPacket();
  }
}

void setMotorsSmooth(int targetLeft, int targetRight) {
  targetLeft = constrain(targetLeft, -255, 255);
  targetRight = constrain(targetRight, -255, 255);

  if (currentLeftSpeed < targetLeft) currentLeftSpeed = min(currentLeftSpeed + MAX_PWM_STEP, targetLeft);
  else if (currentLeftSpeed > targetLeft) currentLeftSpeed = max(currentLeftSpeed - MAX_PWM_STEP, targetLeft);

  if (currentRightSpeed < targetRight) currentRightSpeed = min(currentRightSpeed + MAX_PWM_STEP, targetRight);
  else if (currentRightSpeed > targetRight) currentRightSpeed = max(currentRightSpeed - MAX_PWM_STEP, targetRight);

  int compensatedLeft  = (int)(currentLeftSpeed * LEFT_MOTOR_SCALE);
  int compensatedRight = (int)(currentRightSpeed * RIGHT_MOTOR_SCALE);

  // Original motor deadzone bounds (60 PWM)
  if (compensatedLeft > 0 && compensatedLeft < 60) compensatedLeft = 60;
  if (compensatedLeft < 0 && compensatedLeft > -60) compensatedLeft = -60;
  if (compensatedRight > 0 && compensatedRight < 60) compensatedRight = 60;
  if (compensatedRight < 0 && compensatedRight > -60) compensatedRight = -60;

  if (targetLeft == 0 && targetRight == 0) {
    compensatedLeft = 0;
    compensatedRight = 0;
    currentLeftSpeed = 0;
    currentRightSpeed = 0;
  }

  if (compensatedLeft >= 0 && compensatedRight >= 0) {
    inengmotor.forward(compensatedLeft, compensatedRight);
  } else {
    inengmotor.drive(compensatedLeft, compensatedRight);
  }
}

void setIntakeMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(MR_IN1, HIGH);
    digitalWrite(MR_IN2, LOW);
  } else if (speed < 0) {
    digitalWrite(MR_IN1, LOW);
    digitalWrite(MR_IN2, HIGH);
  } else {
    digitalWrite(MR_IN1, LOW);
    digitalWrite(MR_IN2, LOW);
  }
  ledcWrite(ENA, abs(speed));
}

void startReleaseSequence() {
  if (releaseState == RELEASE_IDLE) {
    setMotorsSmooth(0, 0);
    setIntakeMotor(0);
    isIntakeDriving = false;
    releaseState = STAGE_1;
    releaseTimer = millis();
  }
}

void updateReleaseStateMachine() {
  if (releaseState == RELEASE_IDLE) return;

  unsigned long elapsed = millis() - releaseTimer;

  switch (releaseState) {
    case STAGE_1:
      if (elapsed >= 300) {
        gateServo.write(GATE_SMALL_ANGLE);
        releaseState = STAGE_2;
        releaseTimer = millis();
      }
      break;

    case STAGE_2:
      if (elapsed >= 800) {
        gateServo.write(GATE_CLOSED_ANGLE);
        releaseState = STAGE_3;
        releaseTimer = millis();
      }
      break;

    case STAGE_3:
      if (elapsed >= 800) {
        gateServo.write(GATE_BIG_ANGLE);
        releaseState = STAGE_4;
        releaseTimer = millis();
      }
      break;

    case STAGE_4:
      if (elapsed >= 800) {
        gateServo.write(GATE_SMALL_ANGLE);
        releaseState = STAGE_5;
        releaseTimer = millis();
      }
      break;

    case STAGE_5:
      if (elapsed >= 800) {
        gateServo.write(GATE_CLOSED_ANGLE);
        releaseState = STAGE_6;
        releaseTimer = millis();
      }
      break;

    case STAGE_6:
      if (elapsed >= 500) {
        lastDetectedColor = "UNKNOWN";
        sendUDP("RELEASE_DONE");
        releaseState = RELEASE_IDLE;
      }
      break;

    default:
      releaseState = RELEASE_IDLE;
      break;
  }
}

String readTCS3200Color() {
  digitalWrite(S2, LOW); digitalWrite(S3, LOW);
  redFrequency = pulseIn(sensorOut, LOW, 30000);

  digitalWrite(S2, HIGH); digitalWrite(S3, HIGH);
  greenFrequency = pulseIn(sensorOut, LOW, 30000);

  digitalWrite(S2, LOW); digitalWrite(S3, HIGH);
  blueFrequency = pulseIn(sensorOut, LOW, 30000);

  if (redFrequency == 0 || greenFrequency == 0 || blueFrequency == 0) return "UNKNOWN";

  if ((redFrequency >= 270 && redFrequency <= 300) &&
      (greenFrequency >= 650 && greenFrequency <= 705) &&
      (blueFrequency >= 555 && blueFrequency <= 610)) return "Crimson";

  if ((redFrequency >= 340 && redFrequency <= 380) &&
      (greenFrequency >= 350 && greenFrequency <= 385) &&
      (blueFrequency >= 460 && blueFrequency <= 495)) return "Lime_Green";

  if ((redFrequency >= 275 && redFrequency <= 315) &&
      (greenFrequency >= 550 && greenFrequency <= 595) &&
      (blueFrequency >= 595 && blueFrequency <= 635)) return "Marigold";

  if ((redFrequency >= 450 && redFrequency <= 495) &&
      (greenFrequency >= 700 && greenFrequency <= 750) &&
      (blueFrequency >= 585 && blueFrequency <= 625)) return "Violet";

  if ((redFrequency >= 560 && redFrequency <= 630) &&
      (greenFrequency >= 680 && greenFrequency <= 745) &&
      (blueFrequency >= 675 && blueFrequency <= 730)) return "Sky_Blue";

  if ((redFrequency >= 425 && redFrequency <= 480) &&
      (greenFrequency >= 440 && greenFrequency <= 485) &&
      (blueFrequency >= 420 && blueFrequency <= 465)) return "Cyan";

  return "UNKNOWN";
}

void parseCommand(String line) {
  line.trim();
  if (line == "PING") {
    lastTelemetryTime = millis();
    sendUDP("PONG");
  }
  else if (line.startsWith("POS,")) {
    int c1 = line.indexOf(','), c2 = line.indexOf(',', c1 + 1), c3 = line.indexOf(',', c2 + 1);
    robotX = line.substring(c1 + 1, c2).toFloat();
    robotY = line.substring(c2 + 1, c3).toFloat();
    robotHeading = line.substring(c3 + 1).toFloat();
    lastTelemetryTime = millis();
  } 
  else if (line.startsWith("SET_GOAL,")) {
    int c1 = line.indexOf(','), c2 = line.indexOf(',', c1 + 1);
    targetX = line.substring(c1 + 1, c2).toFloat();
    targetY = line.substring(c2 + 1).toFloat();
    hasGoal = true;
    lastTelemetryTime = millis();
  }
  else if (line == "START_INTAKE") {
    lastDetectedColor = "UNKNOWN";
    setIntakeMotor(220);
    
    if (!isIntakeDriving && (millis() - intakeDriveStartTime > 3000)) {
      isIntakeDriving = true;
      intakeDriveStartTime = millis();
    }
  }
  else if (line == "STOP_INTAKE") {
    setIntakeMotor(0);
    isIntakeDriving = false;
  }
  else if (line == "RELEASE_STONE" || line == "TEST_SERVO") {
    startReleaseSequence();
  }
}

void processUDPCommands() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    remoteIP = udp.remoteIP();
    remotePort = udp.remotePort();
    hasRemoteHost = true;

    char buffer[256];
    int len = udp.read(buffer, 255);
    if (len > 0) {
      buffer[len] = 0;
      parseCommand(String(buffer));
    }
  }
}

void setup() {
  Serial.begin(115200);

  inengmotor.begin();

  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_STA);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to Wi-Fi");

  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWi-Fi Connected!");
    Serial.print("ESP32 IP Address: ");
    Serial.println(WiFi.localIP());
    udp.begin(UDP_PORT);
    udpStarted = true;
  } else {
    Serial.println("\nWi-Fi Connection Timed Out");
  }
  
  pinMode(MR_IN1, OUTPUT); 
  pinMode(MR_IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  ledcAttach(ENA, 5000, 8);
  setIntakeMotor(0);

  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  digitalWrite(S0, HIGH); digitalWrite(S1, LOW);

  ESP32PWM::allocateTimer(0);
  gateServo.setPeriodHertz(50);
  gateServo.attach(SERVO_PIN, 500, 2400); 
  
  gateServo.write(0);
  delay(300);
  gateServo.write(90);
  delay(300);
  gateServo.write(0);

  lastTelemetryTime = millis();
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!udpStarted) {
      udp.begin(UDP_PORT);
      udpStarted = true;
    }
    processUDPCommands();
  } else {
    udpStarted = false;
  }

  updateReleaseStateMachine();

  colorCheckCounter++;
  if (colorCheckCounter >= 10) {
    colorCheckCounter = 0;
    String currentColor = readTCS3200Color();
    if (currentColor != "UNKNOWN" && currentColor != lastDetectedColor) {
      sendUDP("DETECTED_COLOR," + currentColor);
      lastDetectedColor = currentColor;

      if (AUTO_RELEASE_ON_COLOR) {
        startReleaseSequence();
      }
    }
  }

  if (hasGoal && (millis() - lastTelemetryTime > TIMEOUT_MS)) {
    setMotorsSmooth(0, 0);
    hasGoal = false;
    isIntakeDriving = false;
    sendUDP("NAV_TIMEOUT_SAFETY_STOP");
  }

  // Original Intake Driving speed calculation restored
  if (isIntakeDriving) {
    if (millis() - intakeDriveStartTime < INTAKE_DRIVE_DURATION) {
      int reducedSpeed = (int)(250 * COLLECTION_SPEED_SCALE); 
      setMotorsSmooth(reducedSpeed, reducedSpeed); 
    } else {
      isIntakeDriving = false; 
      setMotorsSmooth(0, 0);
    }
    delay(10);
    return;
  }

  if (!hasGoal || releaseState != RELEASE_IDLE) {
    setMotorsSmooth(0, 0);
    delay(10);
    return;
  }

  float deltaX = targetX - robotX;
  float deltaY = -(targetY - robotY);
  float distance = sqrt(deltaX * deltaX + deltaY * deltaY);

  if (distance > DISTANCE_THRESHOLD) {
    float desiredHeading = atan2(deltaY, deltaX);
    float headingError = desiredHeading - robotHeading;
    headingError = atan2(sin(headingError), cos(headingError));

    // Original speed control restored
    int linear = constrain((int)(distance * 2.5), 120, 255);  

    if (distance <= SLOW_DOWN_ZONE_PX) {
      linear = (int)(linear * COLLECTION_SPEED_SCALE);
    }

    int angular = (int)(headingError * 50.0);                 

    setMotorsSmooth(linear - angular, linear + angular);
  } else {
    setMotorsSmooth(0, 0);
    hasGoal = false;
    sendUDP("WAYPOINT_REACHED");
  }

  delay(10);
}