#include <Arduino.h>
#include <math.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_eap_client.h"
#include "InEngMotor.h"

// --- Servo Configuration ---
#define SERVO_PIN 33

// --- Intake Roller Motor Pins ---
#define MR_IN1 21
#define MR_IN2 22
#define ENA 23
#define INTAKE_PWM_CH 4
#define INTAKE_FREQ 5000
#define INTAKE_RES 8

// --- Wi-Fi Enterprise Setup ---
const char* WIFI_SSID = "KMUTT-Secure";
#define EAP_IDENTITY "69070503403"
#define EAP_PASSWORD "Kmutt05p@ssword"

const unsigned int UDP_PORT = 8888;
WiFiUDP udp;
IPAddress remoteIP;
unsigned int remotePort;
bool hasRemoteHost = false;

// --- TCS3200 Color Sensor Pins (Updated S1 to GPIO 13) ---
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

// --- Motor Calibration Scaling (105 Left / 255 Right ratio) ---
const float LEFT_MOTOR_SCALE  = 105.0f / 255.0f;
const float RIGHT_MOTOR_SCALE = 1.000f;          

volatile float robotX = 0.0, robotY = 0.0, robotHeading = 0.0;
volatile float targetX = 0.0, targetY = 0.0;
volatile bool hasGoal = false;

int currentLeftSpeed = 0, currentRightSpeed = 0;
const int MAX_PWM_STEP = 15;
const float DISTANCE_THRESHOLD = 18.7;

unsigned long lastTelemetryTime = 0;
const unsigned long TIMEOUT_MS = 3000;

String lastDetectedColor = "UNKNOWN";
int colorCheckCounter = 0;

// --- Non-Blocking Gate Release Sequence ---
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

  // Stiction threshold compensation to prevent motor stalling at low PWM
  if (compensatedLeft > 0 && compensatedLeft < 60) compensatedLeft = 60;
  if (compensatedLeft < 0 && compensatedLeft > -60) compensatedLeft = -60;
  if (compensatedRight > 0 && compensatedRight < 60) compensatedRight = 60;
  if (compensatedRight < 0 && compensatedRight > -60) compensatedRight = -60;

  if (compensatedLeft >= 0 && compensatedRight >= 0) {
    inengmotor.forward(compensatedLeft, compensatedRight);
  } else {
    inengmotor.drive(compensatedLeft, compensatedRight);
  }
}

void setIntakeMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {
    digitalWrite(MR_IN1, HIGH);
    digitalWrite(MR_IN2, LOW);
  } else {
    digitalWrite(MR_IN1, LOW);
    digitalWrite(MR_IN2, HIGH);
  }
  ledcWrite(ENA, abs(speed));
}

void startReleaseSequence() {
  if (releaseState == RELEASE_IDLE) {
    setMotorsSmooth(0, 0);
    setIntakeMotor(0);
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
        if (lastDetectedColor != "UNKNOWN" && lastDetectedColor != "NONE") {
          gateServo.write(GATE_BIG_ANGLE);
          releaseState = STAGE_4;
        } else {
          releaseState = STAGE_6;
        }
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

  if (redFrequency == 0 || blueFrequency == 0 || greenFrequency == 0) return "UNKNOWN";

  if (redFrequency < blueFrequency && redFrequency < greenFrequency && redFrequency < 120) return "Crimson";
  if (greenFrequency < redFrequency && blueFrequency < redFrequency && redFrequency > 100) return "Cyan";
  if (greenFrequency < redFrequency && greenFrequency < blueFrequency && greenFrequency < 120) return "Lime_Green";
  return "UNKNOWN";
}

void parseCommand(String line) {
  line.trim();
  if (line.startsWith("POS,")) {
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
  }
  else if (line == "STOP_INTAKE") {
    setIntakeMotor(0);
  }
  else if (line == "RELEASE_STONE") {
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

  esp_eap_client_set_identity((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY));
  esp_eap_client_set_username((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY));
  esp_eap_client_set_password((uint8_t *)EAP_PASSWORD, strlen(EAP_PASSWORD));
  esp_eap_client_set_ttls_phase2_method(ESP_EAP_TTLS_PHASE2_PAP);
  esp_wifi_sta_enterprise_enable();
  
  WiFi.begin(WIFI_SSID);

  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    udp.begin(UDP_PORT);
  }
  
  pinMode(MR_IN1, OUTPUT); pinMode(MR_IN2, OUTPUT);
  ledcAttachChannel(ENA, INTAKE_FREQ, INTAKE_RES, INTAKE_PWM_CH);

  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  digitalWrite(S0, HIGH); digitalWrite(S1, LOW);

  gateServo.attach(SERVO_PIN);
  gateServo.write(GATE_CLOSED_ANGLE);
  lastTelemetryTime = millis();
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    processUDPCommands();
  }

  updateReleaseStateMachine();

  colorCheckCounter++;
  if (colorCheckCounter >= 20) {
    colorCheckCounter = 0;
    String currentColor = readTCS3200Color();
    if (currentColor != "UNKNOWN" && currentColor != lastDetectedColor) {
      sendUDP("DETECTED_COLOR," + currentColor);
      lastDetectedColor = currentColor;
    }
  }

  if (hasGoal && (millis() - lastTelemetryTime > TIMEOUT_MS)) {
    setMotorsSmooth(0, 0);
    hasGoal = false;
    sendUDP("NAV_TIMEOUT_SAFETY_STOP");
  }

  if (!hasGoal || releaseState != RELEASE_IDLE) {
    setMotorsSmooth(0, 0);
    delay(20);
    return;
  }

  float deltaX = targetX - robotX;
  float deltaY = -(targetY - robotY);
  float distance = sqrt(deltaX * deltaX + deltaY * deltaY);

  if (distance > DISTANCE_THRESHOLD) {
    float desiredHeading = atan2(deltaY, deltaX);
    float headingError = desiredHeading - robotHeading;
    headingError = atan2(sin(headingError), cos(headingError));

    int linear = constrain((int)(distance * 2.5), 120, 255);  
    int angular = (int)(headingError * 50.0);                 

    setMotorsSmooth(linear - angular, linear + angular);
  } else {
    setMotorsSmooth(0, 0);
    hasGoal = false;
    sendUDP("WAYPOINT_REACHED");
  }

  delay(20);
}