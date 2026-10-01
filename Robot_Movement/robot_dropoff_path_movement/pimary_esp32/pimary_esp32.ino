#include <Arduino.h>
#include <math.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_eap_client.h"

// --- Servo Configuration ---
#define SERVO_PIN 33       
#define PWM_FREQ 50        
#define PWM_RESOLUTION 16  

// --- Intake Roller Motor Pins ---
#define MR_IN1 21  
#define MR_IN2 22  
#define ENA 23     

// =========================================================================
// --- UNIVERSITY WI-FI ENTERPRISE PROFILES ---
// =========================================================================
const char* WIFI_SSID = "KMUTT-Secure";     
#define EAP_IDENTITY "69070503403"      
#define EAP_PASSWORD "Kmutt05p@ssword"      

const unsigned int UDP_PORT = 8888;
WiFiUDP udp;
IPAddress remoteIP;
unsigned int remotePort;
bool hasRemoteHost = false;

// ==========================================================
// --- DRIVE WHEEL PIN ASSIGNMENTS ---
// ==========================================================
const int LEFT_MOTOR_PWM = 12;  
const int LEFT_MOTOR_DIR = 13;  
const int RIGHT_MOTOR_PWM = 2;  
const int RIGHT_MOTOR_DIR = 5;  

// --- ESP32 Hardware PWM Channels ---
#define LEFT_PWM_CH   2
#define RIGHT_PWM_CH  3
#define INTAKE_PWM_CH 4
#define MOTOR_FREQ    5000
#define MOTOR_RES     8    

// ==========================================================
// --- TCS3200 COLOR SENSOR PINS ---
// ==========================================================
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
const int GATE_SMALL_ANGLE = 90;   
const int GATE_BIG_ANGLE = 180;    

// --- Motion Control & Parameters ---
volatile float robotX = 0.0, robotY = 0.0, robotHeading = 0.0;
volatile float targetX = 0.0, targetY = 0.0;
volatile bool hasGoal = false;

int currentLeftSpeed = 0, currentRightSpeed = 0;
const int MAX_PWM_STEP = 6; 
const float DISTANCE_THRESHOLD = 18.7; 

unsigned long lastTelemetryTime = 0;
const unsigned long TIMEOUT_MS = 1000; 

String lastDetectedColor = "UNKNOWN";
int colorCheckCounter = 0; 

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

  digitalWrite(LEFT_MOTOR_DIR, currentLeftSpeed >= 0 ? HIGH : LOW);
  digitalWrite(RIGHT_MOTOR_DIR, currentRightSpeed >= 0 ? HIGH : LOW);

  // FIXED: Core 3.x ledcWrite expects GPIO Pin Numbers instead of Channel IDs
  ledcWrite(LEFT_MOTOR_PWM, abs(currentLeftSpeed));
  ledcWrite(RIGHT_MOTOR_PWM, abs(currentRightSpeed));
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
  // FIXED: Core 3.x ledcWrite expects ENA GPIO Pin Number
  ledcWrite(ENA, abs(speed));
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
  sendUDP("RELEASE_DONE");
}

String readTCS3200Color() {
  digitalWrite(S2, LOW); digitalWrite(S3, LOW);
  redFrequency = pulseIn(sensorOut, LOW, 20000);

  digitalWrite(S2, HIGH); digitalWrite(S3, HIGH);
  greenFrequency = pulseIn(sensorOut, LOW, 20000);

  digitalWrite(S2, LOW); digitalWrite(S3, HIGH);
  blueFrequency = pulseIn(sensorOut, LOW, 20000);

  if (redFrequency == 0 || blueFrequency == 0 || greenFrequency == 0) return "UNKNOWN";

  if (redFrequency < blueFrequency && redFrequency < greenFrequency && redFrequency < 100) return "Crimson";
  if (blueFrequency < redFrequency && blueFrequency < greenFrequency && blueFrequency < 100) return "Cyan";
  if (greenFrequency < redFrequency && greenFrequency < blueFrequency && greenFrequency < 100) return "Lime_Green";
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

void processUDPCommands() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    remoteIP = udp.remoteIP();
    remotePort = udp.remotePort();
    hasRemoteHost = true;

    char buffer[255]; 
    int len = udp.read(buffer, 255);
    if (len > 0) {
      buffer[len] = 0;
      parseCommand(String(buffer));
    }
  }
}

void setup() {
  Serial.begin(115200);

  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_STA); 

  Serial.print("Connecting to University Enterprise Network: ");
  Serial.println(WIFI_SSID);

  esp_eap_client_set_identity((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY));
  esp_eap_client_set_username((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY));
  esp_eap_client_set_password((uint8_t *)EAP_PASSWORD, strlen(EAP_PASSWORD));
  
  esp_eap_client_set_ttls_phase2_method(ESP_EAP_TTLS_PHASE2_PAP); 
  esp_wifi_sta_enterprise_enable(); 
  
  WiFi.begin(WIFI_SSID); 

  unsigned long startAttemptTime = millis();
  
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 20000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected successfully via EAP Client!");
    Serial.print("ESP32 IP Address: ");
    Serial.println(WiFi.localIP());
    udp.begin(UDP_PORT);
  } else {
    Serial.println("\nWiFi Connection Timed Out! Operating in offline test mode...");
  }
  
  pinMode(LEFT_MOTOR_DIR, OUTPUT); 
  pinMode(RIGHT_MOTOR_DIR, OUTPUT);
  pinMode(MR_IN1, OUTPUT); 
  pinMode(MR_IN2, OUTPUT);

  ledcAttachChannel(LEFT_MOTOR_PWM, MOTOR_FREQ, MOTOR_RES, LEFT_PWM_CH);
  ledcAttachChannel(RIGHT_MOTOR_PWM, MOTOR_FREQ, MOTOR_RES, RIGHT_PWM_CH);
  ledcAttachChannel(ENA, MOTOR_FREQ, MOTOR_RES, INTAKE_PWM_CH);
  
  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  gateServo.attach(SERVO_PIN);
  gateServo.write(GATE_CLOSED_ANGLE);
  lastTelemetryTime = millis();
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    processUDPCommands();
  }

  if (hasGoal && (millis() - lastTelemetryTime > TIMEOUT_MS)) {
    setMotorsSmooth(0, 0);
    hasGoal = false;
    sendUDP("NAV_TIMEOUT_SAFETY_STOP");
  }

  colorCheckCounter++;
  if (colorCheckCounter >= 10) {
    colorCheckCounter = 0;
    String currentColor = readTCS3200Color();
    if (currentColor != "UNKNOWN" && currentColor != lastDetectedColor) {
      sendUDP("DETECTED_COLOR," + currentColor);
      lastDetectedColor = currentColor;
    }
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
    float desiredHeading = atan2(-deltaY, deltaX);
    float headingError = desiredHeading - robotHeading;
    headingError = atan2(sin(headingError), cos(headingError));

    int linear = constrain((int)(distance * 1.8), 35, 140);  
    int angular = (int)(headingError * 25.0);                 

    setMotorsSmooth(linear - angular, linear + angular);
  } else {
    setMotorsSmooth(0, 0);
    hasGoal = false;
    sendUDP("WAYPOINT_REACHED");
  }

  delay(20);
}