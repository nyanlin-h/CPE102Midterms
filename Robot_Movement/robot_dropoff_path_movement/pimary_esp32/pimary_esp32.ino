#include <Arduino.h>
#include <math.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_eap_client.h"
#include "InEngMotor.h" //[cite: 13]

// --- Global Hardware Driver Instantiation ---
InEngMotor inengmotor; // Fixed: Global motor instance[cite: 13]

// --- Servo Configuration ---
#define SERVO_PIN 33 //[cite: 13]

// --- Intake Roller Motor Pins ---
#define MR_IN1 21 //[cite: 13]
#define MR_IN2 22 //[cite: 13]
#define ENA 23 //[cite: 13]
#define INTAKE_PWM_CH 4 //[cite: 13]
#define INTAKE_FREQ 5000 //[cite: 13]
#define INTAKE_RES 8 //[cite: 13]

// --- Wi-Fi Enterprise Setup ---
const char* WIFI_SSID = "KMUTT-Secure"; //[cite: 13]
#define EAP_IDENTITY "69070503403" //[cite: 13]
#define EAP_PASSWORD "Kmutt05p@ssword" //[cite: 13]

const unsigned int UDP_PORT = 8888; //[cite: 13]
WiFiUDP udp; //[cite: 13]
IPAddress remoteIP; //[cite: 13]
unsigned int remotePort; //[cite: 13]
bool hasRemoteHost = false; //[cite: 13]

// --- TCS3200 Color Sensor Pins ---
#define S0 14 //[cite: 13]
#define S1 12 //
#define S2 4 //[cite: 13]
#define S3 5 //[cite: 13]
#define sensorOut 35 //[cite: 13]

int redFrequency = 0; //[cite: 13]
int greenFrequency = 0; //[cite: 13]
int blueFrequency = 0; //[cite: 13]

Servo gateServo; //[cite: 13]
const int GATE_CLOSED_ANGLE = 0;   //[cite: 13]
const int GATE_SMALL_ANGLE  = 90;  //[cite: 13]
const int GATE_BIG_ANGLE    = 180; //[cite: 13]

// --- Motor Calibration Scaling ---
const float LEFT_MOTOR_SCALE  = 0.353f; // 90 / 255
const float RIGHT_MOTOR_SCALE = 1.000f; // 255 / 255

volatile float robotX = 0.0, robotY = 0.0, robotHeading = 0.0; //[cite: 13]
volatile float targetX = 0.0, targetY = 0.0; //[cite: 13]
volatile bool hasGoal = false; //[cite: 13]

int currentLeftSpeed = 0, currentRightSpeed = 0; //[cite: 13]
const int MAX_PWM_STEP = 15; //[cite: 13]
const float DISTANCE_THRESHOLD = 18.7; //[cite: 13]

unsigned long lastTelemetryTime = 0; //[cite: 13]
const unsigned long TIMEOUT_MS = 3000; //

String lastDetectedColor = "UNKNOWN"; //[cite: 13]
int colorCheckCounter = 0; //[cite: 13]

// --- Non-Blocking Servo Release Sequence State Machine ---
enum ReleaseState { RELEASE_IDLE, STAGE_1, STAGE_2, STAGE_3, STAGE_4, STAGE_5, STAGE_6 };
ReleaseState releaseState = RELEASE_IDLE;
unsigned long releaseTimer = 0;

void sendUDP(String message) {
  if (hasRemoteHost) {
    udp.beginPacket(remoteIP, remotePort);
    udp.print(message + "\n");
    udp.endPacket();
  }
} //[cite: 13]

void setMotorsSmooth(int targetLeft, int targetRight) {
  targetLeft = constrain(targetLeft, -255, 255); //[cite: 13]
  targetRight = constrain(targetRight, -255, 255); //[cite: 13]

  if (currentLeftSpeed < targetLeft) currentLeftSpeed = min(currentLeftSpeed + MAX_PWM_STEP, targetLeft); //[cite: 13]
  else if (currentLeftSpeed > targetLeft) currentLeftSpeed = max(currentLeftSpeed - MAX_PWM_STEP, targetLeft); //[cite: 13]

  if (currentRightSpeed < targetRight) currentRightSpeed = min(currentRightSpeed + MAX_PWM_STEP, targetRight); //[cite: 13]
  else if (currentRightSpeed > targetRight) currentRightSpeed = max(currentRightSpeed - MAX_PWM_STEP, targetRight); //[cite: 13]

  int compensatedLeft  = (int)(currentLeftSpeed * LEFT_MOTOR_SCALE); //
  int compensatedRight = (int)(currentRightSpeed * RIGHT_MOTOR_SCALE); //

  inengmotor.drive(compensatedLeft, compensatedRight); //
}

void setIntakeMotor(int speed) {
  speed = constrain(speed, -255, 255); //[cite: 13]
  if (speed >= 0) {
    digitalWrite(MR_IN1, HIGH); //[cite: 13]
    digitalWrite(MR_IN2, LOW); //[cite: 13]
  } else {
    digitalWrite(MR_IN1, LOW); //[cite: 13]
    digitalWrite(MR_IN2, HIGH); //[cite: 13]
  }
  ledcWrite(INTAKE_PWM_CH, abs(speed)); //[cite: 13]
}

void startReleaseSequence() {
  if (releaseState == RELEASE_IDLE) {
    setMotorsSmooth(0, 0); //[cite: 13]
    setIntakeMotor(0); //[cite: 13]
    releaseState = STAGE_1;
    releaseTimer = millis();
  }
}

void updateReleaseStateMachine() {
  if (releaseState == RELEASE_IDLE) return;

  unsigned long elapsed = millis() - releaseTimer;

  switch (releaseState) {
    case STAGE_1: // Stop motion delay
      if (elapsed >= 300) {
        gateServo.write(GATE_SMALL_ANGLE); //[cite: 13]
        releaseState = STAGE_2;
        releaseTimer = millis();
      }
      break;

    case STAGE_2: // Small opening pulse
      if (elapsed >= 800) {
        gateServo.write(GATE_CLOSED_ANGLE); //[cite: 13]
        releaseState = STAGE_3;
        releaseTimer = millis();
      }
      break;

    case STAGE_3: // Check color qualification
      if (elapsed >= 800) {
        if (lastDetectedColor != "UNKNOWN" && lastDetectedColor != "NONE") { //[cite: 13]
          gateServo.write(GATE_BIG_ANGLE); //[cite: 13]
          releaseState = STAGE_4;
        } else {
          releaseState = STAGE_6;
        }
        releaseTimer = millis();
      }
      break;

    case STAGE_4: // Full release pulse
      if (elapsed >= 800) {
        gateServo.write(GATE_SMALL_ANGLE); //[cite: 13]
        releaseState = STAGE_5;
        releaseTimer = millis();
      }
      break;

    case STAGE_5: // Gate reset
      if (elapsed >= 800) {
        gateServo.write(GATE_CLOSED_ANGLE); //[cite: 13]
        releaseState = STAGE_6;
        releaseTimer = millis();
      }
      break;

    case STAGE_6: // Completion telemetry broadcast
      if (elapsed >= 500) {
        lastDetectedColor = "UNKNOWN"; //[cite: 13]
        sendUDP("RELEASE_DONE"); //[cite: 13]
        releaseState = RELEASE_IDLE;
      }
      break;

    default:
      releaseState = RELEASE_IDLE;
      break;
  }
}

String readTCS3200Color() {
  digitalWrite(S2, LOW); digitalWrite(S3, LOW); //[cite: 13]
  // Fixed: Increased timeout to 30000us (30ms) for reliable low-light reading
  redFrequency = pulseIn(sensorOut, LOW, 30000);

  digitalWrite(S2, HIGH); digitalWrite(S3, HIGH); //[cite: 13]
  greenFrequency = pulseIn(sensorOut, LOW, 30000);

  digitalWrite(S2, LOW); digitalWrite(S3, HIGH); //[cite: 13]
  blueFrequency = pulseIn(sensorOut, LOW, 30000);

  if (redFrequency == 0 || blueFrequency == 0 || greenFrequency == 0) return "UNKNOWN"; //[cite: 13]

  if (redFrequency < blueFrequency && redFrequency < greenFrequency && redFrequency < 120) return "Crimson"; //[cite: 13]
  if (blueFrequency < redFrequency && blueFrequency < greenFrequency && blueFrequency < 120) return "Cyan"; //[cite: 13]
  if (greenFrequency < redFrequency && greenFrequency < blueFrequency && greenFrequency < 120) return "Lime_Green"; //[cite: 13]
  return "UNKNOWN"; //[cite: 13]
}

void parseCommand(String line) {
  line.trim(); //[cite: 13]
  if (line.startsWith("POS,")) { //[cite: 13]
    int c1 = line.indexOf(','), c2 = line.indexOf(',', c1 + 1), c3 = line.indexOf(',', c2 + 1); //[cite: 13]
    robotX = line.substring(c1 + 1, c2).toFloat(); //[cite: 13]
    robotY = line.substring(c2 + 1, c3).toFloat(); //[cite: 13]
    robotHeading = line.substring(c3 + 1).toFloat(); //[cite: 13]
    lastTelemetryTime = millis(); //[cite: 13]
  } 
  else if (line.startsWith("SET_GOAL,")) { //[cite: 13]
    int c1 = line.indexOf(','), c2 = line.indexOf(',', c1 + 1); //[cite: 13]
    targetX = line.substring(c1 + 1, c2).toFloat(); //[cite: 13]
    targetY = line.substring(c2 + 1).toFloat(); //[cite: 13]
    hasGoal = true; //[cite: 13]
    lastTelemetryTime = millis(); //[cite: 13]
  }
  else if (line == "START_INTAKE") { //[cite: 13]
    setIntakeMotor(220); //[cite: 13]
  }
  else if (line == "STOP_INTAKE") { //[cite: 13]
    setIntakeMotor(0); //[cite: 13]
  }
  else if (line == "RELEASE_STONE") { //[cite: 13]
    startReleaseSequence();
  }
}

void processUDPCommands() {
  int packetSize = udp.parsePacket(); //[cite: 13]
  if (packetSize) { //[cite: 13]
    remoteIP = udp.remoteIP(); //[cite: 13]
    remotePort = udp.remotePort(); //[cite: 13]
    hasRemoteHost = true; //[cite: 13]

    char buffer[255]; //[cite: 13]
    int len = udp.read(buffer, 255); //[cite: 13]
    if (len > 0) {
      buffer[len] = 0; //[cite: 13]
      parseCommand(String(buffer)); //[cite: 13]
    }
  }
}

void setup() {
  Serial.begin(115200); //[cite: 13]

  inengmotor.begin(); //[cite: 13]

  WiFi.disconnect(true); //[cite: 13]
  delay(100); //[cite: 13]
  WiFi.mode(WIFI_STA); //[cite: 13]

  esp_eap_client_set_identity((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY)); //[cite: 13]
  esp_eap_client_set_username((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY)); //[cite: 13]
  esp_eap_client_set_password((uint8_t *)EAP_PASSWORD, strlen(EAP_PASSWORD)); //[cite: 13]
  esp_eap_client_set_ttls_phase2_method(ESP_EAP_TTLS_PHASE2_PAP); //[cite: 13]
  esp_wifi_sta_enterprise_enable(); //[cite: 13]
  
  WiFi.begin(WIFI_SSID); //[cite: 13]

  unsigned long startAttemptTime = millis(); //[cite: 13]
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) { //[cite: 13]
    delay(500); //[cite: 13]
  }

  if (WiFi.status() == WL_CONNECTED) { //[cite: 13]
    udp.begin(UDP_PORT); //[cite: 13]
  }
  
  pinMode(MR_IN1, OUTPUT); pinMode(MR_IN2, OUTPUT); //[cite: 13]
  ledcAttachChannel(ENA, INTAKE_FREQ, INTAKE_RES, INTAKE_PWM_CH); //[cite: 13]

  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT); //[cite: 13]
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT); //[cite: 13]
  pinMode(sensorOut, INPUT); //[cite: 13]

  digitalWrite(S0, HIGH); digitalWrite(S1, LOW); //[cite: 13]

  gateServo.attach(SERVO_PIN); //[cite: 13]
  gateServo.write(GATE_CLOSED_ANGLE); //[cite: 13]
  lastTelemetryTime = millis(); //[cite: 13]
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) { //[cite: 13]
    processUDPCommands(); //[cite: 13]
  }

  updateReleaseStateMachine();

  colorCheckCounter++; //[cite: 13]
  if (colorCheckCounter >= 20) { //[cite: 13]
    colorCheckCounter = 0; //[cite: 13]
    String currentColor = readTCS3200Color(); //[cite: 13]
    if (currentColor != "UNKNOWN" && currentColor != lastDetectedColor) { //[cite: 13]
      sendUDP("DETECTED_COLOR," + currentColor); //[cite: 13]
      lastDetectedColor = currentColor; //[cite: 13]
    }
  }

  if (hasGoal && (millis() - lastTelemetryTime > TIMEOUT_MS)) { //[cite: 13]
    setMotorsSmooth(0, 0); //[cite: 13]
    hasGoal = false; //[cite: 13]
    sendUDP("NAV_TIMEOUT_SAFETY_STOP"); //[cite: 13]
  }

  if (!hasGoal || releaseState != RELEASE_IDLE) {
    setMotorsSmooth(0, 0); //[cite: 13]
    delay(20); //[cite: 13]
    return; //[cite: 13]
  }

  // --- Steering Math Inversion ---
  float deltaX = targetX - robotX; //[cite: 13]
  float deltaY = -(targetY - robotY); // Fixed: Inverted Y-axis to match Cartesian plane
  float distance = sqrt(deltaX * deltaX + deltaY * deltaY); //[cite: 13]

  if (distance > DISTANCE_THRESHOLD) { //[cite: 13]
    float desiredHeading = atan2(deltaY, deltaX); //[cite: 13]
    float headingError = desiredHeading - robotHeading; //[cite: 13]
    headingError = atan2(sin(headingError), cos(headingError)); //[cite: 13]

    int linear = constrain((int)(distance * 2.5), 120, 255);  
    int angular = (int)(headingError * 50.0);                 

    setMotorsSmooth(linear - angular, linear + angular); //[cite: 13]
  } else {
    setMotorsSmooth(0, 0); //[cite: 13]
    hasGoal = false; //[cite: 13]
    sendUDP("WAYPOINT_REACHED"); //[cite: 13]
  }

  delay(20); //[cite: 13]
}