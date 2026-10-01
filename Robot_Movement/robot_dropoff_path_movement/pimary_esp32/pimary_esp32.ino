#include <Arduino.h>
#include <math.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// --- Servo Configuration ---
#define SERVO_PIN 33       // Set GPIO 33 as Servo Pin
#define PWM_FREQ 50        // 50 Hz Frequency
#define PWM_RESOLUTION 16  // 16-bit resolution

// --- Intake Roller Motor Pins ---
#define MR_IN1 21  // Intake motor IN1
#define MR_IN2 22  // Intake motor IN2
#define ENA 23     // Intake motor speed pin (PWM)


// --- Wi-Fi Settings ---
const char* WIFI_SSID = "KMUTT_SECURE";     // Change to your Wi-Fi Network Name
const char* WIFI_PASS = "Kmutt05p@ssword"; // Change to your Wi-Fi Password
const unsigned int UDP_PORT = 8888;

WiFiUDP udp;
IPAddress remoteIP;
unsigned int remotePort;
bool hasRemoteHost = false;

// --- Pin Assignments ---
const int LEFT_MOTOR_PWM = 26, LEFT_MOTOR_DIR = 26;
const int RIGHT_MOTOR_PWM = 27, RIGHT_MOTOR_DIR = 14;

const int INTAKE_MOTOR_PWM = 32;
const int INTAKE_MOTOR_DIR = 33;

 //correct
Servo gateServo;

const int GATE_CLOSED_ANGLE = 0;   
const int GATE_SMALL_ANGLE = 90;   
const int GATE_BIG_ANGLE = 180;    

const int S2 = 18, S3 = 19, OUT_PIN = 21;

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

  // Transmit RELEASE_DONE signal back over UDP
  sendUDP("RELEASE_DONE");
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

  // Connect Wi-Fi
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());

  udp.begin(UDP_PORT);
  
  pinMode(LEFT_MOTOR_PWM, OUTPUT); pinMode(LEFT_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT); pinMode(RIGHT_MOTOR_DIR, OUTPUT);
  pinMode(INTAKE_MOTOR_PWM, OUTPUT); pinMode(INTAKE_MOTOR_DIR, OUTPUT);

  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT); pinMode(OUT_PIN, INPUT);

  gateServo.attach(SERVO_PIN);
  gateServo.write(GATE_CLOSED_ANGLE);
  lastTelemetryTime = millis();
}

void loop() {
  processUDPCommands();

  // Watchdog Safety Check
  if (hasGoal && (millis() - lastTelemetryTime > TIMEOUT_MS)) {
    setMotorsSmooth(0, 0);
    hasGoal = false;
    sendUDP("NAV_TIMEOUT_SAFETY_STOP");
  }

  // Color check throttling
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