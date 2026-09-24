#include <pop32.h> 

const int INTAKE_MOTOR_PORT = 1;
const int RELEASE_SERVO_PORT = 1;
const int COLOR_SENSOR_PIN = A0; 

const int GATE_CLOSED_ANGLE = 0;
const int GATE_OPEN_ANGLE = 90;

String lastReportedColor = "UNKNOWN";

void setup() {
  Serial.begin(115200);
  servo(RELEASE_SERVO_PORT, GATE_CLOSED_ANGLE);
  motor(INTAKE_MOTOR_PORT, 0);
}

String readChuteColorSensor() {
  int sensorVal = analogRead(COLOR_SENSOR_PIN);
  
  if (sensorVal > 100 && sensorVal < 300) return "Crimson";
  if (sensorVal >= 300 && sensorVal < 500) return "Cyan";
  if (sensorVal >= 500 && sensorVal < 700) return "Violet";
  if (sensorVal >= 700 && sensorVal < 900) return "Lime_Green";
  if (sensorVal >= 900 && sensorVal < 1100) return "Marigold";
  if (sensorVal >= 1100) return "Sky_Blue";
  
  return "UNKNOWN";
}

void releaseStone() {
  servo(RELEASE_SERVO_PORT, GATE_OPEN_ANGLE);
  delay(800);
  servo(RELEASE_SERVO_PORT, GATE_CLOSED_ANGLE);
  lastReportedColor = "UNKNOWN"; // Reset state after releasing
}

void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "START_INTAKE") motor(INTAKE_MOTOR_PORT, 80);
    else if (cmd == "STOP_INTAKE") motor(INTAKE_MOTOR_PORT, 0);
    else if (cmd == "RELEASE_STONE") releaseStone();
  }

  String currentColor = readChuteColorSensor();
  
  // Only send serial payload ONCE when a new stone arrives
  if (currentColor != "UNKNOWN" && currentColor != lastReportedColor) {
    motor(INTAKE_MOTOR_PORT, 0); // Pause feeder while stone is ready
    Serial.print("DETECTED_COLOR,");
    Serial.println(currentColor);
    lastReportedColor = currentColor;
  }
  
  delay(50);
}