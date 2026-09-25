#include <pop32.h> 

const int INTAKE_MOTOR_PORT = 1;
const int RELEASE_SERVO_PORT = 1;

const int GATE_CLOSED_ANGLE = 0;
const int GATE_OPEN_ANGLE = 90;

void setup() {
  Serial.begin(115200);
  servo(RELEASE_SERVO_PORT, GATE_CLOSED_ANGLE);
  motor(INTAKE_MOTOR_PORT, 0);
}

void releaseStone() {
  motor(INTAKE_MOTOR_PORT, 0); // Stop intake during drop-off
  servo(RELEASE_SERVO_PORT, GATE_OPEN_ANGLE);
  delay(800);
  servo(RELEASE_SERVO_PORT, GATE_CLOSED_ANGLE);
}

void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "START_INTAKE") motor(INTAKE_MOTOR_PORT, 80);
    else if (cmd == "STOP_INTAKE") motor(INTAKE_MOTOR_PORT, 0);
    else if (cmd == "RELEASE_STONE") releaseStone();
  }
  delay(20);
}