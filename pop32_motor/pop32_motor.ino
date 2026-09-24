#include <pop32.h> 

// Hardware Pins & Setup
const int INTAKE_MOTOR_PORT = 1; // Motor 1 port
const int RELEASE_SERVO_PORT = 1; // Servo 1 for release gate at chute exit
const int COLOR_SENSOR_PIN = A0;  // Analog/I2C Color Sensor port near exit

// Gate Positions
const int GATE_CLOSED_ANGLE = 0;
const int GATE_OPEN_ANGLE = 90;

void setup() {
  Serial.begin(115200); // Communicate with ESP32 or Python
  
  servo(RELEASE_SERVO_PORT, GATE_CLOSED_ANGLE); // Keep gate closed initially
  motor(INTAKE_MOTOR_PORT, 0); // Intake off
}

// Function to classify analog/I2C sensor readings into color tags
String readChuteColorSensor() {
  int sensorVal = analogRead(COLOR_SENSOR_PIN); // Replace with the colour sensor pins later
  
  // Calibrate using  colour_value_getter parameters)
  if (sensorVal > 100 && sensorVal < 300) return "Crimson";
  if (sensorVal >= 300 && sensorVal < 500) return "Cyan";
  if (sensorVal >= 500 && sensorVal < 700) return "Violet";
  if (sensorVal >= 700 && sensorVal < 900) return "Lime_Green";
  if (sensorVal >= 900 && sensorVal < 1100) return "Marigold";
  if (sensorVal >= 1100) return "Sky_Blue";
  
  return "UNKNOWN";
}

void startIntake() {
  motor(INTAKE_MOTOR_PORT, 80); // Run rubber band intake forward at 80% power
}

void stopIntake() {
  motor(INTAKE_MOTOR_PORT, 0);
}

void releaseStone() {
  servo(RELEASE_SERVO_PORT, GATE_OPEN_ANGLE); // Open gate at chute exit
  delay(800);                                  // Allow stone to slide out
  servo(RELEASE_SERVO_PORT, GATE_CLOSED_ANGLE); // Close gate
}

void loop() {
  // Check for incoming commands from ESP32 or Python
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "START_INTAKE") {
      startIntake();
    } 
    else if (cmd == "RELEASE_STONE") {
      releaseStone();
    }
  }

  // Continuously scan for stone arriving at the exit chute
  String detectedColor = readChuteColorSensor();
  if (detectedColor != "UNKNOWN") {
    stopIntake(); // Stop pulling in stones while one is ready at exit
    
    // Send event string up to Master logic: "DETECTED_COLOR,Cyan"
    Serial.print("DETECTED_COLOR,");
    Serial.println(detectedColor);
    
    delay(500); // Debounce
  }
}