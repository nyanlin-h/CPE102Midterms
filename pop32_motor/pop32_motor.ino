#include <pop32.h> 

const int INTAKE_MOTOR_PORT = 1;
const int RELEASE_SERVO_PORT = 1;

// --- Servo Angles ---
const int GATE_CLOSED_ANGLE = 0;   // Home position & small stone drop exit
const int GATE_SMALL_ANGLE = 90;   // Small stone entry port & big stone drop exit
const int GATE_BIG_ANGLE = 180;    // Big stone entry port

// Tracks whether a stone is actively held inside the dropper based on TCS3200 reads
String currentDropperColor = "UNKNOWN";

void setup() {
  Serial.begin(115200);
  servo(RELEASE_SERVO_PORT, GATE_CLOSED_ANGLE);
  motor(INTAKE_MOTOR_PORT, 0);
}

void releaseStone() {
  motor(INTAKE_MOTOR_PORT, 0); // Ensure intake is stopped during release cycle


  servo(RELEASE_SERVO_PORT, GATE_SMALL_ANGLE);
  delay(800); 

  // 2. Return to 0°: Completes small stone discharge
  servo(RELEASE_SERVO_PORT, GATE_CLOSED_ANGLE);
  delay(800); 


  bool stoneStillInDropper = (currentDropperColor != "UNKNOWN" && currentDropperColor != "NONE");

  if (stoneStillInDropper) {
    Serial.println("[POP32]: Small slot release incomplete. Opening to 180° for big stone...");

    // 3. Rotate to 180°: Opens full slot for big stones
    servo(RELEASE_SERVO_PORT, GATE_BIG_ANGLE);
    delay(800); 

    // 4. Move to 90°: Releases the big stone
    servo(RELEASE_SERVO_PORT, GATE_SMALL_ANGLE);
    delay(800); 

    // 5. Return to 0°: Resets to home position
    servo(RELEASE_SERVO_PORT, GATE_CLOSED_ANGLE);
    delay(500); 
  }

  // Clear local color state after release attempt
  currentDropperColor = "UNKNOWN";
}

void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "START_INTAKE") {
      motor(INTAKE_MOTOR_PORT, 95
      
      
      
      );
    } 
    else if (cmd == "STOP_INTAKE") {
      motor(INTAKE_MOTOR_PORT, 0);
    } 
    else if (cmd.startsWith("COLOR,")) {
      // Receives current color status forwarded from Python/ESP32
      currentDropperColor = cmd.substring(6);
      Serial.print("[POP32 STATE]: Dropper status set to ");
      Serial.println(currentDropperColor);
    }
    else if (cmd == "RELEASE_STONE") {
      releaseStone();
    }
  }
  delay(20);
}