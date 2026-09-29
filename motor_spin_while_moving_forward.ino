// --- Motor A (Drive Motor) ---
const int DRIVE_IN1 = 16; // Connected to AI1
const int DRIVE_IN2 = 17; // Connected to AI2

// --- Motor B (Rubber Band Intake Roller Motor) ---
const int INTAKE_IN1 = 18; // Connected to BI1
const int INTAKE_IN2 = 19; // Connected to BI2

// --- Servo Motor (If used to lower/raise intake) ---
const int SERVO_PIN  = 13;

void setup() {
  pinMode(DRIVE_IN1, OUTPUT);
  pinMode(DRIVE_IN2, OUTPUT);
  pinMode(INTAKE_IN1, OUTPUT);
  pinMode(INTAKE_IN2, OUTPUT);
}

void loop() {
  // 1. Turn ON the Intake Roller (Spin inward)
  digitalWrite(INTAKE_IN1, HIGH);
  digitalWrite(INTAKE_IN2, LOW);

  // 2. Drive Forward continuously
  digitalWrite(DRIVE_IN1, HIGH);
  digitalWrite(DRIVE_IN2, LOW);
}