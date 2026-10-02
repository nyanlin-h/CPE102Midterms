#include <InEngMotor.h>  // Import preset library for motor commands

// --- Servo Configuration ---
#define SERVO_PIN 33       // Set GPIO 33 as Servo Pin
#define PWM_FREQ 50        // 50 Hz Frequency
#define PWM_RESOLUTION 16  // 16-bit resolution

// --- Intake Roller Motor Pins ---
#define MR_IN1 21  // Intake motor IN1
#define MR_IN2 22  // Intake motor IN2
#define ENA 23     // Intake motor speed pin (PWM)

// ==========================================
// --- TCS3200 Color Sensor Configuration ---
// ==========================================
#define S0_PIN 14
#define S1_PIN 13
#define S2_PIN 4
#define S3_PIN 5
#define OUT_PIN 35  // Input-only pin (Safe for pulseIn)
int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;

void setup() {
 Serial.begin(115200); // serial monitor for calibrating gem color

  // 1. Initialize Drive Chassis (InEngMotor manages drive pins internally)
  inengmotor.begin();

  // 2. Attach Servo PWM on GPIO 33
  ledcAttach(SERVO_PIN, PWM_FREQ, PWM_RESOLUTION);

  // 3. Set Intake Motor Pins as Outputs
  pinMode(MR_IN1, OUTPUT);
  pinMode(MR_IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  // ------------------------------------------
  // --- TCS3200 Pin Setup (Commented Out) ---
  // ------------------------------------------
  pinMode(S0_PIN, OUTPUT);
  pinMode(S1_PIN, OUTPUT);
  pinMode(S2_PIN, OUTPUT);
  pinMode(S3_PIN, OUTPUT);
  pinMode(OUT_PIN, INPUT);

  // set Frequency Scaling to 20% (recommended)
  digitalWrite(S0_PIN, HIGH);
  digitalWrite(S1_PIN, LOW);
}

void servoWrite(int angle) {
  int pulseWidth = map(angle, 0, 180, 500, 2500);
  uint32_t duty = (pulseWidth * 65535UL) / 20000UL;
  ledcWrite(SERVO_PIN, duty);
}
// ==========================================
// --- Function Read Color Sensor (TCS3200) -
// ==========================================
void readColorSensor() {
  // อ่านค่าสีแดง (Filter: Red)
  digitalWrite(S2_PIN, LOW);
  digitalWrite(S3_PIN, LOW);
  redFrequency = pulseIn(OUT_PIN, LOW);

  //read green (Filter: Green)
  digitalWrite(S2_PIN, HIGH);
  digitalWrite(S3_PIN, HIGH);
  greenFrequency = pulseIn(OUT_PIN, LOW);

  //read blue (Filter: Blue)
  digitalWrite(S2_PIN, LOW);
  digitalWrite(S3_PIN, HIGH);
  blueFrequency = pulseIn(OUT_PIN, LOW);

  // output at Serial Monitor
  Serial.print("R: "); Serial.print(redFrequency);
  Serial.print(" G: "); Serial.print(greenFrequency);
  Serial.print(" B: "); Serial.println(blueFrequency);
  delay(500);
}

void loop() {
  // Forward: (leftspeed, rightspeed)
   inengmotor.forward(110, 255);
   delay(5000);

  // inengmotor.backward(130, 255);
  // delay(5000);

  // Turn ON the Intake Roller Motor
   digitalWrite(MR_IN1, HIGH);
   digitalWrite(MR_IN2, LOW);

  // Ramp up intake roller speed
   analogWrite(ENA, 200);
   delay(10000);

  // --- เรียกใช้งานฟังก์ชันอ่านค่าสี ---
  readColorSensor();

  // Servo positions with 8-second delays
 servoWrite(0);
   delay(8000);

   servoWrite(180);
   delay(8000);

  // servoWrite(0);
  // delay(8000);
}