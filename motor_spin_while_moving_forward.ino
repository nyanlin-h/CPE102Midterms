#include <InEngMotor.h> //Import preset libary for motor commands

#define SERVO_PIN 19 // Set GPIO PIN 19 as Servo Pin

#define PWM_FREQ 50       // Define Frequency 50 Hz
#define PWM_RESOLUTION 16 // 16-bit

#define MR_IN1 26 //backright motor
#define MR_IN2 27
#define MR_IN1 12 //backright motor
#define ENA 15

 // --- Motor A (Drive Motor) ---
const int DRIVE_IN1 = 16; // Connected to AI1
const int DRIVE_IN2 = 17; // Connected to AI2

// --- Servo Motor (If used to lower/raise intake) ---
const int SERVO_PIN  = 13;

void setup() {
  //Start the libary
  inengmotor.begin();

  ledcAttach(SERVO_PIN, PWM_FREQ, PWM_RESOLUTION); // Set Frequency and Resolution for Servo Pin

  pinMode(MR_IN1, OUTPUT);
  pinMode(MR_IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
}

void servoWrite(int angle) {
  // Map Servo pulse and angle
  int pulseWidth = map(angle, 0, 180, 500, 2500);

  // Calculate Duty Cycle
  uint32_t duty = (pulseWidth * 65535UL) / 20000UL;

  // Set the servo angle
  ledcWrite(SERVO_PIN, duty); 
}

void loop() {
  //Forward: (leftspeed, rightspeed)
  inengmotor.forward(200, 200); 

  // 1. Turn ON the Intake Roller (Spin inward)
  digitalWrite(MR_IN1, HIGH);
  digitalWrite(MR_IN2, LOW);
  
  for (int speed = 0; speed <= 255; speed += 5) {
    analogWrite(ENA, speed); // PWM control
    delay(50);
  }

  // Set servo angle to 0
  servoWrite(0);
  delay(8000);

  // Set servo angle to 60
  servoWrite(60);
  delay(8000);

  // Set servo angle to 0
  servoWrite(0);
  delay(8000);

  // Set servo angle to 180
  servoWrite(180);
  delay(8000);

  // Set servo angle to 0
  servoWrite(0);
  delay(8000);
}
