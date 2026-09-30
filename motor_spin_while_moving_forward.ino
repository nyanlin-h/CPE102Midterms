#include <InEngMotor.h> // Import preset library for motor commands

// --- Servo Configuration ---
#define SERVO_PIN 13       // Set GPIO 13 as Servo Pin
#define PWM_FREQ 50        // 50 Hz Frequency
#define PWM_RESOLUTION 16  // 16-bit resolution

// --- Intake Roller Motor Pins ---
#define MR_IN1 36          // Intake motor IN1
#define MR_IN2 39          // Intake motor IN2
#define ENA 34             // Intake motor speed pin (PWM)

void setup() {
  // Start the library (handles pin modes and PWM timers internally)
  inengmotor.begin(); 

  // 2. Attach Servo PWM on GPIO 13
  ledcAttach(SERVO_PIN, PWM_FREQ, PWM_RESOLUTION);

  // 3. Set Intake Motor Pins as Outputs
  pinMode(MR_IN1, OUTPUT);
  pinMode(MR_IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
}

void servoWrite(int angle) {
  int pulseWidth = map(angle, 0, 180, 500, 2500);
  uint32_t duty = (pulseWidth * 65535UL) / 20000UL;
  ledcWrite(SERVO_PIN, duty); 
}

void loop() {
  // Forward: (leftspeed, rightspeed)
  inengmotor.forward(130, 255);

  // Turn ON the Intake Roller Motor
  digitalWrite(MR_IN1, HIGH);
  digitalWrite(MR_IN2, LOW);
  
  // Ramp up intake roller speed
  for (int speed = 0; speed <= 255; speed += 5) {
    analogWrite(ENA, speed);
    delay(50);
  }

  // Servo positions with 8-second delays
  servoWrite(0);
  delay(8000);

  servoWrite(60);
  delay(8000);

  servoWrite(0);
  delay(8000);

  servoWrite(180);
  delay(8000);

  servoWrite(0);
  delay(8000);
}
