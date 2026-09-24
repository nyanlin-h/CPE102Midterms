// motor driver pins
const int ENA = 5;  // PWM pin for speed
const int IN1 = 7;  // Direction pin 1
const int IN2 = 8;  // Direction pin 2

// Define pins for encoder
const int encoderPinA = 2; // Interrupt pin
const int encoderPinB = 3; 

volatile long encoderValue = 0;

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  
  // Trigger interrupt on rising edge for encoder pulse counting
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, RISING);
  
  Serial.begin(9600);
}

void loop() {
  // Set motor direction forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  
  // Set motor speed (0 - 255)
  analogWrite(ENA, 150); 
  
  // Print encoder count to Serial Monitor
  Serial.print("Encoder Position: ");
  Serial.println(encoderValue);
  delay(100);
}

void updateEncoder() {
  // Read encoder B to determine rotation direction
  if (digitalRead(encoderPinB) == HIGH) {
    encoderValue++;
  } else {
    encoderValue--;
  }
}
