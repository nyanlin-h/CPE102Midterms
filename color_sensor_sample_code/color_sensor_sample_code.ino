#define S0 14
#define S1 13
#define S2 4
#define S3 5
#define sensorOut 35

int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;

void setup() {
  // Set pin modes
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  // Set frequency scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  // Begin serial communication
  Serial.begin(115200);
}

void loop() {
  // Read Red component
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  redFrequency = pulseIn(sensorOut, LOW);

  // Read Green component
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  greenFrequency = pulseIn(sensorOut, LOW);

  // Read Blue component
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  blueFrequency = pulseIn(sensorOut, LOW);

  // Print raw frequency values to the Serial Monitor
  Serial.print("R = ");
  Serial.print(redFrequency);
  Serial.print(" | G = ");
  Serial.print(greenFrequency);
  Serial.print(" | B = ");
  Serial.println(blueFrequency);

  delay(500);
}
