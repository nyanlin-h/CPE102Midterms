const int LEFT_MOTOR_PIN1 = 0; // Replace with actual pin later
const int LEFT_MOTOR_PIN2 = 0; // Replace with actual pin later
const int RIGHT_MOTOR_PIN1 = 0; // Replace with actual pin later
const int RIGHT_MOTOR_PIN2 = 0; // Replace with actual pin later

void moveForward() {
  // Left motor forward
  digitalWrite(LEFT_MOTOR_PIN1, HIGH);
  digitalWrite(LEFT_MOTOR_PIN2, LOW);

  // Right motor forward
  digitalWrite(RIGHT_MOTOR_PIN1, HIGH);
  digitalWrite(RIGHT_MOTOR_PIN2, LOW);
}

void moveBackward() {
  // Left motor backward
  digitalWrite(LEFT_MOTOR_PIN1, LOW);
  digitalWrite(LEFT_MOTOR_PIN2, HIGH);

  // Right motor backward
  digitalWrite(RIGHT_MOTOR_PIN1, LOW);
  digitalWrite(RIGHT_MOTOR_PIN2, HIGH);
}

void turnLeft() {
  // Left motor backward
  digitalWrite(LEFT_MOTOR_PIN1, LOW);
  digitalWrite(LEFT_MOTOR_PIN2, HIGH);

  // Right motor forward
  digitalWrite(RIGHT_MOTOR_PIN1, HIGH);
  digitalWrite(RIGHT_MOTOR_PIN2, LOW);
}

void turnRight() {
  // Left motor forward
  digitalWrite(LEFT_MOTOR_PIN1, HIGH);
  digitalWrite(LEFT_MOTOR_PIN2, LOW);

  // Right motor backward
  digitalWrite(RIGHT_MOTOR_PIN1, LOW);
  digitalWrite(RIGHT_MOTOR_PIN2, HIGH);
}

void stop() {
  // Left motor cut
  digitalWrite(LEFT_MOTOR_PIN1, LOW);
  digitalWrite(LEFT_MOTOR_PIN2, LOW);

  // Right motor cut
  digitalWrite(RIGHT_MOTOR_PIN1, LOW);
  digitalWrite(RIGHT_MOTOR_PIN2, LOW);
}


void setup() {
  // put your setup code here, to run once:
  pinMode(LEFT_MOTOR_PIN1, OUTPUT);
  pinMode(LEFT_MOTOR_PIN2, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN1, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN2, OUTPUT);

  stop();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    char command = Serial.read();

    switch (command) {
      case 'F':
        moveForward();
        break;

      case 'B';
        moveBackward();
        break;

      case 'L';
        turnLeft();
        break;

      case 'R';
        turnRight();
        break;

      case 'S';
        stop();
        break;

      default:
        stop();
        break;
    }
  }

}
