#include <SoftwareSerial.h>

// Motor pins
int motor1pin1 = 7;
int motor1pin2 = 10;

int motor2pin1 = 12;
int motor2pin2 = 4;

int enA = 11;  // Motor 1 speed control
int enB = 5;   // Motor 2 speed control

// Bluetooth module pins
SoftwareSerial Bluetooth(8, 9); // RX, TX

// Default motor speed
int motorSpeed = 255; // Speed for both motors

void setup() {
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);

  pinMode(enA, OUTPUT); // PWM pin for motor 1
  pinMode(enB, OUTPUT); // PWM pin for motor 2

  Bluetooth.begin(9600); // Start Bluetooth communication
  Serial.begin(9600);    // Debugging
}

void loop() {
  // Check for incoming data
  if (Bluetooth.available()) {
    String command = Bluetooth.readStringUntil('\n'); // Read command as string
    Serial.println(command);                          // Debugging

    if (command.startsWith("SPEED")) {
      // Command format: SPEED,<motorSpeed>
      int commaIndex = command.indexOf(',');
      motorSpeed = command.substring(commaIndex + 1).toInt(); // Extract speed

      // Constrain speed to 0-255
      motorSpeed = constrain(motorSpeed, 0, 255);

      // Update motor speeds
      analogWrite(enA, motorSpeed);
      analogWrite(enB, motorSpeed);

      Serial.print("MotorSpeed: ");
      Serial.println(motorSpeed);
    } else {
      // Handle directional commands
      char dir = command[0];
      switch (dir) {
        case 'F':
          moveForward();
          break;
        case 'B':
          moveBackward();
          break;
        case 'L':
          turnLeft();
          break;
        case 'R':
          turnRight();
          break;
        case 'S':
          stopMotors();
          break;
        default:
        break;
      }
    }
  }
}

void moveForward() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}

void moveBackward() {
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
}

void turnLeft() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
}

void turnRight() {
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}

void stopMotors() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);
}