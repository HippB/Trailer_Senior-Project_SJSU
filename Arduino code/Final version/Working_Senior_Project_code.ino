#include <SoftwareSerial.h>

int motor1pin1 = 7;
int motor1pin2 = 10;

int motor2pin1 = 12;
int motor2pin2 = 4;

int enA = 11;  // Motor 1 speed control (optional)
int enB = 5;  // Motor 2 speed control (optional)

// Bluetooth module connected to pins 10 (TX of HC-05) and 11 (RX of HC-05)
SoftwareSerial Bluetooth(8, 9); // RX, TX

void setup() {
  // Set up the motor pins
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);

  pinMode(enA, OUTPUT); // (Optional) Motor speed control
  pinMode(enB, OUTPUT); // (Optional) Motor speed control
  
  // Start Bluetooth communication
  Bluetooth.begin(9600); // Set baud rate of Bluetooth module
  Serial.begin(9600);    // For debugging

  // Set initial motor speed
  analogWrite(enA, 255); // Speed control for motor 1 (optional)
  analogWrite(enB, 255); // Speed control for motor 2 (optional)
}

void loop() {
  // Check if any data is received from the Bluetooth app
  if (Bluetooth.available()) {
    char command = Bluetooth.read();  // Read the command sent from the app
    Serial.println(command);          // For debugging

    // Command interpretation for motor control
    switch (command) {
      case 'F': // Forward
        moveForward();
        break;
      case 'B': // Backward
        moveBackward();
        break;
      case 'L': // Left
        turnLeft();
        break;
      case 'R': // Right
        turnRight();
        break;
      case 'S': // Stop
        stopMotors();
        break;
      default:
        // Ignore unrecognized commands
        break;
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
