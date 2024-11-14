#include <SoftwareSerial.h>
#include <Wire.h>
#include <QMC5883LCompass.h>
#include <TinyGPSPlus.h>
#include <math.h>

int motor1pin1 = 7;
int motor1pin2 = 8;

int motor2pin1 = 12;
int motor2pin2 = 4;

int enA = 9;  // Motor 1 speed control (optional)
int enB = 5;  // Motor 2 speed control (optional)

// Bluetooth module connected to pins 10 (TX of HC-05) and 11 (RX of HC-05)
SoftwareSerial Bluetooth(10, 11); // RX, TX
TinyGPSPlus gps;
SoftwareSerial gpsSerial(3, 6);  // RX, TX for GPS
QMC5883LCompass compass;

float lastLat = 0, lastLng = 0;
bool autoMode = false;  // Flag for mode selection
volatile bool flag = 0;
char command;

void setup() {
  // Start Bluetooth communication
  Bluetooth.begin(9600); // Set baud rate of Bluetooth module
  Serial.begin(9600);    // For debugging
  gpsSerial.begin(9600);  // GPS module setup
  delay(100);

  Wire.begin();
  compass.init(); // Initialize compass  

  // Set up the motor pins
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);

  pinMode(enA, OUTPUT); // (Optional) Motor speed control
  pinMode(enB, OUTPUT); // (Optional) Motor speed control
  
  // Set initial motor speed
  analogWrite(enA, 255); // Speed control for motor 1 (optional)
  analogWrite(enB, 255); // Speed control for motor 2 (optional)

    Serial.println("GPS Test Starting...");
  
  // Quick GPS check
  Serial.println("Checking for GPS data...");
  unsigned long startTime = millis();
  bool gpsFound = false;
  
  // Wait up to 5 seconds for GPS data
  while (millis() - startTime < 5000) {
    if (gpsSerial.available()) {
      gpsFound = true;
      Serial.println("GPS data detected!");
      break;
    }
  }
  
  if (!gpsFound) {
    Serial.println("Warning: No GPS data detected in first 5 seconds");
  }

  Serial.println("Setup complete. Waiting for Bluetooth commands...");
}

void loop() {
  Serial.print("Enter command: ");
  if (Serial.available() > 0){
    command = Serial.read();
    Serial.println(command);
    
    if (command == 'A'){
        autoMode = true;
        flag = 1;
        GPSautoMode();  // Switch to GPS-based autonomous mode
        Serial.println("Auto mode enabled.");
    } else if (command == 'M') {
        autoMode = false;
        flag = 1;
        Serial.println("Manual mode enabled.");
    } else if (!autoMode){
        handleManualControl(command);
    } else if (command == 'O'){
      reset();
    }
  }
}

/* Stopping DC motors and bringing servo motor to original position */
void reset() {
    stopMotors();
    flag = 0;
    Serial.println("Resetting...");
}

// Command interpretation for motor control
void handleManualControl(char command){
  if (flag == 1){
    switch (command) {
      case 'F': // Forward
        Serial.println("Moving forward");
        moveForward();
        break;
      case 'B': // Backward
        Serial.println("Moving backward");
        moveBackward();
        break;
      case 'L': // Left
        Serial.println("Turning left");
        turnLeft();
        break;
      case 'R': // Right
        Serial.println("Turning right");
        turnRight();
        break;
      case 'S': // Stop
        Serial.println("Stopping motors");
        stopMotors();
        break;
      default:
        Serial.println("Unrecognized command");
        break;
    }
  }
}

// Autonomous GPS mode
void GPSautoMode() {
  if (flag == 1){
    unsigned long startTime = millis();
    const unsigned long TIMEOUT = 1000; // 1 second timeout

    Serial.println("GPS enabled 1");
    char c = gpsSerial.read(); // debugging
    Serial.println(c);
    while (gpsSerial.available() > 0) {
      Serial.println("GPS enalbed 2");
      if (millis() - startTime > TIMEOUT) {
        Serial.println("GPS read timeout");
        return;
      }
      
      char c = gpsSerial.read();
      if (gps.encode(c)) {
        if (gps.location.isUpdated()) {
          float lat = gps.location.lat();
          float lng = gps.location.lng();

          float angleToTarget = calculateBearing(lastLat, lastLng, lat, lng);
          float distance = calculateDistance(lastLat, lastLng, lat, lng);

          if (distance > 5.0) {  // If target is more than 5 meters away
            moveTowards(angleToTarget);  // Move towards the target
          } else {
            Serial.println("Reached target.");
            stopMotors();  // Stop when close to target
          }

          // Update last known position
          lastLat = lat;
          lastLng = lng;
        }
      }
    }
    delay(10);
  }
}

// Function to calculate the bearing between two coordinates
float calculateBearing(float lat1, float lng1, float lat2, float lng2) {
  float y = sin(lng2 - lng1) * cos(lat2);
  float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lng2 - lng1);
  float brng = atan2(y, x);
  return degrees(brng);  // Convert radians to degrees
}

// Function to calculate the distance between two coordinates (Haversine formula)
float calculateDistance(float lat1, float lng1, float lat2, float lng2) {
  const float R = 6371e3;  // Earth radius in meters
  float phi1 = radians(lat1);
  float phi2 = radians(lat2);
  float deltaPhi = radians(lat2 - lat1);
  float deltaLambda = radians(lng2 - lng1);

  float a = sin(deltaPhi / 2) * sin(deltaPhi / 2) +
            cos(phi1) * cos(phi2) *
            sin(deltaLambda / 2) * sin(deltaLambda / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));

  float distance = R * c;
  return distance;  // Distance in meters
}

// Motor control functions
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

// Move towards target direction based on the angle
void moveTowards(float targetAngle) {
  compass.read();
  int x = compass.getX();
  int y = compass.getY();

  float currentHeading = atan2(y, x) * 180 / PI;
  if (currentHeading < 0) currentHeading += 360;

  float angleDifference = targetAngle - currentHeading;

  if (angleDifference > 0) {
    turnRight();
  } else {
    turnLeft();
  }

  Serial.print("Current Heading: ");
  Serial.println(currentHeading);
  Serial.print("Target Angle: ");
  Serial.println(targetAngle);
  delay(500);  // Read every half second
}