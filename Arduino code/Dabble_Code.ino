#include <Dabble.h>
#include <TinyGPSPlus.h>
#include <QMC5883LCompass.h>

// GPS module and compass setup
TinyGPSPlus gps;
QMC5883LCompass compass;

float lastLat = 0, lastLng = 0;
bool autoMode = false;

// Pin configuration for motors
const int motor1pin1 = 7;
const int motor1pin2 = 8;
const int motor2pin1 = 12;
const int motor2pin2 = 4;
const int enA = 9;
const int enB = 10;

void setup() {
  Serial.begin(9600);
  Dabble.begin(9600);  // Initializes Dabble with Bluetooth
  delay(100);

  // Motor setup
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  // Set initial motor speed
  analogWrite(enA, 255); 
  analogWrite(enB, 255); 

  Wire.begin();
  compass.init();  // Initialize the compass

  Serial.println("Setup complete. Waiting for Bluetooth commands...");
}

void loop() {
  // Read GPS data
  while (Serial.available()) {
    gps.encode(Serial.read());
  }

  // Send GPS data to Serial Monitor
  printGPSData();

  Dabble.processInput();  // Processes incoming data from Dabble app

  // Check if Gamepad module is connected
  if (GamePad.isConnected()) {
    handleGamePadControls();
  }

  // Switch between auto and manual modes based on command
  if (autoMode) {
    GPSautoMode();
  }
}

void handleGamePadControls() {
  if (GamePad.isUpPressed()) {
    Serial.println("Moving forward");	
    moveForward();
  } else if (GamePad.isDownPressed()) {
    Serial.println("Moving backward");
    moveBackward();
  } else if (GamePad.isLeftPressed()) {
    Serial.println("Turning left");
    turnLeft();
  } else if (GamePad.isRightPressed()) {
    Serial.println("Turning right");
    turnRight();
  } else {
    stopMotors();
  }

  if (GamePad.isSquarePressed()) {  // Square button for Auto Mode
    autoMode = true;
    Serial.println("Auto mode enabled.");
  } else if (GamePad.isCirclePressed()) {  // Circle button for Manual Mode
    autoMode = false;
    Serial.println("Manual mode enabled.");
  }
}

// Autonomous GPS navigation
void GPSautoMode() {
  // Implement your GPS-based autonomous mode here
  if (gps.location.isUpdated()) {
    float lat = gps.location.lat();
    float lng = gps.location.lng();

    float angleToTarget = calculateBearing(lastLat, lastLng, lat, lng);
    float distance = calculateDistance(lastLat, lastLng, lat, lng);

    if (distance > 5.0) {
      moveTowards(angleToTarget);
    } else {
      Serial.println("Reached target.");
      stopMotors();
    }

    lastLat = lat;
    lastLng = lng;
  }
}

void printGPSData() {
  if (GamePad.isSquarePressed()) {
    Serial.println(" ");
    Serial.println("PIN LOCATION ---> ");
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    delay(1000);
  } 
}

// Movement functions
void moveForward() {
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}

void moveBackward() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
}

void turnLeft() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}

void turnRight() {
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
}

void stopMotors() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);
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
