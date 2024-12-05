
#include <SoftwareSerial.h>
#include <Wire.h>
#include <QMC5883LCompass.h>

// Bluetooth setup
SoftwareSerial Bluetooth(8, 9); // RX (to TX of Bluetooth), TX (to RX of Bluetooth)

// Motor Control Pins
int motor1pin1 = 7;
int motor1pin2 = 10;
int motor2pin1 = 12;
int motor2pin2 = 4;
int enA = 11;  // Motor 1 speed control
int enB = 5;   // Motor 2 speed control

// Compass setup
QMC5883LCompass compass;

// Variables for GPS navigation
float currentLatitude = 0.0;
float currentLongitude = 0.0;
float targetLatitude = 0.0;
float targetLongitude = 0.0;
int gpsUpdateCounter = 0;

void setup() {
  // Initialize Serial and Bluetooth communication
  Serial.begin(9600);
  Bluetooth.begin(9600);

  // Motor setup
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  analogWrite(enA, 255);
  analogWrite(enB, 255);

  // Compass setup
  Wire.begin();
  compass.init(); // Initialize compass
  compass.setMagneticDeclination(12.89, 0.36); // Adjust for your location (example: Londrina, Brazil)
  Serial.println("System Initialized. Waiting for Bluetooth commands...");
}

void loop() {
  // Check for incoming Bluetooth data
  if (Bluetooth.available()) {
    String data = Bluetooth.readStringUntil('\n');
    Serial.print("Received: ");
    Serial.println(data);

    if (data.startsWith("GPS")) {
      processGPSData(data);
    }
  }

  // Check current position and navigate every second
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck > 1000 && gpsUpdateCounter > 0) {
    navigateToTarget();
    lastCheck = millis();
  }
}

void processGPSData(String data) {
  int firstComma = data.indexOf(',');
  int secondComma = data.indexOf(',', firstComma + 1);

  if (firstComma > 0 && secondComma > 0) {
    String latitudeStr = data.substring(firstComma + 1, secondComma);
    String longitudeStr = data.substring(secondComma + 1);

    float receivedLatitude = latitudeStr.toFloat();
    float receivedLongitude = longitudeStr.toFloat();

    // Print parsed coordinates
    Serial.print("Parsed Latitude: ");
    Serial.println(receivedLatitude, 6);
    Serial.print("Parsed Longitude: ");
    Serial.println(receivedLongitude, 6);

    // Alternate between updating current and target coordinates
    if (gpsUpdateCounter % 2 == 0) {
      currentLatitude = receivedLatitude;
      currentLongitude = receivedLongitude;
      Serial.println("Updated Current Location.");
    } else {
      targetLatitude = receivedLatitude;
      targetLongitude = receivedLongitude;
      Serial.println("Updated Target Location.");
    }
    gpsUpdateCounter++;
  } else {
    Serial.println("Invalid GPS data format.");
  }
}

void navigateToTarget() {
  float distanceCm = calculateDistance(currentLatitude, currentLongitude, targetLatitude, targetLongitude) * 100;
  float targetBearing = calculateBearing(currentLatitude, currentLongitude, targetLatitude, targetLongitude);

  Serial.print("Distance to Target: ");
  Serial.print(distanceCm);
  Serial.println(" cm");

  int heading = getCompassHeading();

  Serial.print("Target Bearing: ");
  Serial.println(targetBearing);
  Serial.print("Current Heading: ");
  Serial.println(heading);

  // Calculate the angle difference between the heading and target bearing
  float angleDifference = targetBearing - heading;

  // Normalize the angle difference to -180° to 180°
  if (angleDifference > 180) {
    angleDifference -= 360;
  }
  if (angleDifference < -180) {
    angleDifference += 360;
  }

  Serial.print("Angle Difference: ");
  Serial.println(angleDifference);

  if (distanceCm > 50) { // Threshold: 50 cm
    if (angleDifference > 5) {
      turnRight();
      Serial.println("Turning Right...");
    } else if (angleDifference < -5) {
      turnLeft();
      Serial.println("Turning Left...");
    } else {
      moveForward();
      Serial.println("Moving Forward...");
    }
  } else {
    stopMotors();
    Serial.println("Target reached. Awaiting new coordinates...");
  }
}


float calculateDistance(float lat1, float lon1, float lat2, float lon2) {
  const float R = 6371e3; // Earth's radius in meters
  float phi1 = radians(lat1);
  float phi2 = radians(lat2);
  float deltaPhi = radians(lat2 - lat1);
  float deltaLambda = radians(lon2 - lon1);

  float a = sin(deltaPhi / 2) * sin(deltaPhi / 2) +
            cos(phi1) * cos(phi2) * sin(deltaLambda / 2) * sin(deltaLambda / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return R * c; // Return distance in meters
}

float calculateBearing(float lat1, float lon1, float lat2, float lon2) {
  float phi1 = radians(lat1);
  float phi2 = radians(lat2);
  float deltaLambda = radians(lon2 - lon1);

  float y = sin(deltaLambda) * cos(phi2);
  float x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(deltaLambda);

  return fmod((degrees(atan2(y, x)) + 360.0), 360.0); // Normalize to 0-360 degrees
}


int getCompassHeading() {
  compass.read(); // Update compass readings
  int azimuth = compass.getAzimuth(); // Get azimuth (heading) in degrees

  // Normalize to 0-360° range
  if (azimuth < 0) {
    azimuth += 180;
  }

  return azimuth;
}


// Motor control functions
void moveForward() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}

void turnLeft() { 
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
}

void turnRight() { 
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
}

void stopMotors() {
  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);
}
