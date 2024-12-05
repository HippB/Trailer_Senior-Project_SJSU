#include <ArduinoBLE.h>
#include <Dabble.h>
#include <TinyGPSPlus.h>
#include <QMC5883LCompass.h>
#include <Wire.h>

// BLE Service and Characteristics
BLEService robotService("180A"); // Custom BLE Service
BLECharacteristic commandChar("2A56", BLEWrite | BLEWriteWithoutResponse, 1); // Command Characteristic
BLECharacteristic gpsChar("2A58", BLERead, 20); // GPS Data Characteristic

// Motor Control Pins
#define motor1pin1 7
#define motor1pin2 8
#define motor2pin1 12
#define motor2pin2 4
#define enA 9
#define enB 10

// Global variables
bool autoMode = false;
char command;
TinyGPSPlus gps;
QMC5883LCompass compass;

float lastLat = 0, lastLng = 0;

// Setup function
void setup() {
  Serial.begin(9600);
  while (!Serial); // Wait for Serial Monitor to connect

  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1);
  }

  BLE.setLocalName("ArduinoR4Robot");
  BLE.setAdvertisedService(robotService);
  robotService.addCharacteristic(commandChar);
  robotService.addCharacteristic(gpsChar);
  BLE.addService(robotService);
  BLE.advertise();

  // Initialize Dabble
  Dabble.begin(9600);

  // Initialize Motor Pins
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  analogWrite(enA, 255); // Set full speed
  analogWrite(enB, 255); // Set full speed

  Wire.begin();
  compass.init();

  Serial.println("Setup complete. Waiting for commands...");
}

void loop() {
  // Process BLE communication
  BLEDevice central = BLE.central();
  if (central) {
    while (central.connected()) {
      processBLECommands();
    }
  }

  // Process Dabble commands
  Dabble.processInput();
  if (GamePad.isConnected()) {
    handleGamePadControls();
  }

  // Execute GPS autonomous mode if enabled
  if (autoMode) {
    GPSautoMode();
  }
}

/*
// Process BLE commands
void processBLECommands() {
  if (commandChar.written()) {
    uint8_t receivedCommand;
    commandChar.readValue(receivedCommand);
    command = (char)receivedCommand;

    Serial.print("BLE Command received: ");
    Serial.println(command);

    if (command == 'A') {
      autoMode = true;
      Serial.println("Switched to Auto Mode");
    } else if (command == 'M') {
      autoMode = false;
      Serial.println("Switched to Manual Mode");
    } else if (autoMode) {
      // Process autonomous mode BLE commands (if any additional logic needed)
    } else {
      handleManualControl(command);
    }
  }

  if (autoMode) {
    String gpsData = "GPS,12.3456,78.9101"; // Mock GPS data
    gpsChar.writeValue(gpsData.c_str());
    Serial.println("Sent GPS data: " + gpsData);
    GPSautoMode();
  }
}
*/

// Handle GamePad controls from Dabble
void handleGamePadControls() {
  if (GamePad.isUpPressed()) {
    moveForward();
    Serial.println("Forward.");
  } else if (GamePad.isDownPressed()) {
    moveBackward();
    Serial.println("Down.");
  } else if (GamePad.isLeftPressed()) {
    turnLeft();
    Serial.println("Left.");
  } else if (GamePad.isRightPressed()) {
    turnRight();
    Serial.println("Right.");
  } else {
    stopMotors();
  }

  if (GamePad.isSquarePressed()) { // Square button for Auto Mode
    autoMode = true;
    Serial.println("Auto mode enabled.");
  } else if (GamePad.isCirclePressed()) { // Circle button for Manual Mode
    autoMode = false;
    Serial.println("Manual mode enabled.");
  }
}

// GPS-based autonomous navigation
void GPSautoMode() {
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

    // update arduino GPS data
    lastLat = lat;
    lastLng = lng;
  }
}

// Motor control functions
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

// Bearing and distance calculations
float calculateBearing(float lat1, float lng1, float lat2, float lng2) {
  // Implement bearing calculation
  float y = sin(lng2 - lng1) * cos(lat2);
  float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lng2 - lng1);
  return degrees(atan2(y, x));  // Convert radians to degrees
}

float calculateDistance(float lat1, float lng1, float lat2, float lng2) {
  // Implement distance calculation
  const float R = 6371e3;  // Earth radius in meters
  float phi1 = radians(lat1);
  float phi2 = radians(lat2);
  float deltaPhi = radians(lat2 - lat1);
  float deltaLambda = radians(lng2 - lng1);

  float a = sin(deltaPhi / 2) * sin(deltaPhi / 2) +
            cos(phi1) * cos(phi2) *
            sin(deltaLambda / 2) * sin(deltaLambda / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;  // Distance in meters
}

void moveTowards(float targetAngle) {
  // Implement movement logic towards a target angle
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
  delay(500);  // Delay for stabilization
}
