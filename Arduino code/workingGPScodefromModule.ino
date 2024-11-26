#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

// Set up GPS and Bluetooth
SoftwareSerial gpsSerial(6, 3);  // RX (to TX of Bluetooth), TX (to RX of Bluetooth)
TinyGPSPlus gps;                // TinyGPSPlus object

void setup() {
  // Initialize Serial communication
  Serial.begin(9600);      // For debugging via Serial Monitor
  gpsSerial.begin(9600);   // For communication with GPS module or HC-05
  Serial.println("System Initialized. Waiting for GPS data...");
}

void loop() {
  // Read data from GPS (Bluetooth or Serial)
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    // Feed data to TinyGPSPlus library
    if (gps.encode(c)) {
      // If a full sentence has been decoded, display parsed data
      displayGPSData();
    }
  }
}

void displayGPSData() {
  Serial.println(); // Add a newline for clarity
  
  if (gps.location.isValid()) {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6); // 6 decimal places
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6); // 6 decimal places
  } else {
    Serial.println("Location not valid.");
  }

  if (gps.date.isValid()) {
    Serial.print("Date: ");
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.print(gps.date.day());
    Serial.print("/");
    Serial.println(gps.date.year());
  } else {
    Serial.println("Date not valid.");
  }

  if (gps.time.isValid()) {
    Serial.print("Time: ");
    Serial.print(gps.time.hour());
    Serial.print(":");
    Serial.print(gps.time.minute());
    Serial.print(":");
    Serial.println(gps.time.second());
  } else {
    Serial.println("Time not valid.");
  }

  Serial.println("-------------------------"); // Divider for readability
}
