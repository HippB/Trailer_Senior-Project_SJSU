#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>

// Set up Bluetooth Serial
SoftwareSerial btSerial(6, 7);  // RX, TX for Bluetooth module
TinyGPSPlus gps;                // Create a TinyGPSPlus object

void setup() {
  Serial.begin(9600);           // Start the Serial Monitor
  btSerial.begin(9600);         // Start Bluetooth serial communication
  Serial.println("Waiting for GPS data from Bluetooth...");
}

void loop() {
  // Read data from Bluetooth and pass it to TinyGPS++ for parsing
  while (btSerial.available() > 0) {
    char c = btSerial.read();
    gps.encode(c);  // Pass each byte to TinyGPS++ for processing

    // If TinyGPS++ has a valid and updated location, display it
    if (gps.location.isUpdated()) {
      displayGPSData();
    }
  }
}

void displayGPSData() {
  Serial.println("GPS Data:");

  // Display latitude and longitude
  Serial.print("Latitude: ");
  Serial.print(gps.location.lat(), 6); // 6 decimal places for precision
  Serial.print(" ");
  Serial.println(gps.location.rawLat().negative ? "S" : "N");

  Serial.print("Longitude: ");
  Serial.print(gps.location.lng(), 6); // 6 decimal places for precision
  Serial.print(" ");
  Serial.println(gps.location.rawLng().negative ? "W" : "E");

  // Display date
  if (gps.date.isValid()) {
    Serial.print("Date (DD/MM/YYYY): ");
    Serial.print(gps.date.day());
    Serial.print("/");
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.println(gps.date.year());
  } else {
    Serial.println("Date: Not available");
  }

  // Display time
  if (gps.time.isValid()) {
    Serial.print("Time (HH:MM:SS): ");
    Serial.print(gps.time.hour());
    Serial.print(":");
    Serial.print(gps.time.minute());
    Serial.print(":");
    Serial.println(gps.time.second());
  } else {
    Serial.println("Time: Not available");
  }

  Serial.println(); // Print a blank line for readability
  delay(5000);      // Slow down output for readability
}
