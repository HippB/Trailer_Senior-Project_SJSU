#include <SoftwareSerial.h>

// Set up Bluetooth Serial
SoftwareSerial btSerial(6, 7);  // RX, TX for Bluetooth module

void setup() {
  Serial.begin(9600);           // Start the Serial Monitor
  btSerial.begin(9600);         // Start Bluetooth serial communication
  Serial.println("Waiting for GPS data from Bluetooth...");
}

void loop() {
  if (btSerial.available()) {
    String gpsData = "";

    // Read until newline character to get a full sentence
    while (btSerial.available()) {
      char c = btSerial.read();
      if (c == '\n') break;
      gpsData += c;
    }

    Serial.print("Full data received: ");
    Serial.println(gpsData);

    // Check if it’s a known NMEA sentence and proceed
    if (gpsData.startsWith("$GPGGA")) {
      Serial.println("Valid GPGGA sentence detected");

      // Parse latitude and longitude for $GPGGA
      int firstComma = gpsData.indexOf(',');             // Skip time field
      int latStart = gpsData.indexOf(',', firstComma + 1) + 1;
      int latEnd = gpsData.indexOf(',', latStart);
      int latHemStart = latEnd + 1;
      int lonStart = gpsData.indexOf(',', latHemStart) + 1;
      int lonEnd = gpsData.indexOf(',', lonStart);
      int lonHemStart = lonEnd + 1;

      String latitude = gpsData.substring(latStart, latEnd);
      String latHemisphere = gpsData.substring(latHemStart, latHemStart + 1);
      String longitude = gpsData.substring(lonStart, lonEnd);
      String lonHemisphere = gpsData.substring(lonHemStart, lonHemStart + 1);

      Serial.print("Latitude: ");
      Serial.print(latitude);
      Serial.print(" ");
      Serial.println(latHemisphere);
      Serial.print("Longitude: ");
      Serial.print(longitude);
      Serial.print(" ");
      Serial.println(lonHemisphere);

    } else if (gpsData.startsWith("$GPRMC")) {
      Serial.println("Valid GPRMC sentence detected");

      // Parse latitude and longitude for $GPRMC
      int firstComma = gpsData.indexOf(',', 12);             // Skip initial fields
      int latStart = gpsData.indexOf(',', firstComma + 1) + 1;
      int latEnd = gpsData.indexOf(',', latStart);
      int latHemStart = latEnd + 1;
      int lonStart = gpsData.indexOf(',', latHemStart) + 1;
      int lonEnd = gpsData.indexOf(',', lonStart);
      int lonHemStart = lonEnd + 1;

      String latitude = gpsData.substring(latStart, latEnd);
      String latHemisphere = gpsData.substring(latHemStart, latHemStart + 1);
      String longitude = gpsData.substring(lonStart, lonEnd);
      String lonHemisphere = gpsData.substring(lonHemStart, lonHemStart + 1);

      Serial.print("Latitude: ");
      Serial.print(latitude);
      Serial.print(" ");
      Serial.println(latHemisphere);
      Serial.print("Longitude: ");
      Serial.print(longitude);
      Serial.print(" ");
      Serial.println(lonHemisphere);

    } else {
      Serial.println("Data does not contain GPRMC or GPGGA GPS coordinates");
    }

    // Slow down the output
    delay(5000);
  }
}
