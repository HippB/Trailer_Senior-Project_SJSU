#include <SoftwareSerial.h>

// Bluetooth setup
SoftwareSerial Bluetooth(11, 10); // RX (to TX of Bluetooth), TX (to RX of Bluetooth)

void setup() {
  Serial.begin(9600);      // Debugging via Serial Monitor
  Bluetooth.begin(9600);   // Bluetooth communication
  Serial.println("Waiting for GPS data...");
}

void loop() {
  if (Bluetooth.available()) {
    String data = Bluetooth.readStringUntil('\n'); // Read until newline
    Serial.print("Received: ");
    Serial.println(data);

    if (data.startsWith("GPS")) { // Parse GPS data
      int firstComma = data.indexOf(',');
      int secondComma = data.indexOf(',', firstComma + 1);

      if (firstComma > 0 && secondComma > 0) {
        String latitude = data.substring(firstComma + 1, secondComma);
        String longitude = data.substring(secondComma + 1);
        Serial.println("Parsed GPS Data:");
        Serial.print("Latitude: ");
        Serial.println(latitude);
        Serial.print("Longitude: ");
        Serial.println(longitude);
      } else {
        Serial.println("Invalid GPS data format.");
      }
    }
  }
}
