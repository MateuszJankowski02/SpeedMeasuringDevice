#include <HardwareSerial.h>
#include <TinyGPS++.h>

HardwareSerial GPSSerial(1);
TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);
  GPSSerial.begin(9600, SERIAL_8N1, 16, 17); // RX, TX

  // UBX command to set the update rate to 5Hz
  byte UBLOX_INIT[] = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A
  };

  // UBX command to set the update rate to 1Hz
  /*
  byte UBLOX_INIT[] = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xE8, 0x03, 0x01, 0x00, 0x01, 0x00, 0x01, 0x39
  };
  */

  GPSSerial.write(UBLOX_INIT, sizeof(UBLOX_INIT));
}

unsigned long previousMillis = 0;

void loop() {
  while (GPSSerial.available()) {
    char c = GPSSerial.read();
    if (gps.encode(c)) {
      if (gps.location.isUpdated() && gps.location.isValid()) {
        unsigned long currentMillis = millis();
        Serial.print("Time since last update: ");
        Serial.print(currentMillis - previousMillis);
        Serial.println(" ms");
        previousMillis = currentMillis;

        Serial.print("Latitude: ");
        Serial.println(gps.location.lat(), 6);
        Serial.print("Longitude: ");
        Serial.println(gps.location.lng(), 6);
      }
    }
  }
  //delay(1000); // Wait for 1 second
  delay(200); // Wait for 0.2 second
}




/*
void setup() {
  Serial.begin(115200);
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);

  delay(500);
  // Send command to set the baud rate to 38400
  SerialGPS.print("$PMTK251,57600*2C\r\n");
  delay(500);

  // Send command to set the update rate to 5Hz
  SerialGPS.print("$PMTK220,200*2C\r\n");
  delay(500);

  // Now begin the SerialGPS at the new baud rate
  SerialGPS.begin(38400, SERIAL_8N1, 16, 17);
  delay(500);

  // Send command to query firmware info
  SerialGPS.print("$PMTK605*31\r\n");
  delay(500);
}

int queryIndex = 0;

void loop() {
  while (SerialGPS.available()) {
    gps.encode(SerialGPS.read());
    if (gps.location.isUpdated()) {
      Serial.println("123");
      Serial.println(queryIndex);
      queryIndex++;
      Serial.print("Latitude= ");
      Serial.print(gps.location.lat(), 6);
      Serial.print(" Longitude= ");
      Serial.println(gps.location.lng(), 6);
    }
  }
}
*/