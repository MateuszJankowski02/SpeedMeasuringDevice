#include "TinyGPS++.h"
#include "HardwareSerial.h"

HardwareSerial SerialGPS(1);
TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);
}

void loop() {
  while (SerialGPS.available()) {
    gps.encode(SerialGPS.read());
    if (gps.location.isUpdated()) {
      Serial.print("Latitude= ");
      Serial.print(gps.location.lat(), 6);
      Serial.print(" Longitude= ");
      Serial.println(gps.location.lng(), 6);
    }
  }
}