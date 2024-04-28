#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_ADDR   0x3C
#define GPS_PIN_RX_WHITE 16
#define GPS_PIN_TX_GREEN 17
/*
DISPLAY PINS
SCK = D22
SDA = D21
*/

HardwareSerial GPSSerial(1);
TinyGPSPlus gps;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

unsigned long previousMicros = 0;
double totalDistance = 0.0; // Total distance in meters
unsigned long totalTimeMicros = 0; // Total time elapsed in microseconds
double prevLat = 0.0; // Previous latitude
double prevLon = 0.0; // Previous longitude
double currentSpeedKmph = 0.0;
double averageSpeedKmph = 0.0;

void setup() {
  Serial.begin(115200);
  GPSSerial.begin(9600, SERIAL_8N1, GPS_PIN_RX_WHITE, GPS_PIN_TX_GREEN); // RX, TX

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

  //Initiate display
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  delay(2000);
}

double haversine(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0; // Earth's radius in meters
  double dLat = (lat2 - lat1) * M_PI / 180.0;
  double dLon = (lon2 - lon1) * M_PI / 180.0;
  double a = sin(dLat / 2.0) * sin(dLat / 2.0) +
             cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) *
             sin(dLon / 2.0) * sin(dLon / 2.0);
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return R * c;
}

void displayCurrentSpeed(){
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(1, 1);
  display.println("Current Speed: ");
  display.setCursor(90, 1);
  display.println(currentSpeedKmph, 2);
}

void displayAverageSpeed(){
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(1, 20);
  display.println("Average Speed: ");
  display.setCursor(90, 20);
  display.println(averageSpeedKmph, 2);
}

void loop() {
  while (GPSSerial.available()) {
    char c = GPSSerial.read();
    if (gps.encode(c)) {
      if (gps.location.isUpdated() && gps.location.isValid()) {
        unsigned long currentMicros = micros();
        unsigned long deltaTimeMicros = currentMicros - previousMicros;
        previousMicros = currentMicros;

        // Calculate distance between consecutive points (Haversine formula)
        if (prevLat != 0.0 && prevLon != 0.0) {
          double lat1 = prevLat;
          double lon1 = prevLon;
          double lat2 = gps.location.lat();
          double lon2 = gps.location.lng();
          totalDistance += haversine(lat1, lon1, lat2, lon2);
        }

        // Update previous latitude and longitude
        prevLat = gps.location.lat();
        prevLon = gps.location.lng();

        // Accumulate total time elapsed
        totalTimeMicros += deltaTimeMicros;

        // Print other GPS data (latitude, longitude, speed, etc.)
        // ...

        // Calculate current speed in km/h
        currentSpeedKmph = gps.speed.kmph();
        Serial.print("Current Speed (km/h): ");
        Serial.println(currentSpeedKmph, 2); // Print with 2 decimal places

        // Calculate average speed
        double totalTimeHours = totalTimeMicros / 3.6e9; // Convert to hours
        averageSpeedKmph = (totalDistance / 1000.0) / totalTimeHours;
        Serial.print("Average Speed (km/h): ");
        Serial.println(averageSpeedKmph, 2); // Print with 2 decimal places

        display.clearDisplay();
        displayCurrentSpeed();
        displayAverageSpeed();
        display.display();

        Serial.print("\r\n\n");
      }
    }
  }
  delay(200); // Wait for 0.2 second
}