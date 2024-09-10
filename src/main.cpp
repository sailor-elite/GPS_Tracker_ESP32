#include <TinyGPS++.h>
#include <cmath>
#include <SPI.h>
#include <FS.h>
#include <SPIFFS.h>
#include <TFT_eSPI.h>

#define GPSTX 17              // Pin number for TX output from ESP32 - RX into GPS
#define GPSRX 16              // Pin number for RX input into ESP32 - TX from GPS
#define GPSserial Serial2     // Define GPSserial as ESP32 Serial2 

#define LED 21                // Pin for LED indicator
#define TIMER_INTERVAL 2000000 // Timer interval in microseconds (2 seconds)

#define TFT_MISO 19
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS   15  // Chip select control pin
#define TFT_DC    2  // Data Command control pin
#define TFT_RST   4  // Reset pin (could connect to RST pin)

TinyGPSPlus gps;              // Create an instance of TinyGPSPlus
TFT_eSPI tft = TFT_eSPI();    // Create an instance of the TFT library

double totalDistance = 0;
double lastLatitude = 0;
double lastLongitude = 0;
double smoothLatitude = 0;
double smoothLongitude = 0;
const double minDistance = 2.5;
const double SMOOTHING_FACTOR = 0.2;
int measurementFlagcounter = 0;

hw_timer_t *My_timer = NULL;
volatile bool measurementFlag = false;

void IRAM_ATTR onTimer() {
  measurementFlag = true;
}

void setup() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  // Initialize Serial and GPS
  GPSserial.begin(9600, SERIAL_8N1, GPSRX, GPSTX);
  Serial.begin(115200);
  Serial.println("GPS Decoder Starting");

  // Initialize Timer for GPS measurements
  My_timer = timerBegin(0, 160, true); // Timer 0, prescaler 160
  timerAttachInterrupt(My_timer, &onTimer, true);
  timerAlarmWrite(My_timer, TIMER_INTERVAL, true);
  timerAlarmEnable(My_timer);

  // Initialize TFT display
  tft.init();
  tft.setRotation(1); // Adjust rotation as needed
  tft.fillScreen(TFT_BLACK); // Clear the screen with black color
  tft.setTextColor(TFT_WHITE, TFT_BLACK); // Set text color to white with black background
  tft.setTextSize(2); // Set text size
}

int dayOfWeek(int year, int month, int day) {
  if (month < 3) {
    month += 12;
    year--;
  }
  return (day + 2*month + (3*(month+1))/5 + year + year/4 - year/100 + year/400) % 7;
}

// Function to determine if Daylight Saving Time is in effect
bool isDST(int month, int day, int year, int hour) {
  // DST starts from the last Sunday of March to the last Sunday of October
  if (month < 3 || month > 10) return false;  // Outside of DST months
  if (month > 3 && month < 10) return true;   // Completely within DST months

  int lastSunday = day - (dayOfWeek(year, month, day) % 7); // Find last Sunday of the month

  if (month == 3) {
    // DST starts on the last Sunday of March
    return (day >= lastSunday && hour >= 2);  // DST starts at 2:00 AM
  } else if (month == 10) {
    // DST ends on the last Sunday of October
    return (day < lastSunday || (day == lastSunday && hour < 2));  // DST ends at 2:00 AM
  }
  return false;
}

void loop() {
  while (GPSserial.available() > 0) {
    char c = GPSserial.read();
    Serial.write(c);           // Echo the GPS data to the Serial Monitor

    // Feed the GPS data to the TinyGPSPlus parser
    gps.encode(c);

    // If new location data is available
    if (gps.location.isUpdated()) {
      double currentLatitude = gps.location.lat();
      double currentLongitude = gps.location.lng();

      if (smoothLatitude == 0 && smoothLongitude == 0) {
        smoothLatitude = currentLatitude;
        smoothLongitude = currentLongitude;
      } else {
        smoothLatitude = smoothLatitude * (1 - SMOOTHING_FACTOR) + currentLatitude * SMOOTHING_FACTOR;
        smoothLongitude = smoothLongitude * (1 - SMOOTHING_FACTOR) + currentLongitude * SMOOTHING_FACTOR;
      }

      Serial.print("Smooth Latitude: ");
      Serial.println(smoothLatitude, 6);
      Serial.print("Smooth Longitude: ");
      Serial.println(smoothLongitude, 6);

      if (measurementFlag) {
        measurementFlag = false;

        if (lastLatitude != 0 || lastLongitude != 0) {
          double distance = TinyGPSPlus::distanceBetween(
            lastLatitude, lastLongitude,
            smoothLatitude, smoothLongitude);

          if (distance > minDistance) {
            totalDistance += fabs(distance);
            Serial.print("Current Distance: ");
            Serial.print(distance, 2);
            Serial.println(" meters");

            lastLatitude = smoothLatitude;
            lastLongitude = smoothLongitude;
          }
        } else {
          lastLatitude = smoothLatitude;
          lastLongitude = smoothLongitude;
        }

        Serial.print("Total Distance Traveled: ");
        Serial.print(totalDistance, 2);
        Serial.println(" meters");
        measurementFlagcounter++;
      }
    }

    // Display time on the TFT screen
    if (gps.time.isUpdated()) {
      int hour = gps.time.hour();
      int minute = gps.time.minute();
      int second = gps.time.second();

      // Adjust time for Polish time zone (UTC+1 or UTC+2 for DST)
      int adjustedHour = (hour + 2) % 24; // Add 2 hours for UTC+2
      if (isDST(gps.date.month(), gps.date.day(), gps.date.year(), hour)) {
        adjustedHour = (hour + 2) % 24; // DST (Daylight Saving Time)
      } else {
        adjustedHour = (hour + 1) % 24; // Standard Time
      }

      char timeBuffer[10];
      snprintf(timeBuffer, sizeof(timeBuffer), "%02d:%02d:%02d", adjustedHour, minute, second);

      tft.fillRect(0, 0, 240, 30, TFT_BLACK); // Clear the previous time
      tft.setCursor(0, 0);
      tft.print("Time: ");
      tft.println(timeBuffer);
      tft.setCursor(0, 25);
      tft.print("LAT: ");
      tft.println(smoothLatitude);
      tft.setCursor(0, 50);
      tft.print("LON: ");
      tft.println(smoothLongitude);
      tft.setCursor(0, 75);
      tft.print("Distance: ");
      tft.println(totalDistance);
      tft.setCursor(0, 100);
      tft.print("Measurement Count: ");
      tft.println(measurementFlagcounter);
      tft.setCursor(0, 125);
      tft.print("Satellites: ");
      tft.println(gps.satellites.value());
      Serial.print("Time: ");
      Serial.println(timeBuffer);
    }
  }

}
