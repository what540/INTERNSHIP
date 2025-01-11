#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>

// OLED display configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Pins for Ultrasonic sensor
const int triggerPin = 33;
const int echoPin = 32;

// Create sensor objects
Adafruit_VL53L0X lox;  // Laser distance sensor
Adafruit_MPU6050 mpu;  // MPU6050 for accelerometer and gyroscope data

// Timing variables
unsigned long previousMillis = 0;
const unsigned long interval = 1000;


unsigned long ReadingPreviousMillis = 0;  // Counter for time indication
const unsigned long ReadingInterval = 100;
const unsigned long numReadings = interval / ReadingInterval;  // Number of readings in one second

int currentIndex = 0;  // Current index to store the reading

double laser = 0;
double previousLaser = 0;           // Stores previous laser distance
double laserSum = 0;                // Stores previous laser distance
double laserReadings[numReadings];  // Array to store the readings

double ultrasonicDistance = 0;
double previousUltrasonic = 0;      // Stores previous ultrasonic distance
double ultrasonicDistanceSum = 0;   // Stores previous ultrasonic distance
double ultraReadings[numReadings];  // Array to store the readings

#include "FS.h"
#include <SD.h>
#include <SPI.h>

String dataMessage;
bool sdCardPresent = false;

// By fair electro...... For more videos... https://www.youtube.com/channel/UCsn5KGbvt5Se7MaLsfNYthA

#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
const long utcOffsetInSeconds = 8 * 3600;  // Adjust this based on your timezone.... 5 is for pakistan
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);

#include <Wire.h>
#include "RTClib.h"
RTC_DS3231 rtc;

#include <cmath>

/*const char *ssid = "ipad";
const char *password = "ily666777";*/

const char *ssid = "Linksys02145";
const char *password = "34pt8f7rxh";

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000);

  // Initialize OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 OLED allocation failed"));
    for (;;)
      ;
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setRotation(0);
  display.display();

  // Initialize VL53L0X sensor
  if (!lox.begin()) {
    Serial.println(F("Failed to initialize VL53L0X sensor"));
    while (1)
      ;
  }

  // Initialize MPU6050 sensor
  if (!mpu.begin(0x69)) {
    Serial.println(F("Failed to initialize MPU6050 sensor"));
    while (1)
      ;
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);  // Set accelerometer range
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);       // Set gyroscope range
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);    // Set bandwidth

  // Initialize ultrasonic sensor pins
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);

  WiFi.begin(ssid, password);
  delay(3000);
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }
  timeClient.begin();
  timeClient.update();
  if (WiFi.status() == WL_CONNECTED) {
    rtc.adjust(DateTime(timeClient.getEpochTime()));
  } else {
    Serial.println("Couldn't find Wifi");
  }
}

void loop() {
  unsigned long currentMillis = millis();

  // --- MPU6050 Accelerometer and Gyroscope Measurement ---
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float gyro = g.gyro.x;  // Get x-axis gyroscope data


  if (currentMillis - ReadingPreviousMillis >= ReadingInterval) {
    // --- VL53L0X Distance Measurement ---
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);                                 // Perform measurement
    if (measure.RangeStatus != 4 && measure.RangeMilliMeter != 8191)  // Phase failures have incorrect data
      laser = (double)measure.RangeMilliMeter - 6;
    else
      laser = 0;  // Error code for invalid measurement
    // Send a pulse to the ultrasonic sensor
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);

    // Measure the time for the pulse to return
    long duration = pulseIn(echoPin, HIGH);

    // Calculate the distance in cm
    ultrasonicDistance = duration * 0.0344 / 2;

    laserReadings[currentIndex] = laser;
    ultraReadings[currentIndex] = ultrasonicDistance;
    Serial.print("currentIndex: ");
    Serial.print(currentIndex);
    Serial.printf(" %.0f    ", laser);
    Serial.printf(" %.3f\n", ultrasonicDistance);
    currentIndex++;
    if (currentIndex >= numReadings) {
      if (currentMillis - previousMillis >= interval) {
        double UltraMaxValueInterval = ultraReadings[0];
        double UltraMinValueInterval = ultraReadings[0];
        double UltraRANGEinterval = 0;
        ultrasonicDistanceSum = 0;  // Reset sum for averaging

        double LaserMaxValueInterval = laserReadings[0];
        double LaserMinValueInterval = laserReadings[0];
        double LaserRANGEinterval = 0;
        laserSum = 0;

        for (int i = 0; i < numReadings; i++) {
          ultrasonicDistanceSum += ultraReadings[i];
          if (ultraReadings[i] > UltraMaxValueInterval) {
            UltraMaxValueInterval = ultraReadings[i];
          } else if (ultraReadings[i] < UltraMinValueInterval) {
            UltraMinValueInterval = ultraReadings[i];
          }

          laserSum += laserReadings[i];
          if (laserReadings[i] > LaserMaxValueInterval) {
            LaserMaxValueInterval = laserReadings[i];
          } else if (laserReadings[i] < LaserMinValueInterval) {
            LaserMinValueInterval = laserReadings[i];
          }
        }

        UltraRANGEinterval = UltraMaxValueInterval - UltraMinValueInterval;
        ultrasonicDistance = ultrasonicDistanceSum / numReadings;
        double diffUltraInOneSec = ultrasonicDistance - previousUltrasonic;

        LaserRANGEinterval = LaserMaxValueInterval - LaserMinValueInterval;
        laser = laserSum / numReadings;
        double diffLaserInOneSec = laser - previousLaser;

        double UltraSqDiff = 0;
        for (int i = 0; i < numReadings; i++) {
          UltraSqDiff += (ultraReadings[i] - ultrasonicDistance) * (ultraReadings[i] - ultrasonicDistance);
          ultraReadings[i] = 0;
        }
        UltraSqDiff = UltraSqDiff / numReadings;
        double UltraStdDev = sqrt(UltraSqDiff);

        int LaserSqDiff = 0;
        for (int i = 0; i < numReadings; i++) {
          LaserSqDiff += (laserReadings[i] - laser) * (laserReadings[i] - laser);
          laserReadings[i] = 0;
        }
        LaserSqDiff = LaserSqDiff / numReadings;
        double LaserStdDev = sqrt(LaserSqDiff);

        // Print all sensor data to Serial Monitor
        Serial.print(ultrasonicDistance, 4);
        Serial.print(",");
        Serial.print(laser);
        Serial.print(",");
        Serial.print(diffUltraInOneSec, 4);
        Serial.print(",");
        Serial.print(diffLaserInOneSec);
        Serial.print(",");
        Serial.print(previousUltrasonic);
        Serial.print(",");
        Serial.print(previousLaser);
        Serial.print(",");
        Serial.println(gyro);

        // Display on OLED
        display.setCursor(0, 0);
        display.print(printLocalTime());

        display.setCursor(0, 10);
        display.print("Laser (mm): ");
        if (laser != 0)
          display.print(laser, 2);
        else
          display.print("N/A");
        // Display Laser Range on OLED
        display.setCursor(0, 20);
        display.print("Range(L): ");
        display.print(diffLaserInOneSec, 2);
        display.print(" mm");

        // Display Ultrasonic distance
        display.setCursor(0, 30);
        display.print("Ultrasonic: ");
        display.print(ultrasonicDistance, 2);
        display.print(" cm");

        // Display Ultrasonic Range on OLED
        display.setCursor(0, 40);
        display.print("Range(U): ");
        display.print(diffUltraInOneSec, 2);
        display.print(" cm");

        // Display MPU6050 gyroscope data
        display.setCursor(0, 50);
        display.print("Gyro(X): ");
        display.print(gyro);

        // Update the display
        display.display();
        display.clearDisplay();


        Serial.printf("  Avg distance: %.4f", laser);
        Serial.printf("  Prev: %.4f", previousLaser);
        Serial.printf("  Difference: %.4f", diffLaserInOneSec);
        Serial.printf("  R4NGEin1s: %.4f", LaserRANGEinterval);
        Serial.printf("  var: %.4f", LaserSqDiff);
        Serial.printf("  stdDev: %.4f\n", LaserStdDev);
        Serial.printf("  Avg distance: %.4f", ultrasonicDistance);
        Serial.printf("  Prev: %.4f", previousUltrasonic);
        Serial.printf("  Difference: %.4f", diffUltraInOneSec);
        Serial.printf("  R4NGEin1s: %.4f", LaserRANGEinterval);
        Serial.printf("  var: %.4f", UltraSqDiff);
        Serial.printf("  stdDev: %.4f\n\n", UltraStdDev);

        dataMessage = padLeft(String(printLocalTime()), 19) + ", " + padLeft(String(ultrasonicDistance), 10) + ", " + padLeft(String(laser), 7) + ", " + padLeft(String(gyro), 7) + "\n";
        if (sdCardPresent) {
          Serial.println("SD card is available!");
          appendFile(SD, "/data.txt", dataMessage.c_str());
          // Perform SD card operations here
        } else {
          Serial.println("SD card is NOT available!");
        }

        checkSDCard();

        previousUltrasonic = ultrasonicDistance;  // Update previous values
        previousLaser = laser;
        previousMillis = currentMillis;  // Reset timing
      }
      currentIndex = 0;
    }
    ReadingPreviousMillis = currentMillis;
  }
}

String printLocalTime() {
  DateTime now = rtc.now();
  String timeString = String(now.year()) + "-" + (now.month() < 10 ? "0" : "") + String(now.month()) + "-" + (now.day() < 10 ? "0" : "") + String(now.day()) + " " + (now.hour() < 10 ? "0" : "") + String(now.hour()) + ":" + (now.minute() < 10 ? "0" : "") + String(now.minute()) + ":" + (now.second() < 10 ? "0" : "") + String(now.second());
  return timeString;  // Print the formatted time
}

void appendFile(fs::FS &fs, const char *path, const char *message) {
  Serial.printf("Appending to file: %s\n", path);
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void checkSDCard() {
  // If the SD card is already initialized, release it before reinitializing
  SD.end();

  // Try reinitializing the SD card
  if (!SD.begin()) {
    sdCardPresent = false;
    //Serial.println("Failed to initialize SD card.");
    return;
  }
  // Check card type
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    //Serial.println("No SD card attached.");
    sdCardPresent = false;
  } else {
    Serial.println("SD card detected.");
    sdCardPresent = true;
  }
}

String padLeft(String str, int width) {
  while (str.length() < width) {
    str = " " + str;  // Add a space to the left until the desired width is reached
  }
  return str;
}
