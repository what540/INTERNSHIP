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

#include <WiFi.h>  //By fair electro 24 Aug 2023 https://youtu.be/FNOC0GFGcG0?si=x4cHJ9zDt9cXv5z3
#include <NTPClient.h>
#include <WiFiUdp.h>
const long utcOffsetInSeconds = 8 * 3600;  // Adjust this based on your timezone.... 5 is for pakistan
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);
#include <Wire.h>  //By fair electro 24 Aug 2023 https://youtu.be/FNOC0GFGcG0?si=x4cHJ9zDt9cXv5z3
#include "RTClib.h"
RTC_DS3231 rtc;

#include <cmath>


#define TX_PIN 17           // RS485 TX Pin
#define RX_PIN 16           // RS485 RX Pin
#define DE_RE_PIN 4         // RS485 Driver Enable/Receiver Enable Pin
char tx_buffer[1000] = "";  // Buffer for data to send
void RS485_SetTX() {
  digitalWrite(DE_RE_PIN, HIGH);  // Enable Driver for Transmission
}
void RS485_SetRX() {
  digitalWrite(DE_RE_PIN, LOW);  // Enable Receiver
}
void RS485_Send(const char *buf, uint16_t size) {
  RS485_SetTX();                        // Switch to transmit mode
  Serial2.write((uint8_t *)buf, size);  // Send data
  Serial2.flush();                      // Wait until data is sent
  RS485_SetRX();                        // Switch back to receive mode
}


/*const char *ssid = "";
const char *password = "";*/

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
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

  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  pinMode(DE_RE_PIN, OUTPUT);
  RS485_SetRX();  // Default to receive mode

  Serial.flush();
  Serial2.flush();
}

void loop() {
  unsigned long currentMillis = millis();

  // --- MPU6050 Accelerometer and Gyroscope Measurement ---
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float gyro = g.gyro.x;  // Get x-axis gyroscope data

  if (currentMillis - ReadingPreviousMillis >= ReadingInterval) {
    Serial.flush();

    // --- VL53L0X Distance Measurement ---
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);                                 // Perform measurement
    if (measure.RangeStatus != 4 && measure.RangeMilliMeter != 8191)  // Phase failures have incorrect data
      laser = (double)measure.RangeMilliMeter - 10;
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
    ultrasonicDistance = 10 * duration * 0.0344 / 2;

    laserReadings[currentIndex] = laser;
    ultraReadings[currentIndex] = ultrasonicDistance;

    currentIndex++;
    if (currentIndex >= numReadings) {
      if (currentMillis - previousMillis >= interval) {
        Serial.flush();
        double UltraMaxValueInterval = ultraReadings[0];
        double UltraMinValueInterval = ultraReadings[0];
        double UltraRANGEinterval = 0;
        ultrasonicDistanceSum = 0;  // Reset sum for averaging

        double LaserMaxValueInterval = laserReadings[0];
        double LaserMinValueInterval = laserReadings[0];
        double LaserRANGEinterval = 0;
        laserSum = 0;

        //by geeksforgeeks 28 Dec, 2024 https://www.geeksforgeeks.org/mathematics-mean-variance-and-standard-deviation/
        //by Roel Van de Paar 23 Oct 2021 https://www.youtube.com/watch?v=NzWNJsG5EP8&t=38s&ab_channel=RoelVandePaar
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

        dataMessage = padLeft(String(printLocalTime()), 19) + ", " + padLeft(String(ultrasonicDistance, 2), 10) + ", " + padLeft(String(laser, 2), 10) + ", " + padLeft(String(gyro), 7) + "\n";
        if (sdCardPresent) {
          //Serial.println("SD card is available!");
          appendFile(SD, "/data.txt", dataMessage.c_str());  //by Engr Fahad 21 Feb 2021 https://youtu.be/fPvW-dtB6i0?si=oah1L_oaQmai2ylV
          // Perform SD card operations here
        } else {
          //Serial.println("SD card is NOT available!");
        }

        //https://youtu.be/kXjPUu9jZOw?si=3qPoobB1JeInpWOU
        //https://how2electronics.com/rs-485-half-duplex-communication-with-max485-arduino/
        snprintf(tx_buffer, sizeof(tx_buffer),
                 "%.4f,%.2f,%.4f,%.2f,%.4f,%.2f,%.2f\n", ultrasonicDistance, laser, diffUltraInOneSec, diffLaserInOneSec, previousUltrasonic, previousLaser, gyro);
        RS485_Send(tx_buffer, strlen(tx_buffer));  // Transmit the data

        // Print all sensor data to Serial Monitor
        Serial.print(ultrasonicDistance, 4);  //to Python by Paul McWhorter 25 Jan 2022 https://www.youtube.com/watch?v=VN3HJm3spRE&ab_channel=PaulMcWhorter
        Serial.print(",");
        Serial.print(laser);
        Serial.print(",");
        Serial.print(diffUltraInOneSec, 4);
        Serial.print(",");
        Serial.print(diffLaserInOneSec);
        Serial.print(",");
        Serial.print(previousUltrasonic, 4);
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
  String timeString = String(now.year()) + "-" + (now.month() < 10 ? "0" : "") + String(now.month()) + "-" + (now.day() < 10 ? "0" : "") + String(now.day()) + " " + (now.hour() < 10 ? "0" : "") + String(now.hour()) + ":" + (now.minute() < 10 ? "0" : "") + String(now.minute()) + ":" + (now.second() < 10 ? "0" : "") + String(now.second());  //Chatgpt
  return timeString;                                                                                                                                                                                                                                                                                                                                 // Print the formatted time
}

//ESP32 SD_Test Example https://github.com/espressif/arduino-esp32/blob/master/libraries/SD/examples/SD_Test/SD_Test.ino
void appendFile(fs::FS &fs, const char *path, const char *message) {
  //Serial.printf("Appending to file: %s\n", path);
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    //Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    //Serial.println("Message appended");
  } else {
    //Serial.println("Append failed");
  }
  file.close();
}

void checkSDCard() {
  SD.end();  //by Ralph S Bacon 15 Mar 2016 https://youtu.be/GQjtG1MeVs4?si=gjt1i7eOD1Qj9MW9

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
    //Serial.println("SD card detected.");
    sdCardPresent = true;
  }
}

//Chatgpt
String padLeft(String str, int width) {
  while (str.length() < width) {
    str = " " + str;  // Add a space to the left until the desired width is reached
  }
  return str;
}
