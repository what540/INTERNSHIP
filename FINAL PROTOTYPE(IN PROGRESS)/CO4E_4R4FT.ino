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
const int triggerPin = 32;
const int echoPin = 33;

float gyro = 0;

// Create sensor objects
Adafruit_VL53L0X lox;  // Laser distance sensor
Adafruit_MPU6050 mpu;  // MPU6050 for accelerometer and gyroscope data

// Timing variables
unsigned long previousMillis = 0;
const unsigned long interval = 1000;

unsigned long ReadingPreviousMillis = 0;  // Counter for time indication
const unsigned long ReadingInterval = 40;
const unsigned long numReadings = interval / ReadingInterval;  // Number of readings in one second

unsigned long previousMillisForRS485connection = 0;
const unsigned long intervalForRS485connection = 5000;

int currentIndex = 0;  // Current index to store the reading

double laser = 0;
double previousLaser = 0;           // Stores previous laser distance
double laserSum = 0;                // Stores previous laser distance
double laserReadings[numReadings];  // Array to store the readings

double ultrasonicDistance = 0;
double previousUltrasonic = 0;      // Stores previous ultrasonic distance
double ultrasonicDistanceSum = 0;   // Stores previous ultrasonic distance
double ultraReadings[numReadings];  // Array to store the readings

double UltraMaxValueInterval = 0;
double UltraMinValueInterval = 0;
double UltraRANGEinterval = 0;

double LaserMaxValueInterval = 0;
double LaserMinValueInterval = 0;
double LaserRANGEinterval = 0;

double diffUltraInOneSec = 0;
double diffLaserInOneSec = 0;

double UltraSqDiff = 0;
double UltraStdDev = 0;
double LaserSqDiff = 0;
double LaserStdDev = 0;

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

#define TX_PIN 17           // RS485 TX Pin
#define RX_PIN 16           // RS485 RX Pin
#define DE_RE_PIN 4         // RS485 Driver Enable/Receiver Enable Pin
char tx_buffer[5096] = "";  // Buffer for data to send
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

const char *ssid = "ipad";
const char *password = "ily666777";

unsigned long currentMillis = 0;

#define LED_pin 25
#define LED_pin1 26

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("Initializing");
  checkSDCard();
  WiFi.begin(ssid, password);
  delay(5000);
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
  } else {
    Serial.println("rtc ok");
  };
  timeClient.begin();
  timeClient.update();
  if (WiFi.status() == WL_CONNECTED) {
    rtc.adjust(DateTime(timeClient.getEpochTime()));
  } else {
    Serial.println("Couldn't find Wifi");
  }

  // Initialize OLED display
  /* if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 OLED allocation failed"));
    for (;;)
      ;
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setRotation(2);
  display.display();*/

  // Initialize VL53L0X sensor
  if (!lox.begin()) {
    Serial.println(F("Failed to initialize VL53L0X sensor"));
    //while (1)
    //  ;
  } else {
    Serial.println("laser ok");
  };
  // Initialize MPU6050 sensor
  if (!mpu.begin(0x69)) {
    Serial.println(F("Failed to initialize MPU6050 sensor"));
    //while (1)
    //;
  } else {
    Serial.println("mpu ok");
  };
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);  // Set accelerometer range
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);       // Set gyroscope range
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);    // Set bandwidth

  // Initialize ultrasonic sensor pins
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  //pinMode(RX_PIN, INPUT_PULLDOWN);  // Replace 16 with your RX pin number

  pinMode(DE_RE_PIN, OUTPUT);
  RS485_SetRX();  // Default to receive mode

  Serial.flush();
  Serial2.flush();

  pinMode(LED_pin, OUTPUT);
  pinMode(LED_pin1, OUTPUT);

  digitalWrite(LED_pin, LOW);
  digitalWrite(LED_pin1, LOW);

  analogReadResolution(12);       // Set ADC resolution to 12-bit (0-4095)
  analogSetAttenuation(ADC_0db);  // Set ADC range to 0-1.1V
}

bool measuring_status = false;
bool laser_pointers_status = false;

String measuring_command = "00";
String prev_measuring_command = "";

String laser_pointers_command = "";

String sdCardPresent_status = "No";

String time_for_all = "";

float battery = 0;

double getAverageVoltage(int pin, int ReadingsForBatteryVolt) {
  double sum = 0;
  for (int i = 0; i < ReadingsForBatteryVolt; i++) {
    int adcValue = analogRead(pin);                    // Read ADC value (0-4095)
    float voltage = ((float)adcValue / 4095.0) * 1.1;  // Convert ADC value to voltage
    sum += voltage;
  }
  double avgReading = sum / (double)ReadingsForBatteryVolt;
  return avgReading;
}
// Convert ADC reading to actual voltage
/*double convertToVoltage(double reading) {
  if (reading < 1 || reading >= 4095) return 0;  // Ignore extreme values
  return -0.000000000000016 * pow(reading, 4) + 0.000000000118171 * pow(reading, 3) - 0.000000301211691 * pow(reading, 2) + 0.001109019271794 * reading + 0.034143524634089;
}*/
// Convert voltage to battery percentage
int mapBatteryPercentage(double voltage) {
  int percent = (int)(((voltage - 0.6885245902) / (1.032786885 - 0.6885245902)) * 100);
  if (percent > 100) percent = 100;  // Clamp max to 100%
  if (percent < 0) percent = 0;      // Clamp min to 0%
  return percent;
}
double avgVoltage = 0;
int batteryPercent = 0;

String printCleanString(String data) {


  String cleaned = "";  // Store the cleaned string
  for (int i = 0; i < data.length(); i++) {
    if (isPrintable(data[i])) {  // Use custom function
      cleaned += data[i];        // Append only printable characters
    }
  }
  return cleaned;  // Return the cleaned string
}

void loop() {
  currentMillis = millis();
  //resistor 220k - ~215.7k
  //ressisotr 100k - ~99k

  //battery = (float)analogRead(36) / 4096 * 10.5 /*max bat voltage*/ * (100833 / 100000);
  //battery = (float) analogRead(36) / 4096 * 10.5 /*max bat voltage*/ * (100833/99000);
  //battery = (float) analogRead(36) / 4096 * 10.5 /*max bat voltage*/ * (98862.5);
  //int adcValue = analogRead(36);

  // Convert ADC value to voltage using 1.1V reference voltage
  /* battery = (adcValue / 4095.0) * 1.1;
  Serial.print("adc: ");
  Serial.println(adcValue);
  Serial.print("Battery voltage");
  Serial.println(battery);*/

  avgVoltage = getAverageVoltage(36, 100);  // Get averaged voltage from 100 readings
  batteryPercent = mapBatteryPercentage(avgVoltage);

  Serial.println(avgVoltage);

  if ((measuring_status)) {
    if (currentMillis - previousMillisForRS485connection > intervalForRS485connection) {
      measuring_command = "00";
    }
  }

  if (Serial2.available() > 3) {
    int availableBytes = Serial2.available();
    Serial.println(availableBytes);
    measuring_command = Serial2.readStringUntil(':');  //https://youtu.be/VdSFwYrYqW0?si=4_7ddYQORFcv9GpS
    measuring_command = printCleanString(measuring_command);

    String ultradist_from_python = Serial2.readStringUntil(';');
    String laserdist_from_python = Serial2.readStringUntil('\n');
    // laserdist_from_python = printCleanString(laserdist_from_python);

    measuring_command.trim();

    ultradist_from_python.trim();
    laserdist_from_python.trim();

    Serial.print(measuring_command);
    Serial.print(",");
    Serial.print(ultradist_from_python);
    Serial.print(",");
    Serial.println(laserdist_from_python);
    dataMessage = padLeft(time_for_all, 19) + ", " + padLeft(ultradist_from_python, 10) + ", " + padLeft(laserdist_from_python, 10) + ", " + padLeft(String(gyro), 7) + "\n";

    if (measuring_command == prev_measuring_command) {
      if (sdCardPresent) {
        //Serial.println("SD card is available!");
        appendFile(SD, "/data.txt", dataMessage.c_str());  //by Engr Fahad 21 Feb 2021 https://youtu.be/fPvW-dtB6i0?si=oah1L_oaQmai2ylV
      }
    }
    prev_measuring_command = measuring_command;

    previousMillisForRS485connection = millis();
  }

  if (measuring_command == "10" | measuring_command == "11") {
    measuring_status = true;
    if (measuring_command == "11") {
      laser_pointers_status = true;
    } else {
      laser_pointers_status = false;
    }
  } else if (measuring_command == "00" | measuring_command == "01") {
    measuring_status = false;
    if (measuring_command == "01") {
      laser_pointers_status = true;
    } else {
      laser_pointers_status = false;
    }
  } else {
    measuring_status = measuring_status;
    laser_pointers_status = laser_pointers_status;
  }

  if (laser_pointers_status) {
    digitalWrite(LED_pin, HIGH);
    digitalWrite(LED_pin1, HIGH);
  } else {
    digitalWrite(LED_pin, LOW);
    digitalWrite(LED_pin1, LOW);
  }

  if (measuring_status) {
    //  Serial.print("5t4te: ");
    // Serial.println(measuring_command);
    mpu.enableSleep(false);
    if (currentMillis - ReadingPreviousMillis >= ReadingInterval) {  // --- MPU6050 Accelerometer and Gyroscope Measurement ---
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      gyro = g.gyro.x;  // Get x-axis gyroscope data

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
      /*Serial.print("currentIndex: ");
    Serial.print(currentIndex);
    Serial.printf("        %.0f    ", laser);
    Serial.printf(" %.3f\n", ultrasonicDistance);*/
      currentIndex++;
      if (currentIndex >= numReadings) {
        //if (currentMillis - previousMillis >= interval) {
        String msg = String(numReadings) + ",";
        for (int i = 0; i < numReadings; i++) {
          msg += String(ultraReadings[i]) + ",";
          ultraReadings[i] = 0;
        }
        for (int i = 0; i < numReadings; i++) {
          msg += String(laserReadings[i]) + ",";
          laserReadings[i] = 0;
        }
        time_for_all = printLocalTime_timeOnly();
        msg += String(gyro) + "," + time_for_all + "," + sdCardPresent_status + "," + String(batteryPercent) + "\n";

        RS485_SetTX();
        Serial2.flush();
        snprintf(tx_buffer, sizeof(tx_buffer),
                 "%s\n", msg.c_str());
        RS485_Send(tx_buffer, strlen(tx_buffer));
        RS485_SetRX();  // Switch back to receive mode

        Serial.print(tx_buffer);

        time_for_all = printLocalTime() + " " + time_for_all;

        /*display.setCursor(0, 0);
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

          // Display Ultrasonic distance
          display.setCursor(0, 30);
          display.print("Ultra(mm): ");
          display.print(ultrasonicDistance, 4);

          // Display Ultrasonic Range on OLED
          display.setCursor(0, 40);
          display.print("Range(U): ");
          display.print(diffUltraInOneSec, 4);

          // Display MPU6050 gyroscope data
          display.setCursor(0, 50);
          display.print("Gyro(X): ");
          display.print(gyro);

          // Update the display
          display.display();
          display.clearDisplay();*/

        // previousUltrasonic = ultrasonicDistance;  // Update previous values
        // previousLaser = laser;
        //previousMillis = currentMillis;  // Reset timing
        // }
        currentIndex = 0;
        //}
        ReadingPreviousMillis = currentMillis;
      }
    } else {
      /* display.setCursor(0, 0);
    display.print(printLocalTime());

    display.setCursor(0, 10);
    display.print("Not measuring");
    display.setCursor(0, 20);
    if (sdCardPresent)
      display.print("SD FOUND");
    else if (!sdCardPresent)
      display.print("SD NOT FOUND, NOT SAVING");
    display.display();
    display.clearDisplay();*/

      checkSDCard();
      mpu.enableSleep(true);

      previousMillis = millis();
      ReadingPreviousMillis = millis();
      currentIndex = 0;
    }
  }
}

String printLocalTime() {
  DateTime now = rtc.now();
  String timeString = String(now.year()) + "-" + (now.month() < 10 ? "0" : "") + String(now.month()) + "-" + (now.day() < 10 ? "0" : "") + String(now.day());  //Chatgpt
  return timeString;                                                                                                                                           // Print the formatted time
}

String printLocalTime_timeOnly() {
  DateTime now = rtc.now();
  String timeString = (now.hour() < 10 ? "0" : "") + String(now.hour()) + ":" + (now.minute() < 10 ? "0" : "") + String(now.minute()) + ":" + (now.second() < 10 ? "0" : "") + String(now.second());  //Chatgpt
  return timeString;                                                                                                                                                                                  // Print the formatted time
}

//ESP32 SD_Test Example https://github.com/espressif/arduino-esp32/blob/master/libraries/SD/examples/SD_Test/SD_Test.ino
void appendFile(fs::FS &fs, const char *path, const char *message) {
  //Serial.printf("Appending to file: %s\n", path);
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    //Serial.println("Failed to open file for appending");
    sdCardPresent = false;
    sdCardPresent_status = "No";
    return;
  }
  if (file.print(message)) {
    //Serial.println("Message appended");
    sdCardPresent = true;
    sdCardPresent_status = "Yes";
  } else {
    //Serial.println("Append failed");
    sdCardPresent = false;
    sdCardPresent_status = "No";
  }
  file.close();
}

void checkSDCard() {
  SD.end();  //by Ralph S Bacon 15 Mar 2016 https://youtu.be/GQjtG1MeVs4?si=gjt1i7eOD1Qj9MW9

  // Try reinitializing the SD card
  if (!SD.begin()) {
    sdCardPresent = false;
    Serial.println("Failed to initialize SD card.");
    sdCardPresent_status = "No";
    return;
  }
  // Check card type
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached.");
    sdCardPresent = false;
    sdCardPresent_status = "No";
  } else {
    Serial.println("SD card detected.");
    sdCardPresent = true;
    sdCardPresent_status = "Yes";
  }
  return;
}

//Chatgpt
String padLeft(String str, int width) {
  while (str.length() < width) {
    str = " " + str;  // Add a space to the left until the desired width is reached
  }
  return str;
}
