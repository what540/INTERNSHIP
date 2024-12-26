#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

// OLED display configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Pins for Ultrasonic sensor
const int triggerPin = 39;
const int echoPin = 41;

// Create sensor objects
Adafruit_VL53L0X lox;
Adafruit_MPU6050 mpu;

// Timing variables
unsigned long previousMillis = 0;
const unsigned long interval = 1000;  // Update interval for OLED

int currentEEPROMAddress = 0;

int timeIndication = 0;


// Define the SensorData object structure
struct SensorData {
  int timeIndication;
  int laser;
  int ultrasonicDistance;
  float gyro;
};

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
  display.display();

  // Initialize VL53L0X sensor
  if (!lox.begin()) {
    Serial.println(F("Failed to initialize VL53L0X sensor"));
    while (1)
      ;
  }

  // Initialize MPU6050 sensor
  if (!mpu.begin()) {
    Serial.println(F("Failed to initialize MPU6050 sensor"));
    while (1)
      ;
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Initialize ultrasonic sensor pins
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.println(F("Setup complete."));
}

void loop() {
  unsigned long currentMillis = millis();

  // --- VL53L0X Distance Measurement ---
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);  // Perform measurement
  int laser;
  if (measure.RangeStatus != 4)  // phase failures have incorrect data
    laser = measure.RangeMilliMeter;
  else
    laser = 8191;

  // --- HC-SR04 Ultrasonic Sensor Measurement ---
  long duration, ultrasonicDistance;

  // Send a pulse to the ultrasonic sensor
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  // Measure the time for the pulse to return
  duration = pulseIn(echoPin, HIGH);

  // Calculate the distance in cm
  ultrasonicDistance = duration * 0.0344 / 2;

  // --- MPU6050 Accelerometer and Gyroscope Measurement ---
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float gyro = g.gyro.x;



  // --- Update OLED Display ---
  if (currentMillis - previousMillis >= interval) {
    timeIndication++;


    // Create a SensorData object
    SensorData sensorData;
    sensorData.timeIndication = timeIndication;
    sensorData.laser = laser;
    sensorData.ultrasonicDistance = ultrasonicDistance;
    sensorData.gyro = gyro;

    // Print all sensor data to Serial Monitor
    Serial.print(laser);
    Serial.print(",");
    Serial.print(ultrasonicDistance);
    Serial.print(",");
    Serial.println(gyro);

    display.setCursor(0, 0);
    display.print("Laser (mm): ");
    if (laser != 8191)
      display.print(laser);
    else
      display.print("Out of Range");

    // Display Ultrasonic distance
    display.setCursor(0, 20);
    display.print("Ultrasonic: ");
    display.print(ultrasonicDistance);
    display.print(" cm");

    // Display MPU6050 gyroscope data
    display.setCursor(0, 40);
    display.print("Gyro (X): ");
    display.print(gyro);

    // Update the display
    display.display();
    display.clearDisplay();
    saveDataToEEPROM(sensorData);

    previousMillis = currentMillis;
  }
}

void saveDataToEEPROM(SensorData data) {
  // Loop through EEPROM to find the next available address
  for (int i = currentEEPROMAddress; i < EEPROM.length(); i += sizeof(SensorData)) {
    SensorData existingData;
    EEPROM.get(i, existingData);  // Read the existing data from EEPROM

    // Check if the EEPROM address is "empty", otherwise skip to next address
    if (existingData.timeIndication == 0 && existingData.laser == 0 && existingData.ultrasonicDistance == 0 && existingData.gyro == 0) {
      // Found an empty slot, write the new data here
      EEPROM.put(i, data);
      currentEEPROMAddress = i + sizeof(SensorData);  // Update to the next slot
      if (currentEEPROMAddress >= EEPROM.length()) {
        currentEEPROMAddress = 0;  // Reset address to start over if we reach the end
      }
      break;  // Exit the loop once the data is saved
    }
  }
}
