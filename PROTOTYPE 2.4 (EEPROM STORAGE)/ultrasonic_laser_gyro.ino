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
Adafruit_VL53L0X lox;  // Laser distance sensor
Adafruit_MPU6050 mpu;  // MPU6050 for accelerometer and gyroscope data

// Timing variables
unsigned long previousMillis = 0;
const unsigned long interval = 1000;  // Update interval for OLED in milliseconds

int currentEEPROMAddress = 0;  // Keeps track of the EEPROM write address

int timeIndication = 0;  // Counter for time indication

int previousLaser = 0;       // Stores previous laser distance
int previousUltrasonic = 0;  // Stores previous ultrasonic distance

// Define the SensorData object structure
struct SensorData {
  int timeIndication;      // Timestamp or loop iteration count
  int laser;               // Laser distance reading
  int ultrasonicDistance;  // Ultrasonic distance reading
  int RANGEultrasonic;     // Change in ultrasonic distance
  int RANGELaser;          // Change in laser distance
  float gyro;              // Gyroscope x-axis data
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
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);  // Set accelerometer range
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);       // Set gyroscope range
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);    // Set bandwidth

  // Initialize ultrasonic sensor pins
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  unsigned long currentMillis = millis();

  // --- VL53L0X Distance Measurement ---
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);  // Perform measurement
  int laser;
  if (measure.RangeStatus != 4 && measure.RangeMilliMeter != 8191)  // Phase failures have incorrect data
    laser = measure.RangeMilliMeter;
  else
    laser = 0;  // Error code for invalid measurement

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
  float gyro = g.gyro.x;  // Get x-axis gyroscope data

  // --- Update OLED Display ---
  if (currentMillis - previousMillis >= interval) {
    timeIndication++;

    // Create a SensorData object
    SensorData sensorData;
    sensorData.timeIndication = timeIndication;
    sensorData.laser = laser;
    sensorData.ultrasonicDistance = ultrasonicDistance;
    int RANGEultrasonic = ultrasonicDistance - previousUltrasonic;
    int RANGELaser = laser - previousLaser;
    sensorData.RANGEultrasonic = RANGEultrasonic;
    sensorData.RANGELaser = RANGELaser;
    sensorData.gyro = gyro;

    // Print all sensor data to Serial Monitor
    Serial.print(ultrasonicDistance);
    Serial.print(",");
    Serial.print(laser);
    Serial.print(",");
    Serial.print(RANGEultrasonic);
    Serial.print(",");
    Serial.print(RANGELaser);
    Serial.print(",");
    Serial.print(previousUltrasonic);
    Serial.print(",");
    Serial.print(previousLaser);
    Serial.print(",");
    Serial.println(gyro);

    // Display Laser distance on OLED
    display.setCursor(0, 0);
    display.print("Laser (mm): ");
    if (laser != 0)
      display.print(laser);
    else
      display.print("N/A");

    // Display Laser Range on OLED
    display.setCursor(0, 10);
    display.print("Range(L): ");
    display.print(RANGELaser);
    display.print(" mm");

    // Display Ultrasonic distance
    display.setCursor(0, 30);
    display.print("Ultrasonic: ");
    display.print(ultrasonicDistance);
    display.print(" cm");

    // Display Ultrasonic Range on OLED
    display.setCursor(0, 40);
    display.print("Range(U): ");
    display.print(RANGEultrasonic);
    display.print(" cm");

    // Display MPU6050 gyroscope data
    display.setCursor(0, 50);
    display.print("Gyro(X): ");
    display.print(gyro);

    // Update the display
    display.display();
    display.clearDisplay();
    saveDataToEEPROM(sensorData);  // Save data to EEPROM

    previousUltrasonic = ultrasonicDistance;  // Update previous values
    previousLaser = laser;
    previousMillis = currentMillis;  // Reset timing
  }
}

void saveDataToEEPROM(SensorData data) {
  // Retrieve the last written address from EEPROM

  int valueAtAddress0 = EEPROM.get(0, valueAtAddress0);

  EEPROM.put(valueAtAddress0, data);

  valueAtAddress0 += sizeof(SensorData);

  if (valueAtAddress0 > EEPROM.length()) {
    valueAtAddress0 = 2;
  }
  EEPROM.put(0, valueAtAddress0);
}

