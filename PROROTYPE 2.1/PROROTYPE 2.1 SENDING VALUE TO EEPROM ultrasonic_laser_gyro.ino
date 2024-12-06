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
const int triggerPin = 9;
const int echoPin = 10;

// Create sensor objects
Adafruit_VL53L0X lox;
Adafruit_MPU6050 mpu;

// Timing variables
unsigned long previousMillis = 0;
const unsigned long interval = 1000;  // Update interval for OLED

int currentEEPROMAddress = 0;


void setup() {
  // Initialize serial communication
  Serial.begin(115200);

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
    laser =9999;

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
  float temperature = temp.temperature;

  // --- Print all sensor data to Serial Monitor ---

  // --- Update OLED Display ---
  if (currentMillis - previousMillis >= interval) {

    Serial.print(laser);
    Serial.print(",");
    Serial.print(ultrasonicDistance);
    Serial.print(",");
    Serial.println(temperature);


    display.setCursor(0, 0);
    display.print("Laser (mm): ");
    if (laser != 9999)
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
    display.print("Temp: ");
    display.print(temperature);
    display.print(" C");

    // Update the display
    display.display();
    display.clearDisplay();

    saveDataToEEPROM(laser, ultrasonicDistance, temperature);

    previousMillis = currentMillis;
  }
}


void saveDataToEEPROM(int laserDistance, int distanceUltrasonic, float gyroReading) {
  // Create a formatted string: "int1,int2,float\n"
  String dataString = String(laserDistance) + "," + String(distanceUltrasonic) + "," + String(gyroReading, 2) + "\n"; // Add '\n' for new line

  // Write the string to EEPROM character by character
  for (int i = 0; i < dataString.length(); i++) {
    if (currentEEPROMAddress >= EEPROM.length()) {
      Serial.println("Error: EEPROM capacity exceeded.");
      return; // Stop writing to avoid overflow
    }

    EEPROM.write(currentEEPROMAddress, dataString[i]);
    currentEEPROMAddress++;  // Increment address for each character
  }

  // Optionally, add a null terminator (for safety during retrieval)
  if (currentEEPROMAddress < EEPROM.length()) {
    EEPROM.write(currentEEPROMAddress, '\0');
  }
}

