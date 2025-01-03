#include <EEPROM.h>

// Define the starting address for reading data from EEPROM
int currentEEPROMAddress = 0;

void setup() {
  // Initialize serial communication for debugging and data output
  Serial.begin(115200);
  delay(2000);  // Wait for 2 seconds to ensure serial monitor is ready

  // Retrieve and display sensor data stored in EEPROM
  getDataFromEEPROM();

  // Clear all EEPROM data after retrieval if required
  clearEEPROM();
}

// Function to retrieve and parse data from EEPROM
void getDataFromEEPROM() {
  // Define a structure to hold sensor data
  struct SensorData {
    int timeIndication;      // Time elapsed since the Arduino started
    int laser;               // Laser sensor reading (in mm)
    int ultrasonicDistance;  // Ultrasonic sensor distance reading (in cm)
    int RANGEultrasonic;     // Range of ultrasonic sensor (in cm)
    int RANGELaser;          // Range of laser sensor (in mm)
    float gyro;              // Gyroscope sensor reading
  };

  // Determine the size of the SensorData structure
  int entrySize = sizeof(SensorData);
  int numEntries = EEPROM.length() / entrySize;  // Calculate how many entries can fit in EEPROM
  Serial.println("\n\n\nRetrieving Sensor Data from EEPROM:");
  // Loop through the EEPROM to retrieve and print each entry
  for (int i = 2; i < EEPROM.length(); i += entrySize) {
    // Create a SensorData instance to store retrieved data
    SensorData data;

    // Read data from the EEPROM at the current address
    EEPROM.get(i, data);

    // Check if the data is valid (non-zero time indication implies valid data)
    if (data.timeIndication != 0) {
      Serial.println("Address | Time(s)         | Laser(mm)       | L.Range(mm)   | Ultrasonic(cm) | U.Range(cm)   | Gyro    ");
      // Print the data with manual alignment
      Serial.print(padLeft(String(i), 7));  // Address column
      Serial.print(" | ");
      Serial.print(padLeft(String(data.timeIndication), 15));  // Time column
      Serial.print(" | ");
      Serial.print(padLeft(String(data.laser), 15));  // Laser column with 2 decimal places
      Serial.print(" | ");
      Serial.print(padLeft(String(data.RANGELaser), 13));  // L.Range column with 2 decimal places
      Serial.print(" | ");
      Serial.print(padLeft(String(data.ultrasonicDistance), 14));  // Ultrasonic column with 2 decimal places
      Serial.print(" | ");
      Serial.print(padLeft(String(data.RANGEultrasonic), 13));  // U.Range column with 2 decimal places
      Serial.print(" | ");
      Serial.println(padLeft(String(data.gyro), 8));  // Gyro column with 2 decimal places
    }
  }

  Serial.println("Retrieval DONE, clearing EEPROM now... ...");
}

// Function to clear all EEPROM data
void clearEEPROM() {
  // Loop through all EEPROM addresses and write 0 to each
  for (int i = 0; i < EEPROM.length(); i++) {
    EEPROM.write(i, 0);  // Write 0 to clear the data
  }
  Serial.println("EEPROM is cleared!\n");
  EEPROM.write(0, 2);
  int valueAtAddress0 = EEPROM.get(0, valueAtAddress0);
  // Print the value to the serial monitor
  Serial.print("Value at EEPROM address 0: ");
  Serial.println(valueAtAddress0);
}


// Helper function to pad a string with spaces on the left for alignment
String padLeft(String str, int width) {
  while (str.length() < width) {
    str = " " + str;  // Add a space to the left until the desired width is reached
  }
  return str;
}

void loop() {
  // The loop function is intentionally left empty
  // This sketch performs all actions in the setup function
}
