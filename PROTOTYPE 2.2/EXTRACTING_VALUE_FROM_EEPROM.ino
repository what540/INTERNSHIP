#include <EEPROM.h>

// Define the starting address for reading
int currentEEPROMAddress = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(2000);

  // Retrieve and parse data from EEPROM
  getDataFromEEPROM();

  Serial.println("Retrieval DONE, clearing EEPROM now... ...");

  // Clear EEPROM after reading if required
  clearEEPROM();
  Serial.println("EEPROM is cleared!");
}

// Function to retrieve and parse data from EEPROM
void getDataFromEEPROM() {
  // Define the SensorData structure
  struct SensorData {
    int timeIndication;
    int laser;
    int ultrasonicDistance;
    float gyro;
  };

  // Determine the size of SensorData
  int entrySize = sizeof(SensorData);
  int numEntries = EEPROM.length() / entrySize;

  Serial.println("\n\n\nRetrieving Sensor Data from EEPROM:");

  for (int i = 0; i < EEPROM.length(); i += entrySize) {
    // Retrieve the data from EEPROM
    SensorData data;
    EEPROM.get(i, data);

    // Check if the data is valid (non-zero)
    if (data.timeIndication != 0) {
      Serial.println("Address | Time (s)  | Laser (mm) | Ultrasonic (cm) | Gyro");
      // Print the data with manual alignment
      Serial.print(padLeft(String(i), 7));  // Address column
      Serial.print(" | ");
      Serial.print(padLeft(String(data.timeIndication), 9));  // Time column
      Serial.print(" | ");
      Serial.print(padLeft(String(data.laser), 10));  // Laser column
      Serial.print(" | ");
      Serial.print(padLeft(String(data.ultrasonicDistance), 15));  // Ultrasonic column
      Serial.print(" | ");
      Serial.println(String(data.gyro, 2));  // Gyro column with 2 decimal places
    }
  }
}

// Function to clear EEPROM
void clearEEPROM() {
  for (int i = 0; i < EEPROM.length(); i++) {
    EEPROM.write(i, 0);  // Clear all EEPROM data by writing 0 to each address
  }
}

String padLeft(String str, int width) {
  while (str.length() < width) {
    str = " " + str;
  }
  return str;
}


void loop() {
  // Nothing to do in loop for now
}
