#include <EEPROM.h>

// Define the starting address for reading
int currentEEPROMAddress = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(5000);

  // Retrieve and parse data from EEPROM
  getDataFromEEPROM();

  Serial.println("DONE Retrieve data from EEPROM");

  // Clear EEPROM after reading
  for (int i = 0; i < EEPROM.length(); i++) {
    EEPROM.write(i, 0);  // Clear all EEPROM data by writing 0 to each address
  }

  Serial.println("EEPROM is cleared!");
}

void loop() {
  // Nothing to do in loop for now
}

// Function to retrieve and parse data from EEPROM
void getDataFromEEPROM() {
  // Define the SensorData structure
  struct SensorData {
    int laser;
    int ultrasonicDistance;
    float gyro;
  };

  // Determine the size of SensorData
  int entrySize = sizeof(SensorData);
  int numEntries = EEPROM.length() / entrySize;

  int count = 1;

  Serial.println("\n\n\nRetrieving Sensor Data from EEPROM:");

  for (int i = 0; i < EEPROM.length(); i += entrySize) {

    // Retrieve the data from EEPROM
    SensorData data;
    EEPROM.get(i, data);

      
      Serial.print("Address ");
      Serial.print(i);
      Serial.print(": Laser = ");
      Serial.print(data.laser);
      Serial.print(", Ultrasonic = ");
      Serial.print(data.ultrasonicDistance);
      Serial.print(", Gyro = ");
      Serial.println(data.gyro, 2);  // Print gyro with 2 decimal places
      
      count++;  // Increment count for each valid entry

  }
}


