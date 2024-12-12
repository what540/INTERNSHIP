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


    Serial.print("Address ");
    char Addressbuffer[5];
    dtostrf(i, 5, 0, Addressbuffer);
    Serial.print(Addressbuffer);
    Serial.print("    |   Time: ");
    char timeIndicationsbuffer[7];
    dtostrf(data.timeIndication, 7, 0, timeIndicationsbuffer);
    Serial.print(timeIndicationsbuffer);
    Serial.print("s  ");
    Serial.print("    |  Laser =   ");
    char laserbuffer[4];
    dtostrf(data.laser, 4, 0, laserbuffer);
    Serial.print(laserbuffer);
    Serial.print("mm  ");
    Serial.print("    |   Ultrasonic =  ");
    char ultrasonicDistancebuffer[4];
    dtostrf(data.ultrasonicDistance, 4, 0, ultrasonicDistancebuffer);
    Serial.print(ultrasonicDistancebuffer);
    Serial.print("cm  ");
    Serial.print("    |   Gyro =   ");
    Serial.println(data.gyro, 2);
  }
}

void loop() {
  // Nothing to do in loop for now
}
4
