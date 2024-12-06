#include <EEPROM.h>

// Define the starting address for reading
int currentEEPROMAddress = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Optionally, you can save data to EEPROM here (if you want to test writing and then reading)
  // saveDataToEEPROM(150, 200, 23.45);

  // Retrieve data from EEPROM
  getDataFromEEPROM();

  for (int i = 0; i < EEPROM.length(); i++) {
    EEPROM.write(i, 0);  // Clear all EEPROM data by writing 0 to each address
  }

  // Check if everything is cleared
  bool isCleared = true;
  for (int i = 0; i < EEPROM.length(); i++) {
    if (EEPROM.read(i) != 0) {
      isCleared = false;  // If any byte is not zero, EEPROM is not fully cleared
      break;
    }
  }

  // Output the result to Serial Monitor
  if (isCleared) {
    Serial.println("EEPROM is cleared!");
  } else {
    Serial.println("EEPROM is not cleared!");
  }
}

void loop() {
  // Nothing to do in loop for now
}

// Function to save data to EEPROM as a string


// Function to retrieve and parse data from EEPROM
void getDataFromEEPROM() {
  // Initialize variables
  int count = 0;
  String dataString = "";
  char character;
  int currentEEPROMAddress = 0;  // Start at the beginning of EEPROM

  // Read data from EEPROM until the end
  while (currentEEPROMAddress < EEPROM.length()) {
    dataString = "";  // Reset the string for each dataset

    // Read characters until '\n' (newline), '\0', or end of EEPROM
    while (currentEEPROMAddress < EEPROM.length()) {
      character = EEPROM.read(currentEEPROMAddress++);
      if (character == '\n' || character == '\0') {
        break;  // End of one dataset
      }
      dataString += character;  // Append character to string
    }

    // If the dataset is non-empty, process it
    if (dataString.length() > 0) {
      Serial.print("Retrieved Data from EEPROM: ");
      Serial.println(dataString);

      // Parse the string into individual values (int, int, float)
      int laserVal, ultrasonicVal;
      float gyroVal;

      // Find the commas and extract values
      int firstCommaIndex = dataString.indexOf(',');
      int secondCommaIndex = dataString.indexOf(',', firstCommaIndex + 1);

      if (firstCommaIndex > -1 && secondCommaIndex > -1) {
        laserVal = dataString.substring(0, firstCommaIndex).toInt();
        ultrasonicVal = dataString.substring(firstCommaIndex + 1, secondCommaIndex).toInt();
        gyroVal = dataString.substring(secondCommaIndex + 1).toFloat();

        // Print the parsed values
        Serial.print("  Laser: ");
        Serial.print(laserVal);
        Serial.print("  Ultrasonic: ");
        Serial.print(ultrasonicVal);
        Serial.print("  Gyro: ");
        Serial.println(gyroVal);
      } else {
        Serial.println("Error: Malformed data string.");
      }
    }
    count++;
  }
  Serial.print("Total sets of data stored: ");
  Serial.println(count);
}
