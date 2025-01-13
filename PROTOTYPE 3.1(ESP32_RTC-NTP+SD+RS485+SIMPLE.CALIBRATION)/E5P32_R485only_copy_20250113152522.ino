#define TX_PIN 17           // RS485 TX2 Pin
#define RX_PIN 16           // RS485 RX2 Pin
#define DE_RE_PIN 4         // RS485 Driver Enable/Receiver Enable Pin
char tx_buffer[1000] = "";  // Buffer for data to send

void RS485_SetTX() {
  digitalWrite(DE_RE_PIN, HIGH);
  delay(1);  // Short delay for stability
}

void RS485_SetRX() {
  digitalWrite(DE_RE_PIN, LOW);
  delay(1);  // Short delay for stability
}

void RS485_Send(const char *buf, uint16_t size) {
  RS485_SetTX();                        // Switch to transmit mode
  Serial2.write((uint8_t *)buf, size);  // Send data
  Serial2.flush();                      // Wait until data is sent
  RS485_SetRX();                        // Switch back to receive mode
}

void setup() {
  Serial.begin(115200);  // Debugging serial monitor
  delay(1000);
  Serial.println("Setup started");

  // Initialize Serial2 for RS485 communication
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  pinMode(DE_RE_PIN, OUTPUT);
  RS485_SetRX();  // Default to receive mode
  Serial.println("Setup completed");
}

double ultrasonicDistance = 800;
double laser = 1000;
double diffUltraInOneSec = 100;
double diffLaserInOneSec = 100;
double previousUltrasonic = 799;
double previousLaser = 999;
double gyro = 0.19;

void loop() {
  snprintf(tx_buffer, sizeof(tx_buffer),
           "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f\n",
           ultrasonicDistance, laser, diffUltraInOneSec, diffLaserInOneSec, previousUltrasonic, previousLaser, gyro);
  RS485_Send(tx_buffer, strlen(tx_buffer));  // Transmit the data
  Serial.println(tx_buffer);              // Print the received data up to the newline

  delay(1000);  // Loop delay
}
