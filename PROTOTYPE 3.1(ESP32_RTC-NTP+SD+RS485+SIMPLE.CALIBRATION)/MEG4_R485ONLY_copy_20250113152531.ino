// Use Serial1 on the Mega for RS485 communication
#define DE 3
#define RE 2

void setup() {
  Serial.begin(115200);   // Monitor
  Serial3.begin(115200);  // RS485 communication
  Serial.flush();
  Serial3.flush();
  delay(1000);
  pinMode(DE, OUTPUT);
  pinMode(RE, OUTPUT);
  // Set default to receive mode
  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);
}

void loop() {
  if (Serial3.available()) {
    String receivedData = Serial3.readStringUntil('\n');  // Read until a newline is encountered
    Serial.println(receivedData);                         // Print the received data up to the newline
  }
}
