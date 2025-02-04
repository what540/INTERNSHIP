#define DE 46                             // Define the DE pin for RS485
#define RE 44                             // Define the DE pin for RS485
unsigned long ReadingPreviousMillis = 0;  // Counter for time indication
const unsigned long ReadingInterval = 1000;
unsigned long currentMillis = 0;
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);

  delay(100);  // wait a moment to allow serial ports to initialize
  pinMode(DE, OUTPUT);
  pinMode(RE, OUTPUT);

  digitalWrite(DE, LOW);  // Enable transmission
  digitalWrite(RE, LOW);  // Enable transmission    t65
}

void loop() {
  if (Serial.available() >= 5) {
    //String measuring_command = Serial.readStringUntil('?');  //https://youtu.be/VdSFwYrYqW0?si=4_7ddYQORFcv9GpS
    String data1 = Serial.readStringUntil('\n');
    data1 = data1 + "\n";
    digitalWrite(DE, HIGH);
    digitalWrite(RE, HIGH);
    Serial2.print(data1);  // Send data followed by newline character
    delay(1);
    digitalWrite(DE, LOW);  // Enable transmission
    digitalWrite(RE, LOW);
    delay(1);
  }
  /*currentMillis = millis();

  if (currentMillis - ReadingPreviousMillis >= ReadingInterval) {  // --- MPU6050 Accelerometer and Gyroscope Measurement ---

    digitalWrite(DE, HIGH);

    digitalWrite(RE, HIGH);

    delay(1);                            // Enable transmission
    String measuring_command = "1";  //https://youtu.be/VdSFwYrYqW0?si=4_7ddYQORFcv9GpS
    String laser_pointers_command = "1";
    String time_from_python = "14:30:339";
    String ultradist_from_python = "4415.35";
    String laserdist_from_python = "1462.27";
    String gyro_from_python = "-3.30";

    String data1 = measuring_command + laser_pointers_command + ":" + ultradist_from_python + ";" + laserdist_from_python + "\n";
    //String data1 = measuring_command + ":" + laser_pointers_command + "\n";
    //String data1 = measuring_com7and + "\n";
    //    String data1 = "vigheoighqeiohgioqegqeeqg;;g;qrhohrqopqeoegq\n";

    Serial2.print(data1);  // Send data followed by newline character
    Serial2.flush();       // Send data followed by newline character

    delay(1);
    Serial.print(data1);

    digitalWrite(DE, LOW);  // Enable transmission
    digitalWrite(RE, LOW);
    ReadingPreviousMillis = currentMillis;
  }*/

  if (Serial1.available() > 3) {
    digitalWrite(DE, LOW);
    digitalWrite(RE, LOW);
    // Enable transmission
    String topython = Serial1.readStringUntil('\n');  //https://youtu.be/VdSFwYrYqW0?si=4_7ddYQORFcv9GpS
    Serial1.flush();

    Serial.println(topython);
    delay(1);
    // Send data followed by newline character
  }
}
