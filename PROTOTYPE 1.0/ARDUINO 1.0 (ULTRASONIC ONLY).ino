const int triggerPin = 9;
const int echoPin = 10;
long duration;
int distance;
unsigned long previousMillis = 0;    // Stores the last time distance was printed
const unsigned long interval = 200;  // Interval in milliseconds (e.g., 100ms)

void setup() {
  Serial.begin(115200);
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  unsigned long currentMillis = millis();

  // Send a pulse to the sensor
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  // Measure the time it takes for the echo to return
  duration = pulseIn(echoPin, HIGH);

  // Calculate the distance (in cm)
  distance = duration * 0.0344 / 2;

  // Send distance every `interval`
  if (currentMillis - previousMillis >= interval) {
    Serial.println(distance);  // Send distance with a newline
    previousMillis = currentMillis;
  }
}
