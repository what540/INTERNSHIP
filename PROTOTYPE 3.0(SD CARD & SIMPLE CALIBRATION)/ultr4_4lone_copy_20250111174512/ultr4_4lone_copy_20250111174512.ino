#include <cmath>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

const int triggerPin = 33;  // Trigger pin for ultrasonic sensor
const int echoPin = 32;     // Echo pin for ultrasonic sensor

Adafruit_VL53L0X lox;  // Laser distance sensor

unsigned long previousMillis = 0;         // Timer for interval-based actions
unsigned long ReadingPreviousMillis = 0;  // Timer for reading intervals

int currentIndex = 0;                                // Current index to store the reading
const unsigned long interval = 1000;                 // Interval for calculating stats (1 second)
const unsigned long ReadingInterval = 100;           // Interval between each ultrasonic reading (50 ms)
const int numReadings = interval / ReadingInterval;  // Number of readings per interval (20 readings per second)

double ultraReadings[numReadings];  // Array to store ultrasonic sensor readings
double laserReadings[numReadings];  // Array to store the readings

double laser = 0;
double previousLaser = 0;  // Stores previous laser distance
double laserSum = 0;       // Stores previous laser distance

long double previousUltrasonic = 0;  // Stores previous ultrasonic distance
double ultrasonicDistanceSum = 0;    // Stores the sum of ultrasonic readings for average calculation

void setup() {
  Serial.begin(115200);         // Start the serial communication
  pinMode(triggerPin, OUTPUT);  // Set trigger pin as output
  pinMode(echoPin, INPUT);      // Set echo pin as input

  if (!lox.begin()) {
    Serial.println(F("Failed to initialize VL53L0X sensor"));
    while (1)
      ;
  }
}

void loop() {
  unsigned long currentMillis = millis();  // Get the current time

  // Collect ultrasonic readings at the defined interval
  if (currentMillis - ReadingPreviousMillis >= ReadingInterval) {

    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);                                 // Perform measurement
    if (measure.RangeStatus != 4 && measure.RangeMilliMeter != 8191)  // Phase failures have incorrect data
      laser = (double)measure.RangeMilliMeter - 6;
    else
      laser = 0;  // Error code for invalid measurement
    // Send a pulse to trigger the ultrasonic sensor
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);

    // Read the echo pin and calculate the distance
    long duration = pulseIn(echoPin, HIGH);
    double distance = duration * 0.034 / 2;  // Convert to cm

    // Store the reading in the array
    laserReadings[currentIndex] = laser;
    ultraReadings[currentIndex] = distance;
    // Print the current reading
    Serial.print("currentIndex: ");
    Serial.print(currentIndex);
    Serial.printf(" %.0f    ", laser);
    Serial.printf(" %.3f\n", distance);
    currentIndex++;

    // Wrap around to 0 when the array is full
    if (currentIndex >= numReadings) {

      // Calculate and print statistics every second
      if (currentMillis - previousMillis >= interval) {
        double maxValueInterval = ultraReadings[0];
        double minValueInterval = ultraReadings[0];
        double RANGEinterval = 0;
        ultrasonicDistanceSum = 0;  // Reset sum for averaging

        double LaserMaxValueInterval = laserReadings[0];
        double LaserMinValueInterval = laserReadings[0];
        double LaserRANGEinterval = 0;
        laserSum = 0;

        // Calculate max, min, and sum for the readings
        for (int i = 0; i < numReadings; i++) {
          ultrasonicDistanceSum += ultraReadings[i];
          if (ultraReadings[i] > maxValueInterval) {
            maxValueInterval = ultraReadings[i];
          } else if (ultraReadings[i] < minValueInterval) {
            minValueInterval = ultraReadings[i];
          }

          laserSum += laserReadings[i];
          if (laserReadings[i] > LaserMaxValueInterval) {
            LaserMaxValueInterval = laserReadings[i];
          } else if (laserReadings[i] < LaserMinValueInterval) {
            LaserMinValueInterval = laserReadings[i];
          }
        }

        // Calculate range (difference between max and min)
        RANGEinterval = maxValueInterval - minValueInterval;

        // Calculate the average distance
        double distance = ultrasonicDistanceSum / numReadings;

        // Calculate the variance (mean squared difference)
        double sqDiff = 0;
        for (int i = 0; i < numReadings; i++) {
          sqDiff += (ultraReadings[i] - distance) * (ultraReadings[i] - distance);
          ultraReadings[i] = 0;
        }

        // Variance and standard deviation
        sqDiff = sqDiff / numReadings;
        double stdDev = sqrt(sqDiff);

        // Calculate the difference in distance between this and the previous second
        double diffUltraInOneSec = distance - previousUltrasonic;


        LaserRANGEinterval = LaserMaxValueInterval - LaserMinValueInterval;
        laser = laserSum / numReadings;
        double diffLaserInOneSec = laser - previousLaser;
        double LaserSqDiff = 0;
        for (int i = 0; i < numReadings; i++) {
          LaserSqDiff += (laserReadings[i] - laser) * (laserReadings[i] - laser);
          laserReadings[i] = 0;
        }
        LaserSqDiff = LaserSqDiff / numReadings;
        double LaserStdDev = sqrt(LaserSqDiff);

        // Print the results
        Serial.printf("  Avg distance: %.4f", laser);
        Serial.printf("  Prev: %.4f", previousLaser);
        Serial.printf("  Difference: %.4f", diffLaserInOneSec);
        Serial.printf("  R4NGEin1s: %.4f", LaserRANGEinterval);
        Serial.printf("  var: %.4f", LaserSqDiff);
        Serial.printf("  stdDev: %.4f\n", LaserStdDev);
        Serial.printf("  Avg distance: %.4f", distance);
        Serial.printf("  Prev: %.4f", previousUltrasonic);
        Serial.printf("  Difference: %.4f", diffUltraInOneSec);
        Serial.printf("  R4NGEin1s: %.4f", RANGEinterval);
        Serial.printf("  var: %.4f", sqDiff);
        Serial.printf("  stdDev: %.4f\n\n", stdDev);

        // Update previous values for the next iteration
        previousUltrasonic = distance;
        previousLaser = laser;
        previousMillis = currentMillis;  // Reset timing for the next interval
      }

      currentIndex = 0;
    }




    ReadingPreviousMillis = currentMillis;  // Reset the reading timer
  }





  // Other tasks can go here, if needed.
}
