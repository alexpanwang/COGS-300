/*
 * Based on code by Rui Santos - https://randomnerdtutorials.com
 * Modified to handle two HC-SR04 ultrasonic sensors + 1 servo motor
 */

#include <Servo.h>

// Sensor A pins
const int trigPinA = 8;
const int echoPinA = 9;

// Sensor B pins
const int trigPinB = 6;
const int echoPinB = 7;

// Servo pin
const int servoPin = 9;

int pos = 0;
int maxDist = 100; // Max distance in cm (adjustable)
int minDist = 5;   // Minimum distance in cm

Servo myservo;

long durationA, durationB;
float distanceA, distanceB;

void setup() {
  Serial.begin(9600);
  myservo.attach(servoPin);

  pinMode(trigPinA, OUTPUT);
  pinMode(echoPinA, INPUT);

  pinMode(trigPinB, OUTPUT);
  pinMode(echoPinB, INPUT);
}

float getDistanceCM(int trigPin, int echoPin) {
  // Ensure trigger pin is LOW
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);

  // Trigger pulse
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read echo
  long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout

  // Return distance in cm
  return (duration > 0) ? (duration / 2.0) / 29.1 : -1; // -1 means timeout
}

void loop() {
  // Read distances from both sensors
  distanceA = getDistanceCM(trigPinA, echoPinA);
  delay(50); // Prevent crosstalk
  distanceB = getDistanceCM(trigPinB, echoPinB);

  // Choose the closer distance (if both are valid)
  float distance = -1;
  if (distanceA > 0 && distanceB > 0) {
    distance = min(distanceA, distanceB);
  } else if (distanceA > 0) {
    distance = distanceA;
  } else if (distanceB > 0) {
    distance = distanceB;
  }

  // Show both distances
  Serial.print("Sensor A: ");
  Serial.print(distanceA);
  Serial.print(" cm | Sensor B: ");
  Serial.print(distanceB);
  Serial.print(" cm | Used: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Map distance to servo position
  if (distance > 0) {
    pos = map(distance, minDist, maxDist, 0, 180);
    pos = constrain(pos, 0, 180);
    myservo.write(pos);
  }

  delay(250);
}
