#include <Servo.h>

// === PIN DEFINITIONS ===
const int motor1pin1 = 6;  // Left forward (PWM)
const int motor1pin2 = 3;  // Left backward
const int motor2pin1 = 9;  // Right forward (PWM)
const int motor2pin2 = 5;  // Right backward

const int trigFront = 13;
const int echoFront = 10;
const int servoPin = 8;
const int buttonPin = 7;

// === CONFIG ===
const int SWEEP_STEP = 30;
const unsigned long SWEEP_INTERVAL = 300;
const float STOP_DISTANCE = 5.0;     // cm distance to stop
const int SPEED_FORWARD = 120;
const int SPEED_TURN = 100;
const int SPEED_SLOW = 80;

Servo ultrasonicServo;

// === VARIABLES ===
unsigned long lastSweepTime = 0;
bool sweepForward = true;
int sweepAngle = 0;
float closestDistance = 9999;
int closestAngle = 90;
unsigned long reachedTime = 0;

enum RobotState { WAITING, SWEEPING, MOVING };
RobotState currentState = WAITING;

// === SETUP ===
void setup() {
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);

  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);

  pinMode(buttonPin, INPUT_PULLUP);

  ultrasonicServo.attach(servoPin);
  ultrasonicServo.write(90);

  Serial.begin(9600);
  stopAll();
  Serial.println("Press button to start searching...");
}

// === LOOP ===
void loop() {
  switch (currentState) {
    case WAITING:
      stopAll();
      if (digitalRead(buttonPin) == LOW) {
        Serial.println("Button pressed! Starting search...");
        delay(500);
        currentState = SWEEPING;
      }
      break;

    case SWEEPING:
      stopAll();
      if (millis() - lastSweepTime >= SWEEP_INTERVAL) {
        lastSweepTime = millis();
        ultrasonicServo.write(sweepAngle);
        float dist = getDistanceCM(trigFront, echoFront);
        Serial.print("Angle "); Serial.print(sweepAngle);
        Serial.print(" => "); Serial.println(dist);

        if (dist > 0 && dist < closestDistance) {
          closestDistance = dist;
          closestAngle = sweepAngle;
        }

        if (sweepForward) {
          sweepAngle += SWEEP_STEP;
          if (sweepAngle >= 180) sweepForward = false;
        } else {
          sweepAngle -= SWEEP_STEP;
          if (sweepAngle <= 0) {
            sweepForward = true;
            // Finished full sweep
            Serial.print("Closest object at "); Serial.print(closestAngle);
            Serial.print(" deg, "); Serial.print(closestDistance); Serial.println(" cm");
            ultrasonicServo.write(90);
            delay(500);
            currentState = MOVING;
          }
        }
      }
      break;

    case MOVING: {
      // Turn toward object
      if (closestAngle < 70) {
        Serial.println("Turning left...");
        steerLeft();
      } else if (closestAngle > 110) {
        Serial.println("Turning right...");
        steerRight();
      } else {
        Serial.println("Driving forward...");
        driveForward(SPEED_FORWARD);
      }

      float dist = getDistanceCM(trigFront, echoFront);
      Serial.print("Front dist: "); Serial.println(dist);

      // Stop when object reached
      if (dist > 0 && dist <= STOP_DISTANCE) {
        Serial.println("Object reached!");
        stopAll();
        delay(1000);
        closestDistance = 9999;
        currentState = SWEEPING;  // Start searching again
      }
      break;
    }
  }
}

// === MOVEMENT FUNCTIONS ===
void driveForward(int speed) {
  analogWrite(motor1pin1, speed);
  digitalWrite(motor1pin2, LOW);
  analogWrite(motor2pin1, speed);
  digitalWrite(motor2pin2, LOW);
}

void steerLeft() {
  analogWrite(motor1pin1, SPEED_SLOW);
  digitalWrite(motor1pin2, LOW);
  analogWrite(motor2pin1, SPEED_TURN);
  digitalWrite(motor2pin2, LOW);
  delay(200);
}

void steerRight() {
  analogWrite(motor1pin1, SPEED_TURN);
  digitalWrite(motor1pin2, LOW);
  analogWrite(motor2pin1, SPEED_SLOW);
  digitalWrite(motor2pin2, LOW);
  delay(200);
}

void stopAll() {
  analogWrite(motor1pin1, 0);
  digitalWrite(motor1pin2, LOW);
  analogWrite(motor2pin1, 0);
  digitalWrite(motor2pin2, LOW);
}

float getDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return -1;
  return (duration / 2.0) / 29.1;
}
