// === MOTOR DRIVER PINS ===
const int motor1pin1 = 2;  // Left forward
const int motor1pin2 = 3;  // Left backward
const int motor2pin1 = 4;  // Right forward
const int motor2pin2 = 5;  // Right backward

// === ULTRASONIC SENSORS ===
const int trigFront = 6;
const int echoFront = 7;
const int trigRight = 8;
const int echoRight = 9;

// === ENCODERS ===
const uint8_t ENC_L = 10;
const uint8_t ENC_R = 11;
volatile long ticksL = 0, ticksR = 0;
int prevL = HIGH, prevR = HIGH;

// === CONFIG ===
const float SIDE_SETPOINT = 25.0;     // Ideal distance to wall
const float SIDE_THRESHOLD = 30.0;    // Wall lost if > 30 cm
const float FRONT_STOP = 10.0;        // Too close to front wall
const float KP = 3.0;                 // Wall-following gain

// === TELEMETRY CONFIG ===
const unsigned long TELEMETRY_MS = 100;
float frontDistance = -1;
float rightDistance = -1;

void setup() {
  pinMode(motor1pin1, OUTPUT); pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT); pinMode(motor2pin2, OUTPUT);

  pinMode(trigFront, OUTPUT); pinMode(echoFront, INPUT);
  pinMode(trigRight, OUTPUT); pinMode(echoRight, INPUT);

  pinMode(ENC_L, INPUT_PULLUP); pinMode(ENC_R, INPUT_PULLUP);

  Serial.begin(115200);
  stopAll();
  Serial.println("=== Wall Follower with Adaptive Reacquisition ===");
  Serial.println("Time(ms),Front(cm),Right(cm),TicksL,TicksR");
}

void loop() {
  frontDistance = getDistanceCM(trigFront, echoFront);
  rightDistance = getDistanceCM(trigRight, echoRight);

  Serial.print("Front: "); Serial.print(frontDistance);
  Serial.print(" cm | Right: "); Serial.println(rightDistance);

  if (frontDistance < 0 || rightDistance < 0) {
    stopAll(); return;
  }

  if (frontDistance < FRONT_STOP) {
    handleInsideCorner();
  } else if (rightDistance > SIDE_THRESHOLD) {
    searchForWallAdaptive();
  } else {
    followWall();  // Maintain ~25 cm from wall
  }

  // === Encoder Telemetry ===
  int rl = digitalRead(ENC_L);
  int rr = digitalRead(ENC_R);
  if (rl != prevL) { ticksL++; prevL = rl; }
  if (rr != prevR) { ticksR++; prevR = rr; }

  static unsigned long lastT = 0;
  unsigned long now = millis();
  if (now - lastT >= TELEMETRY_MS) {
    lastT = now;
    long dL = ticksL; long dR = ticksR;
    ticksL = 0; ticksR = 0;
    Serial.print(now); Serial.print(",");
    Serial.print(frontDistance); Serial.print(",");
    Serial.print(rightDistance); Serial.print(",");
    Serial.print(dL); Serial.print(",");
    Serial.println(dR);
  }
}

// === Motion Control ===

void stopAll() {
  digitalWrite(motor1pin1, LOW); digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW); digitalWrite(motor2pin2, LOW);
}

void driveForward() {
  digitalWrite(motor1pin1, HIGH); digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, HIGH); digitalWrite(motor2pin2, LOW);
}

void steerLeft() {
  digitalWrite(motor1pin1, HIGH);  // Left forward
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW);   // Right backward
  digitalWrite(motor2pin2, HIGH);
}

void steerRight() {
  digitalWrite(motor1pin1, LOW);   // Left backward
  digitalWrite(motor1pin2, HIGH);
  digitalWrite(motor2pin1, HIGH);  // Right forward
  digitalWrite(motor2pin2, LOW);
}

// === Inside Corner ===

void handleInsideCorner() {
  Serial.println("Inside corner → Turning LEFT");
  stopAll(); delay(1000);
  steerLeft(); delay(700);  // adjust if needed
  stopAll(); delay(200);
  driveForward(); delay(400);  // move out of corner
}

// === Adaptive Wall Reacquisition ===

void searchForWallAdaptive() {
  Serial.println("Wall lost → Try slow arc RIGHT...");

  // Arc turn (left wheel forward only)
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, LOW);
  delay(700);

  stopAll(); delay(200);
  float check = getDistanceCM(trigRight, echoRight);
  Serial.print("Right after arc: "); Serial.println(check);
  if (check < SIDE_THRESHOLD) {
    Serial.println("Wall reacquired after arc.");
    return;
  }

  // Pivot right
  Serial.println("Arc failed → Pivot RIGHT...");
  steerRight(); delay(700);
  stopAll(); delay(200);

  check = getDistanceCM(trigRight, echoRight);
  Serial.print("Right after pivot: "); Serial.println(check);
  if (check < SIDE_THRESHOLD) {
    Serial.println("Wall reacquired after pivot.");
    return;
  }

  // Sweep left
  Serial.println("Still no wall → Sweep LEFT...");
  steerLeft(); delay(400);
  stopAll(); delay(200);
}

// === Wall Following Logic (Proportional) ===

void followWall() {
  float error = rightDistance - SIDE_SETPOINT;
  float absError = abs(error);

  if (absError < 2) {
    driveForward();
  } else if (error > 0) {
    // Too far from wall → steer right
    digitalWrite(motor1pin1, HIGH); digitalWrite(motor1pin2, LOW);
    digitalWrite(motor2pin1, HIGH); digitalWrite(motor2pin2, LOW);
    delay(KP * absError);
  } else {
    // Too close → steer left
    digitalWrite(motor1pin1, HIGH); digitalWrite(motor1pin2, LOW);
    digitalWrite(motor2pin1, HIGH); digitalWrite(motor2pin2, LOW);
    delay(KP * absError);
  }
}

// === Ultrasonic Distance Function ===

float getDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(5);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return -1;
  return (duration / 2.0) / 29.1;
}
