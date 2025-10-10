// === MOTOR DRIVER PINS ===
const int motor1pin1 = 2;
const int motor1pin2 = 3;
const int motor2pin1 = 4;
const int motor2pin2 = 5;

// === ULTRASONIC FRONT SENSOR ONLY ===
const int trigFront = 6;
const int echoFront = 7;

// === ENCODERS ===
const uint8_t ENC_L = 10;
const uint8_t ENC_R = 11;

volatile long ticksL = 0, ticksR = 0;
int prevL = HIGH, prevR = HIGH;

// === DRIVE STATE ===
enum DriveMode { MODE_STOP, MODE_DRIVE };
DriveMode mode = MODE_STOP;

int driveDir = 0; // +1 = forward, -1 = backward, 0 = stop

// === FOLLOW-ME CONTROL CONFIG ===
const float set_point = 25.0; // cm
const float deadzone = 2.0;   // cm tolerance
const float slowZone = 10.0;  // simulate slow motion below this error

// === TELEMETRY CONFIG ===
const unsigned long TELEMETRY_MS = 100;
float frontDistance = -1;

void setup() {
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);

  pinMode(trigFront, OUTPUT);
  pinMode(echoFront, INPUT);

  pinMode(ENC_L, INPUT_PULLUP);
  pinMode(ENC_R, INPUT_PULLUP);

  Serial.begin(115200);
  stopAll();

  Serial.println("=== Follow-Me Robot (Front Sensor + Both Encoders) ===");
  Serial.println("Time(ms),Front(cm),DriveDir,TicksL,TicksR");
}

void loop() {
  // Read front sensor only
  frontDistance = getDistanceCM(trigFront, echoFront);

  // === P-Controller (Digital Zones Only) ===
  if (frontDistance > 0) {
    float error = frontDistance - set_point;

    if (abs(error) < deadzone) {
      driveDir = 0;
      mode = MODE_STOP;
    } else if (abs(error) < slowZone) {
      static unsigned long lastToggle = 0;
      static bool movePulse = false;
      unsigned long now = millis();

      if (now - lastToggle >= 400) {
        movePulse = !movePulse;
        lastToggle = now;
      }

      if (movePulse) {
        driveDir = (error > 0) ? 1 : -1;
        mode = MODE_DRIVE;
      } else {
        driveDir = 0;
        mode = MODE_STOP;
      }
    } else {
      driveDir = (error > 0) ? 1 : -1;
      mode = MODE_DRIVE;
    }
  } else {
    driveDir = 0;
    mode = MODE_STOP;
  }

  applyDrive();

  // === ENCODER TICK COUNTING (BOTH) ===
  int rl = digitalRead(ENC_L);
  int rr = digitalRead(ENC_R);
  if (rl != prevL) { ticksL++; prevL = rl; }
  if (rr != prevR) { ticksR++; prevR = rr; }

  // === TELEMETRY OUTPUT ===
  static unsigned long lastT = 0;
  unsigned long now = millis();
  if (now - lastT >= TELEMETRY_MS) {
    lastT = now;
    long dL = ticksL; long dR = ticksR;
    ticksL = 0; ticksR = 0;

    Serial.print(now); Serial.print(",");
    Serial.print(frontDistance); Serial.print(",");
    Serial.print(driveDir); Serial.print(",");
    Serial.print(dL); Serial.print(",");
    Serial.println(dR);
  }
}

void stopAll() {
  digitalWrite(motor1pin1, LOW); digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, LOW); digitalWrite(motor2pin2, LOW);
}

void leftForward()  { digitalWrite(motor1pin1, HIGH); digitalWrite(motor1pin2, LOW); }
void leftBackward() { digitalWrite(motor1pin1, LOW);  digitalWrite(motor1pin2, HIGH); }
void rightForward() { digitalWrite(motor2pin1, HIGH); digitalWrite(motor2pin2, LOW); }
void rightBackward(){ digitalWrite(motor2pin1, LOW);  digitalWrite(motor2pin2, HIGH); }

void applyDrive() {
  if (mode == MODE_STOP || driveDir == 0) {
    stopAll();
  } else if (driveDir > 0) {
    leftForward(); rightForward();
  } else {
    leftBackward(); rightBackward();
  }
}

float getDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(5);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return -1;
  return (duration / 2.0) / 29.1;
}
