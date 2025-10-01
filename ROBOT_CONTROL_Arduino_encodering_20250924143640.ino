// ===== DRIVE (unchanged pins) =====
int motor1pin1 = 2;
int motor1pin2 = 3;
int motor2pin1 = 4;
int motor2pin2 = 5;

const unsigned long TURN_PERIOD_MS = 160;
const unsigned int  TURN_DUTY_PCT  = 70;
const bool          HARD_TURN      = true;

enum DriveMode { MODE_STOP, MODE_DRIVE, MODE_SPIN };
DriveMode mode = MODE_STOP;

int driveDir = 0;   // +1 fwd, -1 back, 0 stop
int turnBias = 0;   // -1 L, +1 R

// left
void leftForward()  { digitalWrite(motor1pin1, HIGH); digitalWrite(motor1pin2, LOW); }
void leftBackward() { digitalWrite(motor1pin1, LOW);  digitalWrite(motor1pin2, HIGH); }
void leftStop()     { digitalWrite(motor1pin1, LOW);  digitalWrite(motor1pin2, LOW); }
// right (your wiring inversion)
void rightForward()  { digitalWrite(motor2pin1, LOW);  digitalWrite(motor2pin2, HIGH); }
void rightBackward() { digitalWrite(motor2pin1, HIGH); digitalWrite(motor2pin2, LOW); }
void rightStop()     { digitalWrite(motor2pin1, LOW);  digitalWrite(motor2pin2, LOW); }
void stopAll() { leftStop(); rightStop(); }
void spinLeft()  { leftBackward(); rightForward(); }
void spinRight() { leftForward();  rightBackward(); }

// ===== ENCODERS (D10 left, D11 right) =====
const uint8_t ENC_L = 10;
const uint8_t ENC_R = 11;

// edge counters (polling)
volatile long ticksL = 0, ticksR = 0;
int prevL = HIGH, prevR = HIGH;           // pullups -> idle HIGH

// telemetry rate
const unsigned long TELEMETRY_MS = 50;    // 20 Hz

void setup() {
  pinMode(motor1pin1, OUTPUT); pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT); pinMode(motor2pin2, OUTPUT);

  pinMode(ENC_L, INPUT_PULLUP);
  pinMode(ENC_R, INPUT_PULLUP);

  stopAll();
  Serial.begin(115200);
}

void applyDrive() {
  if (mode == MODE_STOP) { stopAll(); return; }
  if (mode == MODE_SPIN) { return; }

  if (driveDir > 0) { leftForward(); rightForward(); }
  else if (driveDir < 0) { leftBackward(); rightBackward(); }
  else { stopAll(); return; }

  if (turnBias != 0) {
    unsigned long t = millis() % TURN_PERIOD_MS;
    unsigned long win = (TURN_PERIOD_MS * TURN_DUTY_PCT) / 100;
    if (t < win) {
      bool innerLeft = (turnBias < 0);
      if (HARD_TURN) {
        if (driveDir > 0) { if (innerLeft) leftBackward(); else rightBackward(); }
        else              { if (innerLeft) leftForward();  else rightForward(); }
      } else {
        if (innerLeft) leftStop(); else rightStop();
      }
    }
  }
}

void loop() {
  // --- controls from Processing ---
  if (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c != '\n' && c != '\r') {
      switch (c) {
        case 'w': case 'W': driveDir =  1; mode = MODE_DRIVE; break; // W = fwd
        case 's': case 'S': driveDir = -1; mode = MODE_DRIVE; break; // S = back
        case 'a': case 'A': if (mode == MODE_DRIVE) turnBias = -1;   break;
        case 'd': case 'D': if (mode == MODE_DRIVE) turnBias =  1;   break;
        case 'q': case 'Q': mode = MODE_SPIN; turnBias = 0; spinLeft();  break;
        case 'e': case 'E': mode = MODE_SPIN; turnBias = 0; spinRight(); break;
        case ' ':           mode = MODE_STOP; driveDir = 0; turnBias = 0; stopAll(); break;
        default: break;
      }
    }
  }
  if (mode == MODE_SPIN && driveDir != 0) mode = MODE_DRIVE;

  // --- encoder edge counting (poll) ---
  int rl = digitalRead(ENC_L);
  int rr = digitalRead(ENC_R);
  if (rl != prevL) { ticksL++; prevL = rl; }  // count any change
  if (rr != prevR) { ticksR++; prevR = rr; }

  applyDrive();

  // --- telemetry at fixed rate ---
  static unsigned long lastT = 0;
  unsigned long now = millis();
  if (now - lastT >= TELEMETRY_MS) {
    lastT = now;
    long dL = ticksL, dR = ticksR;
    ticksL = 0; ticksR = 0;
    Serial.print("T,"); Serial.print(now);
    Serial.print(',');  Serial.print(dL);
    Serial.print(',');  Serial.println(dR);
  }
}



