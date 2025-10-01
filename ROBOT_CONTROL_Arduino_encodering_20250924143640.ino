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

// === Direction calibration (set these if wheels act reversed) ===
bool LEFT_INVERT  = false;
bool RIGHT_INVERT = true;

// ---- low-level dir setters (respect inversion flags)
void leftSet(bool fwd) {
  if (LEFT_INVERT) fwd = !fwd;
  if (fwd) { digitalWrite(motor1pin1, HIGH); digitalWrite(motor1pin2, LOW); }
  else     { digitalWrite(motor1pin1, LOW);  digitalWrite(motor1pin2, HIGH); }
}
void rightSet(bool fwd) {
  if (RIGHT_INVERT) fwd = !fwd;
  if (fwd) { digitalWrite(motor2pin1, LOW);  digitalWrite(motor2pin2, HIGH); }
  else     { digitalWrite(motor2pin1, HIGH); digitalWrite(motor2pin2, LOW);  }
}

void leftForward()  { leftSet(true);  }
void leftBackward() { leftSet(false); }
void leftStop()     { digitalWrite(motor1pin1, LOW); digitalWrite(motor1pin2, LOW); }

void rightForward()  { rightSet(true);  }
void rightBackward() { rightSet(false); }
void rightStop()     { digitalWrite(motor2pin1, LOW); digitalWrite(motor2pin2, LOW); }

void stopAll() { leftStop(); rightStop(); }
void spinLeft()  { leftBackward(); rightForward(); }
void spinRight() { leftForward();  rightBackward(); }

// ===== ENCODERS (D10 left, D11 right) =====
const uint8_t ENC_L = 10;
const uint8_t ENC_R = 11;

volatile long ticksL = 0, ticksR = 0;
int prevL = HIGH, prevR = HIGH;

const unsigned long TELEMETRY_MS = 50;

void setup() {
  pinMode(motor1pin1, OUTPUT); pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT); pinMode(motor2pin2, OUTPUT);
  pinMode(ENC_L, INPUT_PULLUP); pinMode(ENC_R, INPUT_PULLUP);
  stopAll();
  Serial.begin(115200);
  
  // Print header for telemetry
  Serial.println("=== Robot Telemetry ===");
  Serial.println("Format: T,time_ms,ticksL,ticksR");
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
        else              { if (innerLeft) leftForward();  else rightForward();  }
      } else {
        if (innerLeft) leftStop(); else rightStop();
      }
    }
  }
}

void loop() {
  if (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c != '\n' && c != '\r') {
      switch (c) {
        case 'w': case 'W': driveDir =  1; mode = MODE_DRIVE; break;
        case 's': case 'S': driveDir = -1; mode = MODE_DRIVE; break;
        case 'a': case 'A': if (mode == MODE_DRIVE) turnBias =  1;   break;
        case 'd': case 'D': if (mode == MODE_DRIVE) turnBias = -1;   break;
        case 'q': case 'Q': mode = MODE_SPIN; turnBias = 0; spinLeft();  break;
        case 'e': case 'E': mode = MODE_SPIN; turnBias = 0; spinRight(); break;
        case ' ':           mode = MODE_STOP; driveDir = 0; turnBias = 0; stopAll(); break;
        default: break;
      }
    }
  }
  if (mode == MODE_SPIN && driveDir != 0) mode = MODE_DRIVE;

  // Encoder polling - detect state changes (edges)
  int rl = digitalRead(ENC_L);
  int rr = digitalRead(ENC_R);
  if (rl != prevL) { ticksL++; prevL = rl; }
  if (rr != prevR) { ticksR++; prevR = rr; }

  applyDrive();

  // Telemetry - print every 50ms
  static unsigned long lastT = 0;
  unsigned long now = millis();
  if (now - lastT >= TELEMETRY_MS) {
    lastT = now;
    long dL = ticksL, dR = ticksR;
    ticksL = 0; ticksR = 0;
    
    // Print in clean CSV format: T,timestamp,left_ticks,right_ticks
    // Serial.print("T,");
    Serial.print(now);
    Serial.print(',');
    Serial.print(dL);
    Serial.print(',');
    Serial.println(dR);
  }
}
