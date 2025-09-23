// pins (same as yours)
int motor1pin1 = 2;
int motor1pin2 = 3;
int motor2pin1 = 4;
int motor2pin2 = 5;

// turn bias settings
const unsigned long TURN_PERIOD_MS = 160;
const unsigned int  TURN_DUTY_PCT  = 70;
const bool          HARD_TURN      = true;

enum DriveMode { MODE_STOP, MODE_DRIVE, MODE_SPIN };
DriveMode mode = MODE_STOP;

int driveDir = 0;   // +1 fwd, -1 back, 0 stop
int turnBias = 0;   // -1 left, 0 straight, +1 right

// left wheel
void leftForward()  { digitalWrite(motor1pin1, HIGH); digitalWrite(motor1pin2, LOW); }
void leftBackward() { digitalWrite(motor1pin1, LOW);  digitalWrite(motor1pin2, HIGH); }
void leftStop()     { digitalWrite(motor1pin1, LOW);  digitalWrite(motor1pin2, LOW); }

// right wheel (inverted wiring)
void rightForward()  { digitalWrite(motor2pin1, LOW);  digitalWrite(motor2pin2, HIGH); }
void rightBackward() { digitalWrite(motor2pin1, HIGH); digitalWrite(motor2pin2, LOW); }
void rightStop()     { digitalWrite(motor2pin1, LOW);  digitalWrite(motor2pin2, LOW); }

void stopAll()   { leftStop(); rightStop(); }

// spins (in place)
void spinLeft()  { leftBackward(); rightForward(); }
void spinRight() { leftForward();  rightBackward(); }

void setup() {
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  stopAll();
  Serial.begin(115200);
}

// drive according to state (no delay)
void applyDrive() {
  if (mode == MODE_STOP) { stopAll(); return; }
  if (mode == MODE_SPIN) { return; }  // spin* already set pins

  // base motion
  if (driveDir > 0) {        // fwd
    leftForward(); rightForward();
  } else if (driveDir < 0) { // back
    leftBackward(); rightBackward();
  } else {
    stopAll(); return;
  }

  // bias turning
  if (turnBias != 0) {
    unsigned long t = millis() % TURN_PERIOD_MS;
    unsigned long win = (TURN_PERIOD_MS * TURN_DUTY_PCT) / 100;
    if (t < win) {
      bool innerLeft = (turnBias < 0);
      if (HARD_TURN) {
        // reverse inner wheel briefly
        if (driveDir > 0) { if (innerLeft) leftBackward(); else rightBackward(); }
        else              { if (innerLeft) leftForward();  else rightForward(); }
      } else {
        // or just stop inner wheel
        if (innerLeft) leftStop(); else rightStop();
      }
    }
  }
}

void loop() {
  // serial controls
  if (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c != '\n' && c != '\r') {
      switch (c) {
        case 'w': case 'W': driveDir = -1; mode = MODE_DRIVE; break; // W = back
        case 's': case 'S': driveDir =  1; mode = MODE_DRIVE; break; // S = fwd
        case 'a': case 'A': if (mode == MODE_DRIVE) turnBias = -1;   break; // left bias
        case 'd': case 'D': if (mode == MODE_DRIVE) turnBias =  1;   break; // right bias
        case 'q': case 'Q': mode = MODE_SPIN; turnBias = 0; spinLeft();  break;
        case 'e': case 'E': mode = MODE_SPIN; turnBias = 0; spinRight(); break;
        case ' ':           mode = MODE_STOP; driveDir = 0; turnBias = 0; stopAll(); break;
        default: break;
      }
    }
  }

  // if spinning and we already have a driveDir, go back to DRIVE
  if (mode == MODE_SPIN && driveDir != 0) mode = MODE_DRIVE;

  applyDrive();
}


