// ===== DRIVE PINS =====
int motor1pin1 = 2;
int motor1pin2 = 3;
int motor2pin1 = 4;
int motor2pin2 = 5;

// ===== ULTRASONICS =====
const int trigFront = 6;
const int echoFront = 7;
const int trigRight = 8;
const int echoRight = 9;

// ===== ENCODERS =====
const uint8_t ENC_L = 10;
const uint8_t ENC_R = 11;

// ===== TURN / SPEED (time-sliced; no hardware PWM) =====
const unsigned long TURN_PERIOD_MS  = 160;
const unsigned int  TURN_DUTY_PCT   = 70;
const bool          HARD_TURN       = true;

const unsigned long SPEED_PERIOD_MS = 60;
unsigned int        speedDutyPct    = 100;   // manual default = full

// ===== MODES =====
enum DriveMode { MODE_STOP, MODE_DRIVE, MODE_SPIN };
enum AutonMode { AUTON_OFF, AUTON_FOLLOW_ME, AUTON_RIGHT_WALL };
DriveMode mode = MODE_STOP;
AutonMode auton = AUTON_OFF;

// drive state
int driveDir = 0;   // +1 fwd, -1 back, 0 stop
int turnBias = 0;   // -1 L, +1 R

// wheel invert (keep your last working combo)
bool LEFT_INVERT  = false;
bool RIGHT_INVERT = true;

// encoders
volatile long ticksL = 0, ticksR = 0;
int prevL = HIGH, prevR = HIGH;

// telemetry timers
const unsigned long TELEMETRY_MS = 50;  // encoders @20Hz
const unsigned long ULTRA_MS     = 60;  // ultrasonics ~16Hz
unsigned long lastTele = 0, lastUltra = 0;

// ===== low-level wheel helpers =====
void leftSet(bool fwd)  { if (LEFT_INVERT)  fwd = !fwd;
  if (fwd) { digitalWrite(motor1pin1, HIGH); digitalWrite(motor1pin2, LOW); }
  else     { digitalWrite(motor1pin1, LOW);  digitalWrite(motor1pin2, HIGH); } }
void rightSet(bool fwd) { if (RIGHT_INVERT) fwd = !fwd;
  if (fwd) { digitalWrite(motor2pin1, LOW);  digitalWrite(motor2pin2, HIGH); }
  else     { digitalWrite(motor2pin1, HIGH); digitalWrite(motor2pin2, LOW);  } }

void leftForward()  { leftSet(true); }
void leftBackward() { leftSet(false); }
void leftStop()     { digitalWrite(motor1pin1, LOW); digitalWrite(motor1pin2, LOW); }
void rightForward()  { rightSet(true); }
void rightBackward() { rightSet(false); }
void rightStop()     { digitalWrite(motor2pin1, LOW); digitalWrite(motor2pin2, LOW); }
void stopAll() { leftStop(); rightStop(); }
void spinLeft()  { leftBackward(); rightForward(); }
void spinRight() { leftForward();  rightBackward(); }

// ===== ultrasonic helpers =====
long echoTimeUS(int trig, int echo, unsigned long to_us=25000UL) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  return pulseIn(echo, HIGH, to_us);     // timeout safe
}
float usToCm(long us) {
  if (us == 0) return 300.0;             // timeout -> big number
  return (us * 0.0343f) / 2.0f;
}
float readCmMedian3(int trig, int echo) {
  long a = echoTimeUS(trig, echo);
  long b = echoTimeUS(trig, echo);
  long c = echoTimeUS(trig, echo);
  long m = (a > b) ? ((a < c) ? a : ((b > c) ? b : c))
                   : ((b < c) ? b : ((a > c) ? a : c));
  float cm = usToCm(m);
  if (cm < 2.0) cm = 2.0;
  if (cm > 300.0) cm = 300.0;
  return cm;
}

// ===== auton controllers =====
// Follow-Me (front setpoint ~25 cm)
const float SET_CM_FRONT = 25.0;
const float KP_FRONT     = 2.5;   // tune 1.5..4.0

void runFollowMe(float front_cm) {
  float err = SET_CM_FRONT - front_cm;       // + => too far
  if (err > 0.5)       driveDir = +1;
  else if (err < -0.5) driveDir = -1;
  else                 driveDir = 0;

  float mag = fabs(err);
  unsigned duty = (unsigned)constrain(mag * 6.0f + 20.0f, 0.0f, 100.0f);
  speedDutyPct = duty;
  mode = (driveDir == 0) ? MODE_STOP : MODE_DRIVE;
  turnBias = 0;                              // straight in this mode
}

// Right-Wall (right setpoint ~20 cm)
const float SET_CM_RIGHT = 20.0;
const float KP_RIGHT     = 0.12;

unsigned int currentTurnDuty = 0;            // per-cycle bias override

void runRightWall(float right_cm) {
  float err = SET_CM_RIGHT - right_cm;       // + => too far from wall
  mode = MODE_DRIVE;
  driveDir = +1;

  int desiredBias = (err > 0.5f) ? +1 : (err < -0.5f ? -1 : 0); // +1 R, -1 L
  float mag = min(fabs(err) * KP_RIGHT * 100.0f, 100.0f);
  unsigned steerDuty = (unsigned)constrain(40.0f + mag * 0.5f, 40.0f, 90.0f);

  turnBias = desiredBias;
  currentTurnDuty = steerDuty;
  speedDutyPct = 70;                          // base forward
}

// ===== drive applier (time-sliced speed + bias) =====
void applyDrive() {
  if (mode == MODE_STOP) { stopAll(); return; }
  if (mode == MODE_SPIN) { return; }

  unsigned long tSpeed = millis() % SPEED_PERIOD_MS;
  unsigned long onWin  = (SPEED_PERIOD_MS * speedDutyPct) / 100;
  if (tSpeed >= onWin) { stopAll(); return; }

  if (driveDir > 0) { leftForward(); rightForward(); }
  else if (driveDir < 0) { leftBackward(); rightBackward(); }
  else { stopAll(); return; }

  if (turnBias != 0) {
    unsigned int biasDuty = (currentTurnDuty > 0) ? currentTurnDuty : TURN_DUTY_PCT;
    unsigned long tTurn   = millis() % TURN_PERIOD_MS;
    unsigned long turnWin = (TURN_PERIOD_MS * biasDuty) / 100;
    if (tTurn < turnWin) {
      bool innerLeft = (turnBias < 0);
      if (HARD_TURN) {
        if (driveDir > 0) { if (innerLeft) leftBackward(); else rightBackward(); }
        else              { if (innerLeft) leftForward();  else rightForward();  }
      } else {
        if (innerLeft) leftStop(); else rightStop();
      }
    }
  }
  currentTurnDuty = 0;   // reset per-cycle override
}

void setup() {
  pinMode(motor1pin1, OUTPUT); pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT); pinMode(motor2pin2, OUTPUT);
  pinMode(ENC_L, INPUT_PULLUP); pinMode(ENC_R, INPUT_PULLUP);
  pinMode(trigFront, OUTPUT); pinMode(echoFront, INPUT);
  pinMode(trigRight, OUTPUT); pinMode(echoRight, INPUT);

  stopAll();
  Serial.begin(115200);
}

void loop() {
  // ---- commands ----
  if (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c != '\n' && c != '\r') {
      switch (c) {
        // manual
        case 'w': case 'W': auton = AUTON_OFF; speedDutyPct=100; driveDir=+1; mode=MODE_DRIVE; turnBias=0; break;
        case 's': case 'S': auton = AUTON_OFF; speedDutyPct=100; driveDir=-1; mode=MODE_DRIVE; turnBias=0; break;
        case 'a': case 'A': if (auton==AUTON_OFF && mode==MODE_DRIVE) turnBias = -1; break; // A = left
        case 'd': case 'D': if (auton==AUTON_OFF && mode==MODE_DRIVE) turnBias =  1; break; // D = right
        case 'q': case 'Q': auton = AUTON_OFF; mode=MODE_SPIN; turnBias=0; spinLeft();  break;
        case 'e': case 'E': auton = AUTON_OFF; mode=MODE_SPIN; turnBias=0; spinRight(); break;
        case ' ':           auton = AUTON_OFF; mode=MODE_STOP; driveDir=0; turnBias=0; stopAll(); break;

        // auton
        case 'F': auton = AUTON_FOLLOW_ME;    break;
        case 'R': auton = AUTON_RIGHT_WALL;   break;
        case 'X': auton = AUTON_OFF; mode=MODE_STOP; driveDir=0; turnBias=0; stopAll(); break;

        default: break;
      }
    }
  }
  if (mode == MODE_SPIN && driveDir != 0) mode = MODE_DRIVE;

  // ---- encoders (edge poll) ----
  int rl = digitalRead(ENC_L);
  int rr = digitalRead(ENC_R);
  if (rl != prevL) { ticksL++; prevL = rl; }
  if (rr != prevR) { ticksR++; prevR = rr; }

  // ---- ultrasonics (periodic) ----
  static float f_cm = -1, r_cm = -1;
  unsigned long now = millis();
  if (now - lastUltra >= ULTRA_MS) {
    lastUltra = now;
    f_cm = readCmMedian3(trigFront, echoFront);
    r_cm = readCmMedian3(trigRight, echoRight);

    // run auton (updates speed/dir/bias)
    if (auton == AUTON_FOLLOW_ME)      runFollowMe(f_cm);
    else if (auton == AUTON_RIGHT_WALL) runRightWall(r_cm);
  }

  // ---- drive ----
  applyDrive();

  // ---- telemetry ----
  if (now - lastTele >= TELEMETRY_MS) {
    lastTele = now;
    long dL = ticksL, dR = ticksR;
    ticksL = 0; ticksR = 0;

    // encoders
    Serial.print("T,");
    Serial.print(now); Serial.print(',');
    Serial.print(dL);  Serial.print(',');
    Serial.println(dR);

    // ultrasonics (ints for readability)
    Serial.print("U,");
    Serial.print(now); Serial.print(',');
    Serial.print((int)f_cm); Serial.print(',');
    Serial.println((int)r_cm);
  }
}

