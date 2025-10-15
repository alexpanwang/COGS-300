// === MOTOR DRIVER PINS (same) ===
const int motor1pin1 = 2;
const int motor1pin2 = 3;
const int motor2pin1 = 4;
const int motor2pin2 = 5;

// === ULTRASONICS ===
const int trigFront = 6;
const int echoFront = 7;
const int trigRight = 8;   // <-- change if you used different pins
const int echoRight = 9;   // <-- change if you used different pins

// === ENCODERS ===
const uint8_t ENC_L = 10;
const uint8_t ENC_R = 11;

// === DRIVE CAL ===
bool LEFT_INVERT  = false;   // flip if your left wheel direction is backward
bool RIGHT_INVERT = true;    // flip if your right wheel direction is backward

// === TURN + SPEED (software duty, no hardware PWM) ===
const unsigned long TURN_PERIOD_MS  = 120;
const unsigned int  TURN_DUTY_PCT   = 70;   // sharper turn window
const bool          HARD_TURN       = true; // briefly reverse inner wheel

const unsigned long SPEED_PERIOD_MS = 60;   // speed duty cycle period
unsigned int        speedDutyPct    = 65;   // % on-time (0–100)

// === AUTON GAINS/SETPOINTS ===
// Front "Follow Me"
const float SET_CM_FRONT = 25.0;  // target 25 cm
const float KP_FRONT     = 2.5;   // tunes speed response (try 1.5–4.0)

// Right-wall follow
const float SET_CM_RIGHT = 20.0;  // target 20 cm from wall
const float KP_RIGHT     = 0.12;  // tunes steering response (bias strength)

// Dist limits
const float MAX_CM = 250.0;       // cap very large reads
const float MIN_CM = 3.0;         // ignore ultra-close echoes

// === MODE ===
enum DriveMode { MODE_STOP, MODE_DRIVE, MODE_SPIN };
enum AutonMode { AUTON_OFF, AUTON_FOLLOW_ME, AUTON_RIGHT_WALL };
DriveMode mode = MODE_STOP;
AutonMode auton = AUTON_OFF;

// Manual state (same semantics)
int driveDir = 0;   // +1 fwd, -1 back, 0 stop
int turnBias = 0;   // -1 L, 0 straight, +1 R

// Encoders (poll edges)
volatile long ticksL = 0, ticksR = 0;
int prevL = HIGH, prevR = HIGH;

// Telemetry timers
const unsigned long TELEMETRY_MS = 50;  // encoder rate
const unsigned long ULTRA_MS     = 60;  // ultrasonic rate
unsigned long lastTele = 0, lastUltra = 0;

// --- low level wheel helpers ---
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

// --- ultrasonic helpers ---
long echoTimeUS(int trig, int echo, unsigned long to_us=25000UL) {
  // 25 ms timeout ~ 4.3 m
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  return pulseIn(echo, HIGH, to_us);
}
float usToCm(long us) {
  if (us == 0) return MAX_CM + 1;           // timeout
  float cm = (us * 0.0343f) / 2.0f;         // round-trip
  if (cm < MIN_CM) cm = MIN_CM;
  if (cm > MAX_CM) cm = MAX_CM;
  return cm;
}
float readCmMedian3(int trig, int echo) {
  long a = echoTimeUS(trig, echo);
  long b = echoTimeUS(trig, echo);
  long c = echoTimeUS(trig, echo);
  // median of 3
  long m = (a > b) ? ((a < c) ? a : ((b > c) ? b : c))
                   : ((b < c) ? b : ((a > c) ? a : c));
  return usToCm(m);
}

// --- auton controllers ---
void runFollowMe(float front_cm) {
  // error > 0 => too far => go forward
  float err = SET_CM_FRONT - front_cm;
  float u   = KP_FRONT * err;    // control output (speed intent)
  // u in arbitrary units; map to speed duty and direction
  if (u > 0) { driveDir = +1; }
  else if (u < 0) { driveDir = -1; }
  else { driveDir = 0; }

  // magnitude → duty  (deadband + clamp)
  float mag = fabs(u);
  unsigned duty = (unsigned)constrain(mag * 6.0f + 20.0f, 0.0f, 100.0f);
  speedDutyPct = duty;
  mode = (driveDir == 0) ? MODE_STOP : MODE_DRIVE;
  turnBias = 0; // straight; this mode doesn’t steer
}

void runRightWall(float right_cm) {
  // Always try to go forward; steer based on error
  float err = SET_CM_RIGHT - right_cm;   // + => too far from wall
  mode = MODE_DRIVE;
  driveDir = +1;

  // bias sign: too far (err>0) -> turn right (+1); too close (err<0) -> left (-1)
  int desiredBias = (err > 0.5f) ? +1 : (err < -0.5f ? -1 : 0);

  // bias strength via duty (steer harder when farther from setpoint)
  float mag = min( fabs(err) * KP_RIGHT * 100.0f, 100.0f ); // 0–100 scaled
  // use mag to tweak the turning window: larger mag => longer bias window
  // clamp to [40..90] so it’s noticeable but not lockup
  unsigned steerDuty = (unsigned)constrain(40.0f + mag * 0.5f, 40.0f, 90.0f);

  // apply desired bias and steer duty by temporarily overriding TURN_DUTY_PCT
  turnBias = desiredBias;
  // we’ll read steerDuty inside applyDrive via a shadow global:
  currentTurnDuty = steerDuty;

  // base forward speed
  speedDutyPct = 70;
}

// shadow for per-cycle bias duty (defaults to TURN_DUTY_PCT when 0)
unsigned int currentTurnDuty = 0;

// --- main drive applier (speed duty + turn bias duty) ---
void applyDrive() {
  if (mode == MODE_STOP) { stopAll(); return; }
  if (mode == MODE_SPIN) { /* spin* already set pins */ return; }

  // speed time-slice
  unsigned long tSpeed = millis() % SPEED_PERIOD_MS;
  unsigned long onWin  = (SPEED_PERIOD_MS * speedDutyPct) / 100;
  bool speedOn = (tSpeed < onWin);

  if (!speedOn) { stopAll(); return; }

  // base direction
  if (driveDir > 0) { leftForward(); rightForward(); }
  else if (driveDir < 0) { leftBackward(); rightBackward(); }
  else { stopAll(); return; }

  // bias slice (if any)
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

  // reset per-cycle override
  currentTurnDuty = 0;
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
  // --- serial commands (manual & auton) ---
  if (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c != '\n' && c != '\r') {
      switch (c) {
        // manual
        case 'w': case 'W': auton = AUTON_OFF; driveDir = +1; mode = MODE_DRIVE; turnBias = 0; break;
        case 's': case 'S': auton = AUTON_OFF; driveDir = -1; mode = MODE_DRIVE; turnBias = 0; break;
        case 'a': case 'A': if (auton==AUTON_OFF && mode==MODE_DRIVE) turnBias = -1; break;
        case 'd': case 'D': if (auton==AUTON_OFF && mode==MODE_DRIVE) turnBias = +1; break;
        case 'q': case 'Q': auton = AUTON_OFF; mode = MODE_SPIN; turnBias = 0; spinLeft();  break;
        case 'e': case 'E': auton = AUTON_OFF; mode = MODE_SPIN; turnBias = 0; spinRight(); break;
        case ' ':           auton = AUTON_OFF; mode = MODE_STOP; driveDir = 0; turnBias = 0; stopAll(); break;

        // auton modes
        case 'F': auton = AUTON_FOLLOW_ME; break;   // front P
        case 'R': auton = AUTON_RIGHT_WALL; break;  // right-wall P
        case 'X': auton = AUTON_OFF; mode = MODE_STOP; driveDir = 0; turnBias = 0; stopAll(); break;
        default: break;
      }
    }
  }

  // --- encoders (poll edges) ---
  int rl = digitalRead(ENC_L);
  int rr = digitalRead(ENC_R);
  if (rl != prevL) { ticksL++; prevL = rl; }
  if (rr != prevR) { ticksR++; prevR = rr; }

  // --- ultrasonics (periodic) ---
  static float f_cm = 0, r_cm = 0;
  unsigned long now = millis();
  if (now - lastUltra >= ULTRA_MS) {
    lastUltra = now;
    f_cm = readCmMedian3(trigFront, echoFront);
    r_cm = readCmMedian3(trigRight, echoRight);
    // auton controllers (update speed/steer targets)
    if (auton == AUTON_FOLLOW_ME)   runFollowMe(f_cm);
    else if (auton == AUTON_RIGHT_WALL) runRightWall(r_cm);
  }

  // --- apply drive (always) ---
  applyDrive();

  // --- telemetry (encoders) ---
  if (now - lastTele >= TELEMETRY_MS) {
    lastTele = now;
    long dL = ticksL, dR = ticksR;
    ticksL = 0; ticksR = 0;
    Serial.print("T,"); Serial.print(now);
    Serial.print(',');  Serial.print(dL);
    Serial.print(',');  Serial.println(dR);
    // ultrasonic line (separate, readable by you / Processing later)
    Serial.print("U,"); Serial.print(now);
    Serial.print(',');  Serial.print((int)f_cm);
    Serial.print(',');  Serial.println((int)r_cm);
  }
}
