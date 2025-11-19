// ============ Line Follower: center + side corrections via TURN_MULT ============
// Motors
const int IN1 = 6;  // Left forward
const int IN2 = 3;  // Left backward
const int IN3 = 9;  // Right forward
const int IN4 = 5;  // Right backward

// IR sensors (digital)
const int IR_C = 2; // Center
const int IR_L = 7; // Left
const int IR_R = 4; // Right

// IR polarity: if your boards are HIGH on white, set true; if LOW on white, set false.
bool IR_HIGH_MEANS_WHITE = false;   // change to true if needed after testing

// Motor inversion (flip if that side runs backward when you expect forward)
bool LEFT_INVERT  = false;
bool RIGHT_INVERT = false;

// Software PWM settings (speed control)
const unsigned long SPEED_PERIOD_MS = 50;
unsigned int BASE_DUTY   = 30;      // 0..100 (overall speed)

// Turn multiplier: >1.0 = stronger correction
const float TURN_MULT = 1.5;

// --- motor helpers ---
void leftSet(bool fwd){
  if (LEFT_INVERT) fwd = !fwd;
  digitalWrite(IN1, fwd ? HIGH : LOW);
  digitalWrite(IN2, fwd ? LOW  : HIGH);
}
void rightSet(bool fwd){
  if (RIGHT_INVERT) fwd = !fwd;
  digitalWrite(IN3, fwd ? HIGH : LOW);
  digitalWrite(IN4, fwd ? LOW  : HIGH);
}

void leftForward()  { leftSet(true);  }
void leftBackward() { leftSet(false); }
void rightForward() { rightSet(true); }
void rightBackward(){ rightSet(false);}
void leftStop()     { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); }
void rightStop()    { digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); }
void stopAll()      { leftStop(); rightStop(); }

// --- IR read: true = on white ---
inline bool onWhiteRaw(int pin){
  int v = digitalRead(pin);
  return IR_HIGH_MEANS_WHITE ? (v == HIGH) : (v == LOW);
}

// Simple debounced states
uint8_t cHit=0, lHit=0, rHit=0;
bool C=false, L=false, R=false;
const uint8_t HIT_PERSIST = 2;

void updateSensors(){
  if (onWhiteRaw(IR_C)) { if (cHit < 255) cHit++; } else cHit = 0;
  if (onWhiteRaw(IR_L)) { if (lHit < 255) lHit++; } else lHit = 0;
  if (onWhiteRaw(IR_R)) { if (rHit < 255) rHit++; } else rHit = 0;

  C = (cHit >= HIT_PERSIST);
  L = (lHit >= HIT_PERSIST);
  R = (rHit >= HIT_PERSIST);
}

// --- main drive step using TURN_MULT everywhere + backward rule ---
void driveStep(){
  updateSensors();

  float leftDuty  = 0;
  float rightDuty = 0;
  bool goingBackward = false;   // NEW: track direction

  // ========== Center OFF (C == false): strong corrections ==========
  if (!C) {
    if (L && !R) {
      // ONLY LEFT sensor: turn LEFT
      // (we want the robot to pivot left)
      leftDuty  = BASE_DUTY / TURN_MULT;
      rightDuty = 0;
    } 
    else if (R && !L) {
      // ONLY RIGHT sensor: turn RIGHT
      rightDuty = BASE_DUTY / TURN_MULT;
      leftDuty  = 0;
    } 
    else if (L && R) {
      // Both sides but no center: go straight, slower
      leftDuty  = BASE_DUTY / TURN_MULT;
      rightDuty = BASE_DUTY / TURN_MULT;
    } 
    else {
      // NEW RULE: no sensors see anything → back up
      leftDuty      = BASE_DUTY / TURN_MULT;
      rightDuty     = BASE_DUTY / TURN_MULT;
      goingBackward = true;
    }
  }
  // ========== Center ON (C == true): soft corrections ==========
  else {
    if (!L && !R) {
      // Center only → straight at base speed
      leftDuty  = BASE_DUTY;
      rightDuty = BASE_DUTY;
    }
    else if (L && !R) {
      // Center + LEFT → soft right correction
      // drifted left, so turn right:
      leftDuty  = BASE_DUTY * TURN_MULT;    // speed up left
      rightDuty = BASE_DUTY / TURN_MULT;    // slow right
    }
    else if (R && !L) {
      // Center + RIGHT → soft left correction
      leftDuty  = BASE_DUTY / TURN_MULT;    // slow left
      rightDuty = BASE_DUTY * TURN_MULT;    // speed up right
    }
    else { // L && R && C
      // Center + both sides: treat as wide line / crossing → straight
      leftDuty  = BASE_DUTY;
      rightDuty = BASE_DUTY;
    }
  }

  // Clamp duties to 0..100
  if (leftDuty  > 100) leftDuty  = 100;
  if (rightDuty > 100) rightDuty = 100;
  if (leftDuty  < 0)   leftDuty  = 0;
  if (rightDuty < 0)   rightDuty = 0;

  // Software PWM per wheel
  unsigned long ts = millis() % SPEED_PERIOD_MS;
  unsigned long leftOn  = (unsigned long)(SPEED_PERIOD_MS * (leftDuty  / 100.0));
  unsigned long rightOn = (unsigned long)(SPEED_PERIOD_MS * (rightDuty / 100.0));

  if (ts < leftOn) {
    if (goingBackward) leftBackward();
    else               leftForward();
  } else {
    leftStop();
  }

  if (ts < rightOn) {
    if (goingBackward) rightBackward();
    else               rightForward();
  } else {
    rightStop();
  }
}

void setup(){
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(IR_C, INPUT);
  pinMode(IR_L, INPUT);
  pinMode(IR_R, INPUT);

  stopAll();
  Serial.begin(115200);
}

void loop(){
  driveStep();
}





