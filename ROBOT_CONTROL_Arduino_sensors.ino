// ================= Line Following per Rules (3 IR sensors) =================
// Wiring
const int IN1 = 6;  // Left forward
const int IN2 = 3;  // Left backward
const int IN3 = 8;  // Right forward
const int IN4 = 5;  // Right backward

const int IR_C = 2; // Center IR (digital OUT)
const int IR_L = 7; // Left IR
const int IR_R = 4; // Right IR

// Most IR boards: HIGH on white, LOW on dark. Flip to false if yours is opposite.
bool IR_HIGH_MEANS_WHITE = true;

// Motor inversion (set true if a side runs opposite)
bool LEFT_INVERT  = false;
bool RIGHT_INVERT = true;  // keep as before; flip to false if your right side is correct

// Speed / steering shaping
const unsigned long SPEED_PERIOD_MS = 50; // software PWM period
unsigned int BASE_DUTY   = 65;            // 55–75 works well
const unsigned long TURN_PERIOD_MS = 120; // steering slice period
unsigned int TURN_WIN_PCT = 65;           // slice window (larger = sharper)
const bool HARD_TURN = false;             // false=stop inner wheel, true=reverse inner (sharper)

// Debounce for sensor “hits”
const uint8_t HIT_PERSIST = 2;

// ---------- Motor helpers ----------
void leftSet(bool fwd){
  if (LEFT_INVERT) fwd = !fwd;
  if (fwd){ digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW); }
  else    { digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH); }
}
void rightSet(bool fwd){
  if (RIGHT_INVERT) fwd = !fwd;
  if (fwd){ digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW); }
  else    { digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH); }
}
void leftForward(){ leftSet(true); }
void leftBackward(){ leftSet(false); }
void rightForward(){ rightSet(true); }
void rightBackward(){ rightSet(false); }
void leftStop(){ digitalWrite(IN1,LOW); digitalWrite(IN2,LOW); }
void rightStop(){ digitalWrite(IN3,LOW); digitalWrite(IN4,LOW); }
void stopAll(){ leftStop(); rightStop(); }

// ---------- IR reads (true = on WHITE) ----------
inline bool onWhiteRaw(int pin){
  int v = digitalRead(pin);
  return IR_HIGH_MEANS_WHITE ? (v==HIGH) : (v==LOW);
}
inline bool onC(){ return onWhiteRaw(IR_C); }
inline bool onL(){ return onWhiteRaw(IR_L); }
inline bool onR(){ return onWhiteRaw(IR_R); }

// Simple persistence (debounce)
uint8_t cHit=0,lHit=0,rHit=0;
bool C=false,L=false,R=false;

void updateSensors(){
  C = onC() ? (cHit=min<uint8_t>(255, cHit+1), true) : (cHit=0, false);
  L = onL() ? (lHit=min<uint8_t>(255, lHit+1), true) : (lHit=0, false);
  R = onR() ? (rHit=min<uint8_t>(255, rHit+1), true) : (rHit=0, false);
  // require small persistence to count as “on”
  C = (cHit >= HIT_PERSIST);
  L = (lHit >= HIT_PERSIST);
  R = (rHit >= HIT_PERSIST);
}

// ---------- Drive forward with optional steer bias ----------
void driveForwardSliced(){
  unsigned long ts = millis() % SPEED_PERIOD_MS;
  unsigned long on = (SPEED_PERIOD_MS * BASE_DUTY)/100;
  if (ts <= on){ leftForward(); rightForward(); }
  else          { stopAll(); }
}

// dir: -1 = steer left, +1 = steer right, 0 = straight
void steerBias(int dir){
  if (dir == 0) return;
  unsigned long tt = millis() % TURN_PERIOD_MS;
  unsigned long win = (TURN_PERIOD_MS * TURN_WIN_PCT)/100;
  if (tt < win){
    bool innerLeft = (dir < 0);
    if (HARD_TURN){
      if (innerLeft) leftBackward(); else rightBackward();
    } else {
      if (innerLeft) leftStop(); else rightStop();
    }
  }
}

// ---------- Main behavior (your rules) ----------
// 1) Move along white line on grey floor.
// 1.2) Keep center sensor on white; “straighten” if centered.
// 1.3) If RIGHT sees white, steer RIGHT.
// 1.4) If LEFT sees white,  steer LEFT.
// Priority: side sensors override; center keeps straight; if none see white, keep last correction briefly.

int lastDir = 0; // memory when no sensor sees white

void followRulesStep(){
  updateSensors();

  // Base motion = forward
  driveForwardSliced();

  // Side sensors override: if R sees white → steer right; if L → steer left
  if (R && !L){ steerBias(+1); lastDir=+1; return; }
  if (L && !R){ steerBias(-1); lastDir=-1; return; }

  // If centered on white and sides don’t see white → go straight
  if (C && !L && !R){ steerBias(0); lastDir=0; return; }

  // If both sides see white (wide/intersection), prefer straight
  if (L && R){ steerBias(0); lastDir=0; return; }

  // If no sensor currently sees white, nudge toward last known side briefly
  steerBias(lastDir);
}

void setup(){
  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);

  pinMode(IR_C,INPUT); pinMode(IR_L,INPUT); pinMode(IR_R,INPUT);

  stopAll();
  Serial.begin(115200);

  // Quick note: if turning directions feel inverted:
  //  - swap LEFT_INVERT/RIGHT_INVERT booleans, or
  //  - swap IR_HIGH_MEANS_WHITE if your boards read LOW on white.
}

void loop(){
  followRulesStep();
}
