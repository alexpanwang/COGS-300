// Robot GUI Control Panel + Replay + Ultrasonic Auton (Processing)
// Keys:
//   Manual: W/S (move), A/D (bias while driving), Q/E (spin), Space (stop)
//   Auton:  F (Follow-Me), R (Right-Wall), X (Exit Auton)
//   Replay: L (Load), R (Play/Restart), T (Stop)
//   Record: M (toggle manual command recording)

import processing.serial.*;
import java.io.File;

Serial port;
int    PORT_INDEX = 0;
String PORT_HINT  = null;
int    BAUD       = 115200;

// Mirror of Arduino manual drive state (for UI only)
static final int MODE_STOP  = 0;
static final int MODE_DRIVE = 1;
static final int MODE_SPIN  = 2;

int mode = MODE_STOP;
int driveDir = 0;        // +1 fwd, -1 back
int turnBias = 0;        // -1 left, +1 right

// Auton mirror (UI only)
static final int AUTON_OFF        = 0;
static final int AUTON_FOLLOW_ME  = 1;
static final int AUTON_RIGHT_WALL = 2;
int autonMode = AUTON_OFF;

String connectedName = "<not connected>";
String lastCmd = "—";
long   lastCmdMillis = 0;

PrintWriter log;            // telemetry CSV (always on)
PrintWriter cmdLog = null;  // commands CSV (manual toggle)
boolean recording = false;

int   lastDL = 0, lastDR = 0;
float hzL = 0,  hzR  = 0;

// Ultrasonic
int   frontCm = -1;
int   rightCm = -1;

// ==== Replay state ====
class ReplayCmd { int t_ms; char c; ReplayCmd(int t, char ch){ t_ms=t; c=ch; } }
ArrayList<ReplayCmd> replay = new ArrayList<ReplayCmd>();
boolean replayLoaded = false;
boolean replayPlaying = false;
int     replayIdx = 0;
int     replayFirstMs = 0;
int     replayLastMs  = 0;
long    replayStartWall = 0;
String  replayFileName = "";
boolean replaySentFinalStop = false;  // send one final STOP at end

// layout
final int MARGIN = 18;
final int GUTTER = 18;

void settings() { size(960, 640); }

void setup() {
  surface.setTitle("Robot Control Panel + Replay + Ultrasonic");
  String[] ports = Serial.list();
  for (int i = 0; i < ports.length; i++) println(i + ": " + ports[i]);
  connectSerial();

  textFont(createFont("Arial", 16, true));
  textAlign(LEFT, TOP);

  String fn = stamp();
  log = createWriter("telemetry_" + fn + ".csv");
  log.println("t_ms,dL,dR");  // encoder telemetry; (U,...) is shown live, not logged here
}

void draw() {
  background(245);

  // header
  fill(0); textSize(22); text("Robot Control Panel + Replay + Ultrasonic", MARGIN, MARGIN - 2);
  textSize(14); fill(40);
  text("Serial: " + connectedName, MARGIN, MARGIN + 26);
  fill(recording ? color(20,150,60) : color(150,60,20));
  text(recording ? "Recording: ON (M to stop)" : "Recording: OFF (M to start)", MARGIN, MARGIN + 46);

  // columns
  int colW = (width - 2*MARGIN - GUTTER) / 2;
  int leftX = MARGIN;
  int rightX = leftX + colW + GUTTER;
  int topY = MARGIN + 72;

  // Current State
  int stateH = 300;
  drawCard(leftX, topY, colW, stateH, "Current State", () -> {
    drawStateRow("Mode",     modeLabel(),  modeColor());
    drawStateRow("Auton",    autonLabel(), autonColor());
    drawStateRow("Drive",    driveLabel(), color(20));
    drawStateRow("Turn",     turnLabel(),  color(20));
    String since = nf((millis() - lastCmdMillis) / 1000.0, 0, 2) + "s ago";
    drawStateRow("Last Cmd", lastCmd + "   (" + since + ")", color(20));
    drawStateRow("Enc L/R",
      lastDL + " / " + lastDR + "  (~" + nf(hzL, 0, 1) + " / " + nf(hzR, 0, 1) + " Hz)",
      color(20));
    String fr = (frontCm>=0 ? (frontCm+" cm") : "—");
    String rr = (rightCm>=0 ? (rightCm+" cm") : "—");
    drawStateRow("Ultra F/R", fr + " / " + rr, color(20));
  });

  // Commands
  int cmdsH = 300;
  drawCard(rightX, topY, colW, cmdsH, "Commands", () -> {
    String cmds =
      "Manual:\n" +
      "  W – Forward    |  S – Backward\n" +
      "  A – Bias Left  |  D – Bias Right (only in DRIVE)\n" +
      "  Q – Spin Left  |  E – Spin Right\n" +
      "  Space – STOP\n\n" +
      "Auton (Ultrasonic):\n" +
      "  F – Follow-Me (hold set distance in front)\n" +
      "  R – Right-Wall (hold right distance while moving forward)\n" +
      "  X – Exit Auton (return to manual)\n\n" +
      "Other:\n" +
      "  M – Toggle Command Recording (creates/closes commands_*.csv)\n" +
      "Replay: L = Load  |  R = Play/Restart  |  T = Stop";
    fill(30);
    text(cmds, 12, 34, colW - 24, cmdsH - 46);
  });

  // Replay
  int replayY = topY + stateH + GUTTER;
  int replayH = height - replayY - (MARGIN + 16);
  drawCard(leftX, replayY, width - 2*MARGIN, replayH, "Replay", () -> {
    fill(30);
    String fileLine = "File: " + (replayLoaded ? replayFileName : "<none>");
    String st = replayPlaying ? "PLAYING" : "STOPPED";
    String dur = replayLoaded ? (replayLastMs - replayFirstMs) + " ms" : "—";
    text(fileLine, 12, 34);
    text("State: " + st, 12, 58);
    text("Duration: " + dur, 12, 82);
    if (replayLoaded) text("Index: " + replayIdx + " / " + replay.size(), 12, 106);

    // progress bar
    int barX = 12, barY = 136, barW = width - 2*MARGIN - 24, barH = 18;
    stroke(200); noFill(); rect(barX, barY, barW, barH, 9);
    noStroke();
    if (replayLoaded) {
      float total = max(1, (float)(replayLastMs - replayFirstMs));
      float elapsed = replayPlaying ? (millis() - replayStartWall) :
                       (replayIdx > 0 ? replayIdxTime(replayIdx-1) - replayFirstMs : 0);
      float frac = constrain(elapsed / total, 0, 1);
      fill(60,140,220);
      rect(barX, barY, barW * frac, barH, 9);
      fill(40);
      text(nf(frac*100, 0, 1) + "%", barX + 6, barY + 22);
    } else {
      fill(120); text("Load a commands_*.csv (L) to enable replay.", barX, barY + 22);
    }

    String help = "Replay sends recorded commands with original timing, then auto-stops.";
    fill(30);
    text(help, 12, 170, width - 2*MARGIN - 24, replayH - 182);
  });

  // footer
  fill(80);
  text("A/D steer only in DRIVE; others override. Auton uses ultrasonic readings from Arduino.",
       MARGIN, height - MARGIN);

  // ---- Replay loop ----
  if (replayPlaying && replayLoaded) {
    int elapsed = int(millis() - replayStartWall);
    while (replayIdx < replay.size()) {
      int target = replayIdxTime(replayIdx) - replayFirstMs; // normalized timeline
      if (elapsed >= target) {
        char cc = replay.get(replayIdx).c;
        sendCmd(cc, /*logIt=*/false);  // don't log during replay
        // update local mirrors for UI when replaying
        mirrorLocalState(cc);
        replayIdx++;
      } else break;
    }
    if (replayIdx >= replay.size()) {
      replayPlaying = false;
      if (!replaySentFinalStop) {
        sendCmd(' ', false);
        mirrorLocalState(' ');
        replaySentFinalStop = true;
      }
    }
  }
}

// ===== Input =====
void keyPressed() {
  if (key == ESC) {
    if (log != null)   { log.flush(); log.close(); }
    if (cmdLog != null){ cmdLog.flush(); cmdLog.close(); }
    exit();
  }

  // Manual command recording toggle
  if (key=='m' || key=='M') { toggleRecording(); return; }

  // Replay controls
  if (key=='l'||key=='L') { selectInput("Select commands_*.csv to replay", "replayFileSelected"); return; }
  if (key=='r'||key=='R') { startReplay(); return; }
  if (key=='t'||key=='T') { stopReplay(); return; }

  // ignore manual keys while replaying
  if (replayPlaying) return;

  if (port == null) return;
  char c = key;
  boolean send = false;

  // Manual drive
  if (c=='a'||c=='A') { if (mode==MODE_DRIVE && autonMode==AUTON_OFF) { turnBias=-1; send=true; } }
  else if (c=='d'||c=='D') { if (mode==MODE_DRIVE && autonMode==AUTON_OFF) { turnBias=1; send=true; } }
  else {
    switch (c) {
      case 'w': case 'W': mode=MODE_DRIVE; driveDir= 1; autonMode=AUTON_OFF; send=true; break;
      case 's': case 'S': mode=MODE_DRIVE; driveDir=-1; autonMode=AUTON_OFF; send=true; break;
      case 'q': case 'Q': mode=MODE_SPIN;  autonMode=AUTON_OFF; send=true; break;
      case 'e': case 'E': mode=MODE_SPIN;  autonMode=AUTON_OFF; send=true; break;
      case ' ':           mode=MODE_STOP;  driveDir=0; turnBias=0; autonMode=AUTON_OFF; send=true; break;

      // Auton keys (sent as uppercase to Arduino)
      case 'f': case 'F': autonMode = AUTON_FOLLOW_ME;  send=true; c='F'; break;
      case 'r': case 'R': autonMode = AUTON_RIGHT_WALL; send=true; c='R'; break;
      case 'x': case 'X': autonMode = AUTON_OFF; mode=MODE_STOP; driveDir=0; turnBias=0; send=true; c='X'; break;
    }
  }

  if (send) {
    sendCmd(c, /*logIt=*/true);
    mirrorLocalState(c);
  }
}

// Update local UI mirrors when we send or replay a command
void mirrorLocalState(char c) {
  switch (c) {
    case 'w': case 'W': mode=MODE_DRIVE; driveDir=+1; turnBias=0; autonMode=AUTON_OFF; break;
    case 's': case 'S': mode=MODE_DRIVE; driveDir=-1; turnBias=0; autonMode=AUTON_OFF; break;
    case 'a': case 'A': if (mode==MODE_DRIVE && autonMode==AUTON_OFF) turnBias=-1; break;
    case 'd': case 'D': if (mode==MODE_DRIVE && autonMode==AUTON_OFF) turnBias=+1; break;
    case 'q': case 'Q': mode=MODE_SPIN;  autonMode=AUTON_OFF; break;
    case 'e': case 'E': mode=MODE_SPIN;  autonMode=AUTON_OFF; break;
    case ' ':           mode=MODE_STOP;  driveDir=0; turnBias=0; autonMode=AUTON_OFF; break;

    case 'F': autonMode = AUTON_FOLLOW_ME;  break;
    case 'R': autonMode = AUTON_RIGHT_WALL; break;
    case 'X': autonMode = AUTON_OFF; mode=MODE_STOP; driveDir=0; turnBias=0; break;
  }
}

void sendCmd(char c, boolean logIt) {
  if (port != null) { port.write(c); port.write('\n'); }
  if (logIt && recording && cmdLog != null) {
    cmdLog.println(millis() + "," + c);
    cmdLog.flush();
  }
  lastCmd = printableName(c);
  lastCmdMillis = millis();
}

// ===== Recording helpers =====
void toggleRecording() {
  if (replayPlaying) { println("Recording disabled during replay."); return; }
  if (!recording) {
    String fn = stamp();
    cmdLog = createWriter("commands_" + fn + ".csv");
    cmdLog.println("t_ms,cmd");
    recording = true;
    println("Recording: ON -> commands_" + fn + ".csv");
  } else {
    if (cmdLog != null) { cmdLog.flush(); cmdLog.close(); cmdLog = null; }
    recording = false;
    println("Recording: OFF");
  }
}

// ===== Replay helpers =====
void replayFileSelected(File f) {
  if (f == null) return;
  String[] lines = loadStrings(f.getAbsolutePath());
  if (lines == null || lines.length == 0) { println("Replay: empty file"); return; }
  replay.clear();
  int startIdx = lines[0].toLowerCase().startsWith("t_ms") ? 1 : 0;
  for (int i = startIdx; i < lines.length; i++) {
    String ln = trim(lines[i]); if (ln.length()==0) continue;
    String[] parts = split(ln, ',');
    if (parts.length >= 2) {
      try {
        int t = int(trim(parts[0]));
        String s = trim(parts[1]);
        if (s.length() > 0) replay.add(new ReplayCmd(t, s.charAt(0)));
      } catch (Exception ex) { println("Replay parse error line " + i + ": " + ln); }
    }
  }
  if (replay.isEmpty()) { println("Replay: no commands parsed."); return; }
  replay.sort(new java.util.Comparator<ReplayCmd>() {
    public int compare(ReplayCmd a, ReplayCmd b) { return a.t_ms - b.t_ms; }
  });
  replayFirstMs = replay.get(0).t_ms;
  replayLastMs  = replay.get(replay.size()-1).t_ms;
  replayIdx = 0;
  replayLoaded = true;
  replayPlaying = false;
  replaySentFinalStop = false;
  replayFileName = f.getName();
  println("Replay loaded: " + replay.size() + " cmds, " + (replayLastMs - replayFirstMs) + " ms");
}

void startReplay() {
  if (!replayLoaded) { println("Replay: load a commands_*.csv first (L)."); return; }
  replayIdx = 0;
  replayPlaying = true;
  replaySentFinalStop = false;
  replayStartWall = millis();
  println("Replay started.");
  sendCmd(' ', false); // optional: stop robot before starting
}

void stopReplay() {
  if (!replayPlaying) return;
  replayPlaying = false;
  replayIdx = 0;
  replaySentFinalStop = false;
  println("Replay stopped.");
  sendCmd(' ', false);
}

int replayIdxTime(int idx) { return replay.get(idx).t_ms; }

// ===== Serial =====
void connectSerial() {
  String[] ports = Serial.list();
  if (ports.length == 0) { println("No serial ports."); return; }
  int idx = PORT_INDEX;
  if (PORT_HINT != null) {
    for (int i=0;i<ports.length;i++) {
      if (ports[i].toLowerCase().contains(PORT_HINT.toLowerCase())) { idx=i; break; }
    }
  }
  try {
    port = new Serial(this, ports[idx], BAUD);
    port.clear(); port.bufferUntil('\n');
    connectedName = ports[idx];
    println("Connected on " + ports[idx]);
  } catch (Exception e) {
    println("Serial open failed: " + e);
    port = null; connectedName = "<not connected>";
  }
}

void serialEvent(Serial p) {
  String line = p.readStringUntil('\n'); if (line == null) return;
  line = trim(line); if (line.length() == 0) return;

  if (line.startsWith("T,")) {             // T,<t_ms>,<dL>,<dR>
    String[] parts = split(line, ',');
    if (parts.length == 4) {
      int tms = int(parts[1]);
      lastDL = int(parts[2]); lastDR = int(parts[3]);
      hzL = lastDL * (1000.0/50.0); hzR = lastDR * (1000.0/50.0); // matches Arduino 50 ms window
      if (log != null) { log.println(tms + "," + lastDL + "," + lastDR); log.flush(); }
    }
  } else if (line.startsWith("U,")) {      // U,<t_ms>,<front_cm>,<right_cm>
    String[] parts = split(line, ',');
    if (parts.length == 4) {
      // int tms = int(parts[1]); // not used in UI
      frontCm = parseIntSafe(parts[2], -1);
      rightCm = parseIntSafe(parts[3], -1);
    }
  } else {
    println("< " + line);
  }
}

// ===== UI helpers =====
void drawCard(int x, int y, int w, int h, String title, Runnable body) {
  pushStyle();
  rectMode(CORNER); noStroke(); fill(255); rect(x, y, w, h, 14);
  fill(30); textSize(16); text(title, x + 12, y + 10);
  pushMatrix(); translate(x + 12, y + 10); lineY = 0; body.run(); popMatrix();
  popStyle();
}
int lineY = 0;
void drawStateRow(String key, String val, int valColor) {
  lineY += 28;
  fill(70); textSize(14); text(key + ":", 0, lineY);
  fill(valColor); text(val, 150, lineY);
}
String modeLabel() { if (mode==MODE_STOP) return "STOP"; if (mode==MODE_DRIVE) return "DRIVE"; return "SPIN"; }
int modeColor() { if (mode==MODE_STOP) return color(180,40,40); if (mode==MODE_DRIVE) return color(30,140,60); return color(30,80,180); }
String autonLabel() {
  if (autonMode == AUTON_OFF) return "OFF";
  if (autonMode == AUTON_FOLLOW_ME) return "Follow-Me";
  return "Right-Wall";
}
int autonColor() {
  if (autonMode == AUTON_OFF) return color(120);
  if (autonMode == AUTON_FOLLOW_ME) return color(30,120,180);
  return color(180,120,30);
}
String driveLabel() { if (driveDir>0) return "Forward"; if (driveDir<0) return "Backward"; return "—"; }
String turnLabel() { if (turnBias<0) return "Left"; if (turnBias>0) return "Right"; return "Straight"; }
String printableName(char c) {
  switch (c) {
    case 'w': case 'W': return "W (Forward)";
    case 's': case 'S': return "S (Backward)";
    case 'a': case 'A': return "A (Left)";
    case 'd': case 'D': return "D (Right)";
    case 'q': case 'Q': return "Q (Spin Left)";
    case 'e': case 'E': return "E (Spin Right)";
    case ' ':           return "Space (STOP)";
    case 'F':           return "F (Follow-Me)";
    case 'R':           return "R (Right-Wall)";
    case 'X':           return "X (Exit Auton)";
  } return str(c);
}

// ===== Misc =====
String stamp() { return nf(year(),4)+nf(month(),2)+nf(day(),2)+"_"+nf(hour(),2)+nf(minute(),2)+nf(second(),2); }
int parseIntSafe(String s, int def) { try { return Integer.parseInt(trim(s)); } catch (Exception e) { return def; } }
