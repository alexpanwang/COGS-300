// Robot GUI Control Panel + Replay (Processing) — roomy layout

import processing.serial.*;
import java.io.File;

Serial port;
int    PORT_INDEX = 0;
String PORT_HINT  = null;
int    BAUD       = 115200;

static final int MODE_STOP  = 0;
static final int MODE_DRIVE = 1;
static final int MODE_SPIN  = 2;

int mode = MODE_STOP;
int driveDir = 0;    // +1 fwd, -1 back
int turnBias = 0;    // -1 left, +1 right

String connectedName = "<not connected>";
String lastCmd = "—";
long   lastCmdMillis = 0;

PrintWriter log;     // telemetry CSV
PrintWriter cmdLog;  // commands CSV
int   lastDL = 0, lastDR = 0;
float hzL = 0,  hzR  = 0;

// ==== Replay state ====
class ReplayCmd {
  int t_ms;  char c;
  ReplayCmd(int t, char ch) { t_ms = t; c = ch; }
}
ArrayList<ReplayCmd> replay = new ArrayList<ReplayCmd>();
boolean replayLoaded = false;
boolean replayPlaying = false;
boolean replayPaused  = false;
int     replayIdx = 0;
int     replayFirstMs = 0;
int     replayLastMs  = 0;
long    replayStartWall = 0;
String  replayFileName = "";

// layout
final int MARGIN = 18;
final int GUTTER = 18;

void settings() { size(960, 640); }  // bigger window

void setup() {
  surface.setTitle("Robot Control Panel + Replay");
  String[] ports = Serial.list();
  for (int i = 0; i < ports.length; i++) println(i + ": " + ports[i]);
  connectSerial();

  textFont(createFont("Arial", 16, true));
  textAlign(LEFT, TOP);

  String fn = nf(year(), 4) + nf(month(), 2) + nf(day(), 2) + "_" + nf(hour(), 2) + nf(minute(), 2) + nf(second(), 2);
  log = createWriter("telemetry_" + fn + ".csv");
  log.println("t_ms,dL,dR");
  cmdLog = createWriter("commands_" + fn + ".csv");
  cmdLog.println("t_ms,cmd");
}

void draw() {
  background(245);

  // header
  fill(0); textSize(22); text("Robot Control Panel + Replay", MARGIN, MARGIN - 2);
  textSize(14); fill(40); text("Serial: " + connectedName, MARGIN, MARGIN + 26);

  // column widths
  int colW = (width - 2*MARGIN - GUTTER) / 2;
  int leftX = MARGIN;
  int rightX = leftX + colW + GUTTER;
  int topY = MARGIN + 56;

  // Current State (left, fixed height)
  int stateH = 260;
  drawCard(leftX, topY, colW, stateH, "Current State", () -> {
    drawStateRow("Mode",  modeLabel(),  modeColor());
    drawStateRow("Drive", driveLabel(), color(20));
    drawStateRow("Turn",  turnLabel(),  color(20));

    String since = nf((millis() - lastCmdMillis) / 1000.0, 0, 2) + "s ago";
    drawStateRow("Last Cmd", lastCmd + "   (" + since + ")", color(20));
    drawStateRow("Enc L/R",
      lastDL + " / " + lastDR + "  (~" + nf(hzL, 0, 1) + " / " + nf(hzR, 0, 1) + " Hz)",
      color(20));
  });

  // Commands (right, wrapped text)
  int cmdsH = 260;
  drawCard(rightX, topY, colW, cmdsH, "Commands", () -> {
    String cmds =
      "W – Forward\n" +
      "S – Backward\n" +
      "A – Bias Left (only in DRIVE)\n" +
      "D – Bias Right (only in DRIVE)\n" +
      "Q – Spin Left\n" +
      "E – Spin Right\n" +
      "Space – STOP";
    fill(30);
    // wrap to card width (inner padding 12*2)
    text(cmds, 12, 34, colW - 24, cmdsH - 46);
  });

  // Replay (full width under the two top cards)
  int replayY = topY + stateH + GUTTER;
  int replayH = height - replayY - (MARGIN + 16);
  drawCard(leftX, replayY, width - 2*MARGIN, replayH, "Replay", () -> {
    fill(30);
    String fileLine = "File: " + (replayLoaded ? replayFileName : "<none>");
    String st = replayPlaying ? (replayPaused ? "PAUSED" : "PLAYING") : "STOPPED";
    String dur = replayLoaded ? (replayLastMs - replayFirstMs) + " ms" : "—";
    int played = replayPlaying && !replayPaused ? int(millis() - replayStartWall) : 0;

    text(fileLine, 12, 34);
    text("State: " + st, 12, 58);
    text("Duration: " + dur, 12, 82);
    if (replayLoaded) text("Index: " + replayIdx + " / " + replay.size(), 12, 106);

    String help = "Keys: L = Load  |  R = Play/Restart  |  P = Pause/Resume  |  T = Stop\n"
                + "Note: While replaying, manual drive keys are disabled.";
    text(help, 12, 132, width - 2*MARGIN - 24, replayH - 144); // wrapped
  });

  // footer
  fill(80);
  text("A/D steer only in DRIVE; others override. Replay reproduces recorded timing.",
       MARGIN, height - MARGIN);
  
  // ---- Replay loop ----
  if (replayPlaying && replayLoaded && !replayPaused) {
    int elapsed = int(millis() - replayStartWall);
    while (replayIdx < replay.size()) {
      int target = replayIdxTime(replayIdx) - replayFirstMs;
      if (target <= elapsed) {
        char cc = replay.get(replayIdx).c;
        sendCmd(cc, /*logIt=*/false);
        replayIdx++;
      } else break;
    }
    if (replayIdx >= replay.size()) {
      replayPlaying = false;
      sendCmd(' ', false);
    }
  }
}

// ===== Input =====
void keyPressed() {
  if (key == ESC) {
    if (log != null)   { log.flush();   log.close(); }
    if (cmdLog != null){ cmdLog.flush(); cmdLog.close(); }
    exit();
  }

  // replay controls
  if (key=='l'||key=='L') { selectInput("Select commands_*.csv to replay", "replayFileSelected"); return; }
  if (key=='r'||key=='R') { startReplay(); return; }
  if (key=='p'||key=='P') { togglePauseReplay(); return; }
  if (key=='t'||key=='T') { stopReplay(); return; }

  // ignore drive keys during replay
  if (replayPlaying && !replayPaused) return;

  if (port == null) return;
  char c = key;
  boolean send = false;

  if (c=='a'||c=='A') { if (mode==MODE_DRIVE) { turnBias=-1; send=true; } }
  else if (c=='d'||c=='D') { if (mode==MODE_DRIVE) { turnBias=1; send=true; } }
  else {
    switch (c) {
      case 'w': case 'W': mode=MODE_DRIVE; driveDir= 1; send=true; break;
      case 's': case 'S': mode=MODE_DRIVE; driveDir=-1; send=true; break;
      case 'q': case 'Q': mode=MODE_SPIN;  send=true; break;
      case 'e': case 'E': mode=MODE_SPIN;  send=true; break;
      case ' ':           mode=MODE_STOP;  driveDir=0; turnBias=0; send=true; break;
    }
  }
  if (send) sendCmd(c, /*logIt=*/true);
}

void sendCmd(char c, boolean logIt) {
  if (port != null) { port.write(c); port.write('\n'); }
  if (logIt && cmdLog != null) { cmdLog.println(millis() + "," + c); cmdLog.flush(); }
  lastCmd = printableName(c);
  lastCmdMillis = millis();
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
  replayIdx = 0; replayLoaded = true; replayPlaying = false; replayPaused = false;
  replayFileName = f.getName();
  println("Replay loaded: " + replay.size() + " cmds, " + (replayLastMs - replayFirstMs) + " ms");
}

void startReplay() {
  if (!replayLoaded) { println("Replay: load a commands_*.csv first (L)."); return; }
  replayIdx = 0; replayPlaying = true; replayPaused = false; replayStartWall = millis();
  sendCmd(' ', false); // stop robot before starting
}
void togglePauseReplay() {
  if (!replayPlaying) return;
  if (!replayPaused) { replayPaused = true; println("Replay paused."); }
  else { replayPaused = false;
         replayStartWall = millis() - (replayIdxTime(replayIdx) - replayFirstMs);
         println("Replay resumed.");
  }
}
void stopReplay() {
  if (!replayPlaying && !replayPaused) return;
  replayPlaying = false; replayPaused = false; replayIdx = 0;
  println("Replay stopped."); sendCmd(' ', false);
}
int replayIdxTime(int idx) { return replay.get(idx).t_ms; }

// ===== Serial =====
void connectSerial() {
  String[] ports = Serial.list();
  if (ports.length == 0) { println("No serial ports."); return; }
  int idx = PORT_INDEX;
  if (PORT_HINT != null) for (int i=0;i<ports.length;i++)
    if (ports[i].toLowerCase().contains(PORT_HINT.toLowerCase())) { idx=i; break; }
  try {
    port = new Serial(this, ports[idx], BAUD);
    port.clear(); port.bufferUntil('\n');
    connectedName = ports[idx]; println("Connected on " + ports[idx]);
  } catch (Exception e) {
    println("Serial open failed: " + e);
    port = null; connectedName = "<not connected>";
  }
}
void serialEvent(Serial p) {
  String line = p.readStringUntil('\n'); if (line == null) return;
  line = trim(line); if (line.length() == 0) return;
  if (line.startsWith("T,")) {
    String[] parts = split(line, ',');
    if (parts.length == 4) {
      int tms = int(parts[1]);
      lastDL = int(parts[2]); lastDR = int(parts[3]);
      hzL = lastDL * (1000.0/50.0); hzR = lastDR * (1000.0/50.0);
      if (log != null) { log.println(tms + "," + lastDL + "," + lastDR); log.flush(); }
    }
  } else println("< " + line);
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
  fill(valColor); text(val, 130, lineY);
}
String modeLabel() { if (mode==MODE_STOP) return "STOP"; if (mode==MODE_DRIVE) return "DRIVE"; return "SPIN"; }
int modeColor() { if (mode==MODE_STOP) return color(180,40,40); if (mode==MODE_DRIVE) return color(30,140,60); return color(30,80,180); }
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
  } return str(c);
}

