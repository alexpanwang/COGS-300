// Robot GUI Control Panel (Processing)

import processing.serial.*;

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

void settings() { size(680, 340); }

void setup() {
  surface.setTitle("Robot Control Panel");
  String[] ports = Serial.list();
  for (int i = 0; i < ports.length; i++) println(i + ": " + ports[i]);
  connectSerial();
  textFont(createFont("Arial", 16, true));
  textAlign(LEFT, TOP);
}

void draw() {
  background(245);

  fill(0);
  textSize(18);
  text("Robot Control Panel", 16, 12);
  textSize(14);
  fill(40);
  text("Serial: " + connectedName, 16, 40);

  int x0 = 16, y0 = 76, cardW = 380, cardH = 188, gap = 14;
  drawCard(x0, y0, cardW, cardH, "Current State", () -> {
    drawStateRow("Mode", modeLabel(), modeColor());
    drawStateRow("Drive", driveLabel(), color(20));
    drawStateRow("Turn", turnLabel(),  color(20));
    String since = nf((millis() - lastCmdMillis) / 1000.0, 0, 2) + "s ago";
    drawStateRow("Last Cmd", lastCmd + "   (" + since + ")", color(20));
  });

  int x1 = x0 + cardW + gap, y1 = y0;
  drawCard(x1, y1, 640 - x1, cardH, "Commands", () -> {
    String cmds =
      "W – Forward\n" +
      "S – Backward\n" +
      "A – Bias Left (only in DRIVE)\n" +
      "D – Bias Right (only in DRIVE)\n" +
      "Q – Spin Left\n" +
      "E – Spin Right\n" +
      "Space – STOP";
    fill(30);
    text(cmds, 12, 34);
  });

  fill(80);
  text("A/D steer only in DRIVE; others override.", 16, height - 24);
}

void keyPressed() {
  if (port == null) return;
  char c = key;
  boolean send = false;

  if (c=='a'||c=='A') { if (mode==MODE_DRIVE) { turnBias=-1; send=true; } }
  else if (c=='d'||c=='D') { if (mode==MODE_DRIVE) { turnBias=1; send=true; } }
  else {
    switch (c) {
      case 'w': case 'W': mode=MODE_DRIVE; driveDir=1;  send=true; break;
      case 's': case 'S': mode=MODE_DRIVE; driveDir=-1; send=true; break;
      case 'q': case 'Q': mode=MODE_SPIN; send=true; break;
      case 'e': case 'E': mode=MODE_SPIN; send=true; break;
      case ' ': mode=MODE_STOP; driveDir=0; turnBias=0; send=true; break;
    }
  }

  if (send) {
    port.write(c);
    port.write('\n');
    lastCmd = printableName(c);
    lastCmdMillis = millis();
  }
}

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
    port.clear();
    port.bufferUntil('\n');
    connectedName = ports[idx];
    println("Connected on " + ports[idx]);
  } catch (Exception e) {
    println("Serial open failed: " + e);
    port = null;
    connectedName = "<not connected>";
  }
}

void serialEvent(Serial p) {
  String line = p.readStringUntil('\n');
  if (line != null) println("< " + line.trim());
}

void drawCard(int x, int y, int w, int h, String title, Runnable body) {
  pushStyle();
  rectMode(CORNER);
  noStroke();
  fill(255);
  rect(x, y, w, h, 14);
  fill(30);
  textSize(16);
  text(title, x + 12, y + 10);
  pushMatrix();
  translate(x + 12, y + 10);
  lineY = 0;
  body.run();
  popMatrix();
  popStyle();
}

int lineY = 0;
void drawStateRow(String key, String val, int valColor) {
  lineY += 26;
  fill(70); textSize(14); text(key + ":", 0, lineY);
  fill(valColor); text(val, 110, lineY);
}

String modeLabel() {
  if (mode==MODE_STOP) return "STOP";
  if (mode==MODE_DRIVE) return "DRIVE";
  return "SPIN";
}

int modeColor() {
  if (mode==MODE_STOP) return color(180,40,40);
  if (mode==MODE_DRIVE) return color(30,140,60);
  return color(30,80,180);
}

String driveLabel() {
  if (driveDir>0) return "Forward";
  if (driveDir<0) return "Backward";
  return "—";
}

String turnLabel() {
  if (turnBias<0) return "Left";
  if (turnBias>0) return "Right";
  return "Straight";
}

String printableName(char c) {
  switch (c) {
    case 'w': case 'W': return "W (Forward)";
    case 's': case 'S': return "S (Backward)";
    case 'a': case 'A': return "A (Left)";
    case 'd': case 'D': return "D (Right)";
    case 'q': case 'Q': return "Q (Spin Left)";
    case 'e': case 'E': return "E (Spin Right)";
    case ' ':           return "Space (STOP)";
  }
  return str(c);
}
