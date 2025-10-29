import processing.serial.*;

Serial myPort;
String data = "";

float angle = 0;
float frontDist = 0;
float rightDist = 0;

void setup() {
  size(600, 600);
  println(Serial.list());
  // Change [0] to match your Arduino port after checking the console list
  myPort = new Serial(this, Serial.list()[0], 9600);
  myPort.bufferUntil('\n');
  smooth();
}

void draw() {
  background(10, 15, 25);

  translate(width/2, height/2);
  noFill();
  stroke(0, 255, 0, 80);
  strokeWeight(1);
  
  // Radar rings
  for (int r = 100; r <= 250; r += 50) {
    ellipse(0, 0, r*2, r*2);
  }

  // Radar lines
  for (int a = 0; a < 180; a += 30) {
    float x = cos(radians(a)) * 250;
    float y = -sin(radians(a)) * 250;
    line(0, 0, x, y);
  }

  // Rotating sweep line
  stroke(0, 255, 0);
  strokeWeight(2);
  float sweepX = cos(radians(angle)) * 250;
  float sweepY = -sin(radians(angle)) * 250;
  line(0, 0, sweepX, sweepY);

  // Detected Front distance (green blip)
  if (frontDist > 0 && frontDist < 250) {
    float frontX = cos(radians(angle)) * frontDist;
    float frontY = -sin(radians(angle)) * frontDist;
    noStroke();
    fill(0, 255, 0);
    ellipse(frontX, frontY, 10, 10);
  }

  // Detected Right distance (cyan blip)
  if (rightDist > 0 && rightDist < 250) {
    float rightAngle = angle + 30; // offset visualization
    float rightX = cos(radians(rightAngle)) * rightDist;
    float rightY = -sin(radians(rightAngle)) * rightDist;
    noStroke();
    fill(0, 255, 255);
    ellipse(rightX, rightY, 10, 10);
  }

  // HUD info
  fill(0, 255, 0);
  textAlign(LEFT);
  textSize(14);
  text("Angle: " + angle + "°", -width/2 + 20, height/2 - 60);
  text("Front: " + nf(frontDist, 0, 1) + " cm", -width/2 + 20, height/2 - 40);
  text("Right: " + nf(rightDist, 0, 1) + " cm", -width/2 + 20, height/2 - 20);
  text("Arduino Radar Visualization", width/2 - 200, height/2 - 20);
}

void serialEvent(Serial myPort) {
  data = myPort.readStringUntil('\n');
  if (data != null) {
    data = trim(data);
    // Expected format: "Angle: 90 | Front: 45.2 cm | Right: 60.8"
    try {
      String[] parts = split(data, '|');
      if (parts.length == 3) {
        angle = float(trim(split(parts[0], ':')[1]));
        frontDist = float(trim(split(split(parts[1], ':')[1], ' ')[0]));
        rightDist = float(trim(split(split(parts[2], ':')[1], ' ')[0]));
      }
    } 
    catch (Exception e) {
      println("Parse error: " + data);
    }
  }
}
