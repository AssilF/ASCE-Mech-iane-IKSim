import processing.event.MouseEvent;

// Processing sketch that visualises a simplified 3D model of the Mech'iane arm
// and solves inverse kinematics to follow an interactive XYZ target point.
ArmKinematics arm;
PVector desiredTarget;

float camYaw = radians(45);
float camPitch = radians(25);
float camDistance = 900;

boolean rotatingCamera = false;
int lastMouseX;
int lastMouseY;
boolean elbowUpSolution = false;

void settings() {
  size(1280, 800, P3D);
}

void setup() {
  surface.setTitle("Mech'iane IK Simulation");
  smooth(8);
  sphereDetail(22);
  arm = new ArmKinematics();
  desiredTarget = arm.target.copy();
  hint(ENABLE_DEPTH_TEST);
}

void draw() {
  background(12);
  setLights();
  applyCamera();
  arm.solve(desiredTarget, elbowUpSolution);
  drawScene();
  drawHUD();
}

void setLights() {
  ambientLight(60, 60, 70);
  directionalLight(220, 210, 200, -0.3, -0.8, -0.2);
  directionalLight(120, 130, 180, 0.2, 0.4, 0.8);
}

void applyCamera() {
  PVector eye = new PVector(
    camDistance * cos(camPitch) * sin(camYaw),
    camDistance * sin(camPitch),
    camDistance * cos(camPitch) * cos(camYaw)
  );
  PVector center = new PVector(0, arm.basePos.y, 0);
  PVector up = new PVector(0, 1, 0);

  PVector eyeP = mechToP3D(eye);
  PVector centerP = mechToP3D(center);
  PVector upP = mechToP3D(up);
  perspective(radians(55), width / (float) height, 10, 8000);
  camera(eyeP.x, eyeP.y, eyeP.z, centerP.x, centerP.y, centerP.z, upP.x, upP.y, upP.z);
}

void drawScene() {
  drawGroundGrid(80, 12, 0);
  drawAxes(120);
  arm.drawArm();
  drawTargetMarker();
}

void drawTargetMarker() {
  stroke(255, 80, 140, 180);
  strokeWeight(2.2);
  drawLine3d(desiredTarget, arm.reachedTarget);
  noStroke();
  fill(255, 90, 150, 160);
  drawSphere(desiredTarget, 12);
  fill(90, 240, 190);
  drawSphere(arm.reachedTarget, 10);
}

void drawGroundGrid(float spacing, int count, float y) {
  stroke(60, 70, 80);
  strokeWeight(1);
  for (int i = -count; i <= count; i++) {
    float x = i * spacing;
    PVector a = new PVector(x, y, -count * spacing);
    PVector b = new PVector(x, y, count * spacing);
    drawLine3d(a, b);
    PVector c = new PVector(-count * spacing, y, x);
    PVector d = new PVector(count * spacing, y, x);
    drawLine3d(c, d);
  }
}

void drawAxes(float length) {
  strokeWeight(2.4);
  // X axis (red)
  stroke(220, 70, 70);
  drawLine3d(new PVector(0, 0, 0), new PVector(length, 0, 0));
  // Y axis (green)
  stroke(70, 200, 120);
  drawLine3d(new PVector(0, 0, 0), new PVector(0, length, 0));
  // Z axis (blue)
  stroke(70, 120, 220);
  drawLine3d(new PVector(0, 0, 0), new PVector(0, 0, length));
}

void drawHUD() {
  hint(DISABLE_DEPTH_TEST);
  camera();
  resetMatrix();
  fill(255);
  textAlign(LEFT, TOP);
  textSize(14);
  String reachState = arm.status.reachLimited ? "reach limited" : (arm.status.baseLimited ? "base limited" : "ok");
  if (arm.status.minDistanceApplied) {
    reachState += " - min-dist";
  }
  if (arm.status.radialAdjusted) {
    reachState += " - radial clamp";
  }
  text("Target (mm)  X:" + nf(desiredTarget.x, 1, 1)
       + "  Y:" + nf(desiredTarget.y, 1, 1)
       + "  Z:" + nf(desiredTarget.z, 1, 1), 16, 18);
  text("End effector  X:" + nf(arm.reachedTarget.x, 1, 1)
       + "  Y:" + nf(arm.reachedTarget.y, 1, 1)
       + "  Z:" + nf(arm.reachedTarget.z, 1, 1), 16, 36);
  text("Position error: " + nf(arm.status.distanceError, 1, 2) + " mm", 16, 54);
  text("Base yaw:" + nf(degrees(arm.baseAngle), 1, 1) + " deg  Shoulder:" + nf(degrees(arm.shoulderAngle), 1, 1)
       + " deg  Elbow:" + nf(degrees(arm.elbowAngle), 1, 1) + " deg  Extension:" + nf(arm.extension, 1, 1) + " mm", 16, 72);
  text("Status: " + reachState, 16, 90);

  textAlign(RIGHT, TOP);
  text("Controls:\nW/S: target up/down\nA/D: target left/right\nQ/E: target toward/away\nR: reset target  -  SPACE: toggle elbow posture\nMouse drag: orbit camera  -  Wheel: zoom\nHold SHIFT for fine steps", width - 22, height - 118);
  hint(ENABLE_DEPTH_TEST);
}

void keyPressed() {
  float step = (keyEvent != null && keyEvent.isShiftDown()) ? 5 : 20;
  if (key == 'w' || key == 'W') {
    desiredTarget.y += step;
  } else if (key == 's' || key == 'S') {
    desiredTarget.y -= step;
  } else if (key == 'a' || key == 'A') {
    desiredTarget.x -= step;
  } else if (key == 'd' || key == 'D') {
    desiredTarget.x += step;
  } else if (key == 'q' || key == 'Q') {
    desiredTarget.z -= step;
  } else if (key == 'e' || key == 'E') {
    desiredTarget.z += step;
  } else if (key == 'r' || key == 'R') {
    desiredTarget.set(arm.homeTarget());
  } else if (key == ' ') {
    elbowUpSolution = !elbowUpSolution;
  }
  clampTarget();
}

void mousePressed() {
  if (mouseButton == LEFT) {
    rotatingCamera = true;
    lastMouseX = mouseX;
    lastMouseY = mouseY;
  }
}

void mouseDragged() {
  if (rotatingCamera) {
    float sensitivity = 0.01;
    camYaw += (mouseX - lastMouseX) * sensitivity;
    camPitch -= (mouseY - lastMouseY) * sensitivity;
    camPitch = constrain(camPitch, radians(-5), radians(85));
    lastMouseX = mouseX;
    lastMouseY = mouseY;
  }
}

void mouseReleased() {
  rotatingCamera = false;
}

void mouseWheel(MouseEvent event) {
  camDistance += event.getCount() * 40;
  camDistance = constrain(camDistance, 320, 2200);
}

void clampTarget() {
  float horizontal = 320;
  desiredTarget.x = constrain(desiredTarget.x, -horizontal, horizontal);
  desiredTarget.z = constrain(desiredTarget.z, -horizontal, horizontal);
  desiredTarget.y = constrain(desiredTarget.y, 40, arm.basePos.y + 280);
}

PVector mechToP3D(PVector v) {
  return new PVector(v.x, -v.y, v.z);
}

void drawLine3d(PVector a, PVector b) {
  PVector pa = mechToP3D(a);
  PVector pb = mechToP3D(b);
  line(pa.x, pa.y, pa.z, pb.x, pb.y, pb.z);
}

void drawSphere(PVector center, float radius) {
  PVector c = mechToP3D(center);
  pushMatrix();
  translate(c.x, c.y, c.z);
  sphere(radius);
  popMatrix();
}

void drawBoxCentered(PVector center, float w, float h, float d) {
  PVector c = mechToP3D(center);
  pushMatrix();
  translate(c.x, c.y, c.z);
  box(w, h, d);
  popMatrix();
}

class ArmKinematics {
  final float baseHeight = 90;
  final float baseRadius = 60;
  final float shoulderLength = 140;
  final float elbowLength = 110;
  final float extensionMax = 110;
  final float baseYawLimit = radians(135);
  final float shoulderMin = radians(-10);
  final float shoulderMax = radians(175);
  final float elbowMin = radians(0);
  final float elbowMax = radians(175);
  final float minRadial = 10;
  final float minPlanar = 30;

  PVector basePos;
  PVector target;
  PVector reachedTarget;
  PVector[] joints;

  float baseAngle;
  float shoulderAngle;
  float elbowAngle;
  float wristPitch;
  float wristYaw;
  float extension;

  IKStatus status;

  ArmKinematics() {
    basePos = new PVector(0, baseHeight, 0);
    target = homeTarget();
    reachedTarget = target.copy();
    joints = new PVector[5];
    for (int i = 0; i < joints.length; i++) {
      joints[i] = new PVector();
    }
    status = new IKStatus();
  }

  PVector homeTarget() {
    return new PVector(basePos.x + 160, basePos.y + 120, 140);
  }

  void solve(PVector desired, boolean elbowUp) {
    status.reset();
    target.set(desired);
    PVector relative = PVector.sub(desired, basePos);
    float rawBaseAngle = atan2(relative.x, relative.z);
    baseAngle = constrain(rawBaseAngle, -baseYawLimit, baseYawLimit);
    if (abs(baseAngle - rawBaseAngle) > 0.001f) {
      status.baseLimited = true;
    }

    PVector radialDir = new PVector(sin(baseAngle), 0, cos(baseAngle));
    float radial = relative.x * radialDir.x + relative.z * radialDir.z;
    float lateral = relative.x * cos(baseAngle) - relative.z * sin(baseAngle);
    status.lateralError = lateral;

    if (radial < minRadial) {
      radial = minRadial;
      status.radialAdjusted = true;
    }

    float vertical = relative.y;
    float planar = sqrt(radial * radial + vertical * vertical);

    if (planar < minPlanar) {
      if (planar < 1e-3f) {
        radial = minPlanar;
        vertical = 0;
      } else {
        float scale = minPlanar / planar;
        radial *= scale;
        vertical *= scale;
      }
      planar = minPlanar;
      status.minDistanceApplied = true;
    }

    float maxReach = shoulderLength + elbowLength + extensionMax;
    if (planar > maxReach) {
      float scale = maxReach / planar;
      radial *= scale;
      vertical *= scale;
      planar = maxReach;
      status.reachLimited = true;
    }

    float effectiveL2 = elbowLength;
    if (planar > shoulderLength + elbowLength) {
      effectiveL2 = planar - shoulderLength;
    }
    effectiveL2 = constrain(effectiveL2, elbowLength, elbowLength + extensionMax);
    extension = effectiveL2 - elbowLength;

    float cosElbow = (sq(shoulderLength) + sq(effectiveL2) - sq(planar)) / (2 * shoulderLength * effectiveL2);
    cosElbow = constrain(cosElbow, -1, 1);
    float elbowInternal = acos(cosElbow);
    elbowAngle = PI - elbowInternal;
    float cosShoulder = (sq(shoulderLength) + sq(planar) - sq(effectiveL2)) / (2 * shoulderLength * planar);
    cosShoulder = constrain(cosShoulder, -1, 1);
    float shoulderOffset = acos(cosShoulder);
    float angleToTarget = atan2(vertical, radial);
    shoulderAngle = elbowUp ? angleToTarget + shoulderOffset : angleToTarget - shoulderOffset;

    if (shoulderAngle < shoulderMin) {
      shoulderAngle = shoulderMin;
      status.shoulderLimited = true;
    } else if (shoulderAngle > shoulderMax) {
      shoulderAngle = shoulderMax;
      status.shoulderLimited = true;
    }

    if (elbowAngle < elbowMin) {
      elbowAngle = elbowMin;
      status.elbowLimited = true;
    } else if (elbowAngle > elbowMax) {
      elbowAngle = elbowMax;
      status.elbowLimited = true;
    }

    float link2Angle = shoulderAngle + elbowAngle;
    joints[0].set(basePos.x, 0, basePos.z);
    joints[1].set(basePos);

    float l1Radial = shoulderLength * cos(shoulderAngle);
    float l1Vertical = shoulderLength * sin(shoulderAngle);
    joints[2].set(
      basePos.x + radialDir.x * l1Radial,
      basePos.y + l1Vertical,
      basePos.z + radialDir.z * l1Radial
    );

    float baseL2Radial = elbowLength * cos(link2Angle);
    float baseL2Vertical = elbowLength * sin(link2Angle);
    joints[3].set(
      joints[2].x + radialDir.x * baseL2Radial,
      joints[2].y + baseL2Vertical,
      joints[2].z + radialDir.z * baseL2Radial
    );

    float totalL2Radial = (elbowLength + extension) * cos(link2Angle);
    float totalL2Vertical = (elbowLength + extension) * sin(link2Angle);
    joints[4].set(
      joints[2].x + radialDir.x * totalL2Radial,
      joints[2].y + totalL2Vertical,
      joints[2].z + radialDir.z * totalL2Radial
    );

    reachedTarget.set(joints[4]);
    wristPitch = -(shoulderAngle + elbowAngle);
    wristYaw = 0;
    status.distanceError = PVector.dist(desired, reachedTarget);
  }

  void drawArm() {
    // Base platform
    noStroke();
    fill(50, 55, 65);
    drawBoxCentered(new PVector(basePos.x, baseHeight / 2f, basePos.z), baseRadius * 2.2, baseHeight, baseRadius * 2.2);
    fill(80, 90, 110);
    drawBoxCentered(new PVector(basePos.x, basePos.y - 8, basePos.z), baseRadius * 2.6, 16, baseRadius * 2.6);

    // Base column
    stroke(140, 180, 240);
    strokeWeight(9);
    drawLine3d(joints[0], joints[1]);

    // Shoulder link
    stroke(255, 170, 60);
    strokeWeight(7);
    drawLine3d(joints[1], joints[2]);

    // Elbow link (before extension)
    stroke(255, 220, 120);
    strokeWeight(6);
    drawLine3d(joints[2], joints[3]);

    // Prismatic extension
    stroke(90, 240, 190);
    strokeWeight(5);
    drawLine3d(joints[3], joints[4]);

    // Joints markers
    noStroke();
    fill(200);
    drawSphere(joints[1], 12);
    fill(230, 200, 120);
    drawSphere(joints[2], 11);
    fill(160, 230, 200);
    drawSphere(joints[3], 10);
    fill(220, 250, 255);
    drawSphere(joints[4], 9);
  }
}

class IKStatus {
  boolean reachLimited;
  boolean baseLimited;
  boolean radialAdjusted;
  boolean minDistanceApplied;
  boolean shoulderLimited;
  boolean elbowLimited;
  float lateralError;
  float distanceError;

  void reset() {
    reachLimited = false;
    baseLimited = false;
    radialAdjusted = false;
    minDistanceApplied = false;
    shoulderLimited = false;
    elbowLimited = false;
    lateralError = 0;
    distanceError = 0;
  }
}
