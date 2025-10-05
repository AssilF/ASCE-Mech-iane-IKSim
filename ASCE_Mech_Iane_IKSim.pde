import processing.event.MouseEvent;

// Processing sketch that visualises a simplified 3D model of the Mech'iane arm
// and solves inverse kinematics to follow an interactive XYZ target point.
ArmKinematics arm;
PVector desiredTarget;

ControlState controls = new ControlState();
PVector moveVelocity = new PVector();
PVector moveVelocityTarget = new PVector();
float yawVelocity;
float yawVelocityTarget;
float pitchVelocity;
float pitchVelocityTarget;
float rollVelocity;
float rollVelocityTarget;

float speedMultiplier = 1.0f;
final float SPEED_MIN = 0.1f;
final float SPEED_MAX = 5.0f;
final float FINE_FACTOR = 0.25f;
final float BASE_MOVE_SPEED = 220.0f;
final float BASE_ANGULAR_SPEED = radians(90);
final float BASE_OPEN_ADJUST_RATE = 2.0f;
final float BASE_LENGTH_ADJUST_RATE = 40.0f;

float camYaw = radians(45);
float camPitch = radians(25);
float camDistance = 900;

boolean rotatingCamera = false;
int lastMouseX;
int lastMouseY;
boolean elbowUpSolution = false;

int lastFrameTime;

void setup() {
  size(1280, 800, P3D);
  surface.setTitle("Mech'iane IK Simulation");
  smooth(8);
  sphereDetail(22);
  arm = new ArmKinematics();
  desiredTarget = arm.target.copy();
  hint(ENABLE_DEPTH_TEST);
  lastFrameTime = millis();
}

void draw() {
  int now = millis();
  float dt = (now - lastFrameTime) / 1000.0f;
  dt = constrain(dt, 0.0f, 0.05f);

  updateControlSmoothing(dt);

  background(12);
  setLights();
  applyCamera();
  arm.solve(desiredTarget, elbowUpSolution);
  drawScene();
  drawHUD();

  lastFrameTime = now;
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
  stroke(220, 70, 70);
  drawLine3d(new PVector(0, 0, 0), new PVector(length, 0, 0));
  stroke(70, 200, 120);
  drawLine3d(new PVector(0, 0, 0), new PVector(0, length, 0));
  stroke(70, 120, 220);
  drawLine3d(new PVector(0, 0, 0), new PVector(0, 0, length));
}

void drawHUD() {
  hint(DISABLE_DEPTH_TEST);
  camera();
  resetMatrix();

  float uiScale = constrain(width / 1280.0f, 0.75f, 1.8f);
  float margin = 18 * uiScale;
  float lineHeight = 18 * uiScale;

  textAlign(LEFT, TOP);
  textSize(14 * uiScale);
  float y = margin;

  String reachState = arm.status.reachLimited ? "reach limited" : (arm.status.baseLimited ? "base limited" : "ok");
  if (arm.status.minDistanceApplied) {
    reachState += " - min-dist";
  }
  if (arm.status.radialAdjusted) {
    reachState += " - radial clamp";
  }
  if (arm.status.shoulderLimited) {
    reachState += " - shoulder clamp";
  }
  if (arm.status.elbowLimited) {
    reachState += " - elbow clamp";
  }

  text("Target (mm)  X:" + nf(desiredTarget.x, 1, 1)
       + "  Y:" + nf(desiredTarget.y, 1, 1)
       + "  Z:" + nf(desiredTarget.z, 1, 1), margin, y);
  y += lineHeight;
  text("End effector  X:" + nf(arm.reachedTarget.x, 1, 1)
       + "  Y:" + nf(arm.reachedTarget.y, 1, 1)
       + "  Z:" + nf(arm.reachedTarget.z, 1, 1), margin, y);
  y += lineHeight;
  text("Position error: " + nf(arm.status.distanceError, 1, 2) + " mm", margin, y);
  y += lineHeight;
  text("Base yaw:" + nf(degrees(arm.baseAngle), 1, 1) + " deg  Shoulder:" + nf(degrees(arm.shoulderAngle), 1, 1)
       + " deg  Elbow:" + nf(degrees(arm.elbowAngle), 1, 1) + " deg  Extension:" + nf(arm.extension, 1, 1) + " mm", margin, y);
  y += lineHeight;
  text("Gripper yaw:" + nf(degrees(arm.gripperYaw), 1, 1) + " deg  Pitch:" + nf(degrees(arm.gripperPitch), 1, 1)
       + " deg  Roll:" + nf(degrees(arm.gripperRoll), 1, 1) + " deg", margin, y);
  y += lineHeight;
  text("Gripper open: " + nf(arm.gripperOpenAmount * 100, 1, 0) + "%", margin, y);
  y += lineHeight;
  text("Segments [mm]  Shoulder:" + nf(arm.shoulderLength, 1, 1)
       + "  Elbow:" + nf(arm.elbowLength, 1, 1)
       + "  Extension max:" + nf(arm.extensionMax, 1, 1), margin, y);
  y += lineHeight;
  text("Speed multiplier x" + nf(speedMultiplier, 1, 2) + (controls.shiftHeld ? "  (fine)" : ""), margin, y);
  y += lineHeight;
  text("Status: " + reachState, margin, y);

  textAlign(RIGHT, TOP);
  textSize(13 * uiScale);
  textLeading(lineHeight);
  String controlsText = "Controls:\n"
    + "Hold keys for continuous motion\n"
    + "W/S: target up/down    A/D: target left/right\n"
    + "Q/E: target toward/away    Shift: fine speed\n"
    + "J/L: gripper yaw    I/K: gripper pitch\n"
    + "U/O: gripper roll    [/]: gripper close/open\n"
    + "P: toggle gripper    G: reset gripper angles\n"
    + "Z/X: shoulder length -/+\n"
    + "C/V: elbow length -/+    B/N: extension max -/+\n"
    + "-/=: speed multiplier -/+    0: reset speed\n"
    + "R: reset target    Space: toggle elbow\n"
    + "Mouse drag: orbit camera    Wheel: zoom";
  text(controlsText, width - margin, margin);

  hint(ENABLE_DEPTH_TEST);
}

void updateControlSmoothing(float dt) {
  float blend = constrain(dt * 10.0f, 0, 1);
  float moveSpeed = BASE_MOVE_SPEED * speedMultiplier * (controls.shiftHeld ? FINE_FACTOR : 1.0f);

  moveVelocityTarget.set(
    moveSpeed * dir(controls.moveRight, controls.moveLeft),
    moveSpeed * dir(controls.moveUp, controls.moveDown),
    moveSpeed * dir(controls.moveAway, controls.moveToward)
  );
  moveVelocity.lerp(moveVelocityTarget, blend);
  desiredTarget.x += moveVelocity.x * dt;
  desiredTarget.y += moveVelocity.y * dt;
  desiredTarget.z += moveVelocity.z * dt;

  float angularSpeed = BASE_ANGULAR_SPEED * speedMultiplier * (controls.shiftHeld ? FINE_FACTOR : 1.0f);
  yawVelocityTarget = angularSpeed * dir(controls.yawRight, controls.yawLeft);
  pitchVelocityTarget = angularSpeed * dir(controls.pitchUp, controls.pitchDown);
  rollVelocityTarget = angularSpeed * dir(controls.rollCcw, controls.rollCw);

  yawVelocity += (yawVelocityTarget - yawVelocity) * blend;
  pitchVelocity += (pitchVelocityTarget - pitchVelocity) * blend;
  rollVelocity += (rollVelocityTarget - rollVelocity) * blend;

  arm.adjustGripper(yawVelocity * dt, pitchVelocity * dt, rollVelocity * dt);

  float openAdjustSpeed = BASE_OPEN_ADJUST_RATE * (controls.shiftHeld ? FINE_FACTOR : 1.0f);
  float openDir = dir(controls.gripperOpen, controls.gripperClose);
  if (openDir != 0) {
    arm.nudgeGripperOpen(openDir * openAdjustSpeed * dt);
  }
  arm.updateGripperOpen(dt);

  float lengthAdjustSpeed = BASE_LENGTH_ADJUST_RATE * (controls.shiftHeld ? FINE_FACTOR : 1.0f);
  float shoulderDir = dir(controls.shoulderIncrease, controls.shoulderDecrease);
  if (shoulderDir != 0) {
    arm.adjustShoulderLength(shoulderDir * lengthAdjustSpeed * dt);
  }
  float elbowDir = dir(controls.elbowIncrease, controls.elbowDecrease);
  if (elbowDir != 0) {
    arm.adjustElbowLength(elbowDir * lengthAdjustSpeed * dt);
  }
  float extensionDir = dir(controls.extensionIncrease, controls.extensionDecrease);
  if (extensionDir != 0) {
    arm.adjustExtensionMax(extensionDir * (lengthAdjustSpeed + 20.0f) * dt);
  }

  clampTarget();
}

int dir(boolean positive, boolean negative) {
  return (positive ? 1 : 0) - (negative ? 1 : 0);
}

void setSpeedMultiplier(float value) {
  speedMultiplier = constrain(value, SPEED_MIN, SPEED_MAX);
}

void keyPressed() {
  if (key == CODED) {
    if (keyCode == SHIFT) {
      controls.shiftHeld = true;
    }
    return;
  }

  if (key == '[') {
    controls.gripperClose = true;
  } else if (key == ']') {
    controls.gripperOpen = true;
  } else if (key == '-' || key == '_') {
    setSpeedMultiplier(speedMultiplier * 0.8f);
  } else if (key == '=' || key == '+') {
    setSpeedMultiplier(speedMultiplier * 1.25f);
  }

  if (key == ' ') {
    elbowUpSolution = !elbowUpSolution;
    return;
  }

  char k = Character.toLowerCase(key);
  switch (k) {
  case 'w':
    controls.moveUp = true;
    break;
  case 's':
    controls.moveDown = true;
    break;
  case 'a':
    controls.moveLeft = true;
    break;
  case 'd':
    controls.moveRight = true;
    break;
  case 'q':
    controls.moveToward = true;
    break;
  case 'e':
    controls.moveAway = true;
    break;
  case 'j':
    controls.yawLeft = true;
    break;
  case 'l':
    controls.yawRight = true;
    break;
  case 'i':
    controls.pitchUp = true;
    break;
  case 'k':
    controls.pitchDown = true;
    break;
  case 'u':
    controls.rollCcw = true;
    break;
  case 'o':
    controls.rollCw = true;
    break;
  case 'z':
    controls.shoulderDecrease = true;
    break;
  case 'x':
    controls.shoulderIncrease = true;
    break;
  case 'c':
    controls.elbowDecrease = true;
    break;
  case 'v':
    controls.elbowIncrease = true;
    break;
  case 'b':
    controls.extensionDecrease = true;
    break;
  case 'n':
    controls.extensionIncrease = true;
    break;
  case 'r':
    desiredTarget.set(arm.homeTarget());
    moveVelocity.set(0, 0, 0);
    moveVelocityTarget.set(0, 0, 0);
    break;
  case 'g':
    arm.resetGripperOrientation();
    yawVelocity = pitchVelocity = rollVelocity = 0;
    yawVelocityTarget = pitchVelocityTarget = rollVelocityTarget = 0;
    break;
  case 'p':
    arm.toggleGripperOpen();
    break;
  case '0':
    setSpeedMultiplier(1.0f);
    break;
  }
}

void keyReleased() {
  if (key == CODED) {
    if (keyCode == SHIFT) {
      controls.shiftHeld = false;
    }
    return;
  }

  if (key == '[') {
    controls.gripperClose = false;
  } else if (key == ']') {
    controls.gripperOpen = false;
  }

  char k = Character.toLowerCase(key);
  switch (k) {
  case 'w':
    controls.moveUp = false;
    break;
  case 's':
    controls.moveDown = false;
    break;
  case 'a':
    controls.moveLeft = false;
    break;
  case 'd':
    controls.moveRight = false;
    break;
  case 'q':
    controls.moveToward = false;
    break;
  case 'e':
    controls.moveAway = false;
    break;
  case 'j':
    controls.yawLeft = false;
    break;
  case 'l':
    controls.yawRight = false;
    break;
  case 'i':
    controls.pitchUp = false;
    break;
  case 'k':
    controls.pitchDown = false;
    break;
  case 'u':
    controls.rollCcw = false;
    break;
  case 'o':
    controls.rollCw = false;
    break;
  case 'z':
    controls.shoulderDecrease = false;
    break;
  case 'x':
    controls.shoulderIncrease = false;
    break;
  case 'c':
    controls.elbowDecrease = false;
    break;
  case 'v':
    controls.elbowIncrease = false;
    break;
  case 'b':
    controls.extensionDecrease = false;
    break;
  case 'n':
    controls.extensionIncrease = false;
    break;
  }
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
    float sensitivity = 0.01f;
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
  float horizontal = max(120, arm.maxPlanarReach() * 1.05f);
  desiredTarget.x = constrain(desiredTarget.x, -horizontal, horizontal);
  desiredTarget.z = constrain(desiredTarget.z, -horizontal, horizontal);

  float minHeight = 40;
  float maxHeight = arm.basePos.y + max(arm.shoulderLength + arm.elbowLength, 150);
  desiredTarget.y = constrain(desiredTarget.y, minHeight, maxHeight);
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

PVector rotateAroundAxis(PVector v, PVector axis, float angle) {
  if (abs(angle) < 1e-6f) {
    return v.copy();
  }
  PVector k = axis.copy();
  if (k.magSq() < 1e-6f) {
    return v.copy();
  }
  k.normalize();
  float cosA = cos(angle);
  float sinA = sin(angle);
  PVector term1 = PVector.mult(v, cosA);
  PVector term2 = PVector.mult(k.copy().cross(v), sinA);
  float dotKV = v.dot(k);
  PVector term3 = PVector.mult(k, dotKV * (1 - cosA));
  PVector result = PVector.add(term1, term2);
  result.add(term3);
  return result;
}

class ControlState {
  boolean moveUp;
  boolean moveDown;
  boolean moveLeft;
  boolean moveRight;
  boolean moveToward;
  boolean moveAway;

  boolean yawLeft;
  boolean yawRight;
  boolean pitchUp;
  boolean pitchDown;
  boolean rollCcw;
  boolean rollCw;

  boolean gripperOpen;
  boolean gripperClose;

  boolean shoulderIncrease;
  boolean shoulderDecrease;
  boolean elbowIncrease;
  boolean elbowDecrease;
  boolean extensionIncrease;
  boolean extensionDecrease;

  boolean shiftHeld;
}

class ArmKinematics {
  float baseHeight = 90;
  float baseRadius = 60;
  float shoulderLength = 140;
  float elbowLength = 110;
  float extensionMax = 110;
  final float baseYawLimit = radians(135);
  final float shoulderMin = radians(-10);
  final float shoulderMax = radians(175);
  final float elbowMin = radians(0);
  final float elbowMax = radians(175);
  final float minRadial = 10;
  final float minPlanar = 30;
  final float gripperAngleMin = radians(-90);
  final float gripperAngleMax = radians(90);

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
  float gripperYaw;
  float gripperPitch;
  float gripperRoll;

  PVector gripperRight;
  PVector gripperUp;
  PVector gripperForward;

  float gripperOpenAmount = 1.0f;
  float gripperOpenTarget = 1.0f;

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
    gripperYaw = 0;
    gripperPitch = 0;
    gripperRoll = 0;
    gripperRight = new PVector(1, 0, 0);
    gripperUp = new PVector(0, 1, 0);
    gripperForward = new PVector(0, 0, 1);
  }

  PVector homeTarget() {
    float planar = (shoulderLength + elbowLength) * 0.6f;
    float height = basePos.y + max(70, shoulderLength * 0.55f);
    return new PVector(basePos.x + planar, height, 140);
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
    updateGripperFrame(radialDir, link2Angle);
    status.distanceError = PVector.dist(desired, reachedTarget);
  }

  void adjustGripper(float yawDelta, float pitchDelta, float rollDelta) {
    gripperYaw = constrain(gripperYaw + yawDelta, gripperAngleMin, gripperAngleMax);
    gripperPitch = constrain(gripperPitch + pitchDelta, gripperAngleMin, gripperAngleMax);
    gripperRoll = constrain(gripperRoll + rollDelta, gripperAngleMin, gripperAngleMax);
  }

  void resetGripperOrientation() {
    gripperYaw = 0;
    gripperPitch = 0;
    gripperRoll = 0;
  }

  void updateGripperFrame(PVector radialDir, float link2Angle) {
    PVector forward = PVector.sub(joints[4], joints[3]);
    if (forward.magSq() < 1e-5f) {
      forward = PVector.sub(joints[4], joints[2]);
    }
    if (forward.magSq() < 1e-5f) {
      float forwardRadial = cos(link2Angle);
      float forwardVertical = sin(link2Angle);
      forward.set(radialDir.x * forwardRadial, forwardVertical, radialDir.z * forwardRadial);
    }
    forward.normalize();

    PVector worldUp = new PVector(0, 1, 0);
    PVector right = worldUp.copy().cross(forward);
    if (right.magSq() < 1e-5f) {
      right = radialDir.copy().cross(forward);
    }
    if (right.magSq() < 1e-5f) {
      right.set(1, 0, 0);
    } else {
      right.normalize();
    }
    PVector up = forward.copy().cross(right);
    if (up.magSq() < 1e-5f) {
      up.set(0, 1, 0);
    } else {
      up.normalize();
    }

    gripperForward.set(forward);
    gripperRight.set(right);
    gripperUp.set(up);

    applyGripperServoRotations();
  }

  void applyGripperServoRotations() {
    PVector newRight = rotateAroundAxis(gripperRight, gripperUp, gripperYaw);
    PVector newForward = rotateAroundAxis(gripperForward, gripperUp, gripperYaw);
    gripperRight.set(newRight);
    gripperForward.set(newForward);

    PVector newUp = rotateAroundAxis(gripperUp, gripperRight, gripperPitch);
    newForward = rotateAroundAxis(gripperForward, gripperRight, gripperPitch);
    gripperUp.set(newUp);
    gripperForward.set(newForward);

    newRight = rotateAroundAxis(gripperRight, gripperForward, gripperRoll);
    newUp = rotateAroundAxis(gripperUp, gripperForward, gripperRoll);
    gripperRight.set(newRight);
    gripperUp.set(newUp);

    ensureGripperOrthonormal();
  }

  void ensureGripperOrthonormal() {
    gripperForward.normalize();
    gripperRight.normalize();

    PVector up = gripperForward.copy().cross(gripperRight);
    if (up.magSq() < 1e-6f) {
      up = gripperForward.copy().cross(new PVector(0, 1, 0));
    }
    if (up.magSq() < 1e-6f) {
      up = new PVector(0, 1, 0);
    }
    up.normalize();
    gripperUp.set(up);

    PVector right = gripperUp.copy().cross(gripperForward);
    if (right.magSq() < 1e-6f) {
      right = new PVector(1, 0, 0);
    }
    right.normalize();
    gripperRight.set(right);
  }

  void drawArm() {
    noStroke();
    fill(50, 55, 65);
    drawBoxCentered(new PVector(basePos.x, baseHeight / 2f, basePos.z), baseRadius * 2.2, baseHeight, baseRadius * 2.2);
    fill(80, 90, 110);
    drawBoxCentered(new PVector(basePos.x, basePos.y - 8, basePos.z), baseRadius * 2.6, 16, baseRadius * 2.6);

    stroke(140, 180, 240);
    strokeWeight(9);
    drawLine3d(joints[0], joints[1]);

    stroke(255, 170, 60);
    strokeWeight(7);
    drawLine3d(joints[1], joints[2]);

    stroke(255, 220, 120);
    strokeWeight(6);
    drawLine3d(joints[2], joints[3]);

    stroke(90, 240, 190);
    strokeWeight(5);
    drawLine3d(joints[3], joints[4]);

    noStroke();
    fill(200);
    drawSphere(joints[1], 12);
    fill(230, 200, 120);
    drawSphere(joints[2], 11);
    fill(160, 230, 200);
    drawSphere(joints[3], 10);
    fill(220, 250, 255);
    drawSphere(joints[4], 9);

    drawGripper();
  }

  void drawGripper() {
    float axisLength = 60;
    strokeWeight(3);
    stroke(255, 100, 100);
    drawLine3d(joints[4], PVector.add(joints[4], PVector.mult(gripperRight, axisLength)));
    stroke(120, 255, 160);
    drawLine3d(joints[4], PVector.add(joints[4], PVector.mult(gripperUp, axisLength)));
    stroke(120, 160, 255);
    drawLine3d(joints[4], PVector.add(joints[4], PVector.mult(gripperForward, axisLength)));

    float fingerSpacingClosed = 3;
    float fingerSpacingOpen = 14;
    float fingerSpacing = lerp(fingerSpacingClosed, fingerSpacingOpen, gripperOpenAmount);

    float fingerLengthClosed = 22;
    float fingerLengthOpen = 36;
    float fingerLength = lerp(fingerLengthClosed, fingerLengthOpen, gripperOpenAmount);

    float palmOffsetClosed = 20;
    float palmOffsetOpen = 28;
    float palmOffset = lerp(palmOffsetClosed, palmOffsetOpen, gripperOpenAmount);

    PVector palmCenter = PVector.add(joints[4], PVector.mult(gripperForward, palmOffset));
    PVector offset = PVector.mult(gripperRight, fingerSpacing);
    PVector finger1Base = PVector.add(palmCenter, offset);
    PVector finger2Base = PVector.sub(palmCenter, offset);
    PVector forwardOffset = PVector.mult(gripperForward, fingerLength);
    PVector finger1Tip = PVector.add(finger1Base, forwardOffset);
    PVector finger2Tip = PVector.add(finger2Base, forwardOffset);

    stroke(235, 245, 255);
    strokeWeight(5);
    drawLine3d(finger1Base, finger1Tip);
    drawLine3d(finger2Base, finger2Tip);
    strokeWeight(3);
    drawLine3d(finger1Base, finger2Base);
  }

  void nudgeGripperOpen(float delta) {
    gripperOpenTarget = constrain(gripperOpenTarget + delta, 0, 1);
  }

  void toggleGripperOpen() {
    gripperOpenTarget = gripperOpenTarget < 0.5f ? 1.0f : 0.0f;
  }

  void updateGripperOpen(float dt) {
    float chaseSpeed = 6.0f;
    float diff = gripperOpenTarget - gripperOpenAmount;
    if (abs(diff) > 1e-4f) {
      float step = constrain(diff, -chaseSpeed * dt, chaseSpeed * dt);
      gripperOpenAmount += step;
    }
  }

  void adjustShoulderLength(float delta) {
    shoulderLength = constrain(shoulderLength + delta, 80, 260);
  }

  void adjustElbowLength(float delta) {
    elbowLength = constrain(elbowLength + delta, 60, 220);
  }

  void adjustExtensionMax(float delta) {
    extensionMax = constrain(extensionMax + delta, 30, 220);
    extension = constrain(extension, 0, extensionMax);
  }

  float maxPlanarReach() {
    return shoulderLength + elbowLength + extensionMax;
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
