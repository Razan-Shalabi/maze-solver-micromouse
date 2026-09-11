#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <WiFiManager.h>   // https://github.com/tzapu/WiFiManager - captive portal WiFi setup
#include <ESPmDNS.h>       // built into ESP32 core - lets users reach the robot at http://toots.local

// maze size
#define MAZE_SIZE 8

// ===== WIFI SETUP (WiFiManager) 
// On first boot (or after a WiFi reset) the ESP32 opens its own access point called "TOOTs-Setup".
// Connect a phone or laptop to that AP, a setup page will pop up (or go to 192.168.4.1), pick your home WiFi network from the list, enter the
// password, and the ESP32 will save it to flash and reboot onto your
// network automatically. Every future boot will reconnect on its own.
const char* AP_SETUP_NAME = "TOOTs-Setup";
const char* MDNS_HOSTNAME = "toots"; // reachable at http://toots.local

WebServer server(80);

// tof pins
#define TOF_SDA     21
#define TOF_SCL     22
#define FRONT_XSHUT 25
#define LEFT_XSHUT  26
#define RIGHT_XSHUT 27
#define FRONT_ADDR  0x30
#define LEFT_ADDR   0x31
#define RIGHT_ADDR  0x32
VL53L0X tofFront;
VL53L0X tofLeft;
VL53L0X tofRight;
int leftDist  = 500;
int frontDist = 500;
int rightDist = 500;

// motor pins
#define IN1 14
#define IN2 12
#define IN3 13
#define IN4 15

// encoders pins
#define ENC_LEFT_C1  34
#define ENC_LEFT_C2  35
#define ENC_RIGHT_C1 32
#define ENC_RIGHT_C2 33

// encoders
volatile long leftTicks  = 0;
volatile long rightTicks = 0;

void IRAM_ATTR leftEncoderISR() {
  if (digitalRead(ENC_LEFT_C2) == HIGH)
    leftTicks++;
  else
    leftTicks--;
}

void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(ENC_RIGHT_C2) == HIGH)
    rightTicks++;
  else
    rightTicks--;
}

// tunable parameters
int   BASE_SPEED     = 120;
int   MIN_SPEED      = 58;
int   MAX_CORRECTION = 50;
int   BASE_CORRECTION = 3;

// sensor calibration
int   LEFT_OFFSET  = -70;
int   RIGHT_OFFSET = -10;

// WALL CORRECTION PARAMETERS
int   SIDE_TARGET_MM   = 45;
int   WALL_CORR_STRENGTH = 2;
int   WALL_CORR_LIMIT  = 15;

// PID VALUES
float Kp = 3.0;
float Ki = 0.005;
float Kd = 0.5;

// TURN CALIBRATION
int   TURN_SPEED        = 120;
int   TURN_DELAY_LEFT   = 700;
int   TURN_DELAY_RIGHT  = 435;
int   TURN_DELAY_180    = 790;

// CELL CALIBRATION
long  CELL_TICKS     = 250;
int   WALL_THRESHOLD = 200;
int   FRONT_STOP     = 100;

// MAZE VARIABLES
#define MAZE MAZE_SIZE
uint8_t wallsGrid[MAZE][MAZE];
bool    visitedGrid[MAZE][MAZE];
uint8_t floodGrid[MAZE][MAZE];
uint8_t pathGrid[MAZE][MAZE];

int robotX   = 0;
int robotY   = 0;
int robotDir = 0;

int startX = 0;
int startY = 0;
int startDir = 0;

const int DX[4] = { 0, 1, 0, -1 };
const int DY[4] = { 1, 0, -1, 0 };

enum RunMode { MODE_IDLE, MODE_SOLVE, MODE_RETURN, MODE_SPEEDRUN };
RunMode runMode = MODE_IDLE;

bool mazeMapped = false;
bool abortFlag    = false;
bool motionActive = false;
bool continuousForward = false;
String statusText   = "READY";
String lastDecision = "-";
int    decisionSeq  = 0;

// FORWARD DECLARATIONS
void startMode(RunMode m);
void stopEverything();
void logDecision(String d);
void wallAlignment();
void postTurnCalibration();
int getBestDirection();

// ENCODER HELPERS
void resetEncoders() {
  noInterrupts();
  leftTicks = 0;
  rightTicks = 0;
  interrupts();
}

// MOTORS
void setMotor(int leftSpeed, int rightSpeed) {
  rightSpeed = -rightSpeed;
  leftSpeed  = constrain(leftSpeed,  -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  if (leftSpeed > 0) {
    analogWrite(IN1, leftSpeed);
    analogWrite(IN2, 0);
  } else if (leftSpeed < 0) {
    analogWrite(IN1, 0);
    analogWrite(IN2, -leftSpeed);
  } else {
    analogWrite(IN1, 0);
    analogWrite(IN2, 0);
  }

  if (rightSpeed > 0) {
    analogWrite(IN3, rightSpeed);
    analogWrite(IN4, 0);
  } else if (rightSpeed < 0) {
    analogWrite(IN3, 0);
    analogWrite(IN4, -rightSpeed);
  } else {
    analogWrite(IN3, 0);
    analogWrite(IN4, 0);
  }
}

void stopMotorsOnly() { setMotor(0, 0); }

// TOF FUNCTIONS
void setupToFs() {
  pinMode(FRONT_XSHUT, OUTPUT);
  pinMode(LEFT_XSHUT,  OUTPUT);
  pinMode(RIGHT_XSHUT, OUTPUT);

  // Turn off all sensors first
  digitalWrite(FRONT_XSHUT, LOW);
  digitalWrite(LEFT_XSHUT,  LOW);
  digitalWrite(RIGHT_XSHUT, LOW);
  delay(20);

  // Start front sensor
  digitalWrite(FRONT_XSHUT, HIGH);
  delay(20);

  if (!tofFront.init()) {
    Serial.println("ERROR: Front ToF sensor not detected");
  } else {
    tofFront.setAddress(FRONT_ADDR);
    tofFront.setTimeout(100);
    tofFront.startContinuous(20);
    Serial.println("Front ToF ready");
  }

  // Start left sensor
  digitalWrite(LEFT_XSHUT, HIGH);
  delay(20);

  if (!tofLeft.init()) {
    Serial.println("ERROR: Left ToF sensor not detected");
  } else {
    tofLeft.setAddress(LEFT_ADDR);
    tofLeft.setTimeout(100);
    tofLeft.startContinuous(20);
    Serial.println("Left ToF ready");
  }

  // Start right sensor
  digitalWrite(RIGHT_XSHUT, HIGH);
  delay(20);

  if (!tofRight.init()) {
    Serial.println("ERROR: Right ToF sensor not detected");
  } else {
    tofRight.setAddress(RIGHT_ADDR);
    tofRight.setTimeout(100);
    tofRight.startContinuous(20);
    Serial.println("Right ToF ready");
  }
}

void readToFs() {
  int f = tofFront.readRangeContinuousMillimeters();
  int l = tofLeft.readRangeContinuousMillimeters();
  int r = tofRight.readRangeContinuousMillimeters();

  // Front sensor reading check
  if (tofFront.timeoutOccurred() || f <= 0 || f > 2000) {
    f = 2000;
  }

  // Left sensor reading check
  if (tofLeft.timeoutOccurred() || l <= 0 || l > 2000) {
    l = 2000;
  } else {
    l += LEFT_OFFSET;

    if (l < 0) {
      l = 0;
    }
  }

  // Right sensor reading check
  if (tofRight.timeoutOccurred() || r <= 0 || r > 2000) {
    r = 2000;
  } else {
    r += RIGHT_OFFSET;

    if (r < 0) {
      r = 0;
    }
  }

  frontDist = f;
  leftDist  = l;
  rightDist = r;
}

void readToFsAveraged() {
  long sl = 0, sf = 0, sr = 0;
  for (int i = 0; i < 3; i++) {
    readToFs();
    sl += leftDist; sf += frontDist; sr += rightDist;
    delay(20);
  }
  leftDist = sl/3; frontDist = sf/3; rightDist = sr/3;
}

bool leftWall()  { return leftDist  < WALL_THRESHOLD; }
bool frontWall() { return frontDist < WALL_THRESHOLD; }
bool rightWall() { return rightDist < WALL_THRESHOLD; }

// GET WALL CORRECTION
int getWallCorrection() {
  int wallCorr = 0;
  bool hasLeftWall = (leftDist < WALL_THRESHOLD);
  bool hasRightWall = (rightDist < WALL_THRESHOLD);

  if (hasLeftWall && hasRightWall) {
    int error = leftDist - rightDist;
    wallCorr = error / WALL_CORR_STRENGTH;
  } else if (hasLeftWall) {
    int error = leftDist - SIDE_TARGET_MM;
    wallCorr = error / WALL_CORR_STRENGTH;
  } else if (hasRightWall) {
    int error = SIDE_TARGET_MM - rightDist;
    wallCorr = error / WALL_CORR_STRENGTH;
  }

  wallCorr = constrain(wallCorr, -WALL_CORR_LIMIT, WALL_CORR_LIMIT);
  return wallCorr;
}

// POST-TURN CALIBRATION
void postTurnCalibration() {
  Serial.println("=== POST-TURN CALIBRATION ===");

  stopMotorsOnly();
  delay(150);

  const int MAX_ATTEMPTS = 3;
  const int ERROR_TOLERANCE = 3;
  const int CALIBRATION_SPEED = 55;

  for (int attempt = 0; attempt < MAX_ATTEMPTS; attempt++) {
    readToFsAveraged();

    bool hasLeftWall  = leftDist  < WALL_THRESHOLD;
    bool hasRightWall = rightDist < WALL_THRESHOLD;

    Serial.print("Attempt: ");
    Serial.print(attempt + 1);

    Serial.print(" | Left: ");
    Serial.print(leftDist);

    Serial.print(" mm | Right: ");
    Serial.println(rightDist);

    int correction = 0;

    // Robot is between two walls
    if (hasLeftWall && hasRightWall) {
      correction = leftDist - rightDist;
    }

    // Only left wall exists
    else if (hasLeftWall) {
      correction = leftDist - SIDE_TARGET_MM;
    }

    // Only right wall exists
    else if (hasRightWall) {
      correction = SIDE_TARGET_MM - rightDist;
    }

    // No side walls, so ToF calibration is not possible
    else {
      Serial.println("No side walls available for calibration");
      return;
    }

    correction /= WALL_CORR_STRENGTH;

    correction = constrain(
      correction,
      -WALL_CORR_LIMIT,
      WALL_CORR_LIMIT
    );

    Serial.print("Correction: ");
    Serial.println(correction);

    // Calibration is good enough
    if (abs(correction) <= ERROR_TOLERANCE) {
      Serial.println("Post-turn calibration completed");
      break;
    }

    // Small pivot instead of moving forward
    if (correction > 0) {
      setMotor(-CALIBRATION_SPEED, CALIBRATION_SPEED);
    } else {
      setMotor(CALIBRATION_SPEED, -CALIBRATION_SPEED);
    }

    // Small correction pulse
    int pulseTime = abs(correction) * 3;
    pulseTime = constrain(pulseTime, 15, 60);

    delay(pulseTime);

    stopMotorsOnly();
    delay(100);
  }

  stopMotorsOnly();
  readToFsAveraged();

  Serial.print("Final Left: ");
  Serial.print(leftDist);

  Serial.print(" mm | Final Right: ");
  Serial.print(rightDist);

  Serial.println(" mm");
}

// WALL ALIGNMENT AFTER CELL
void wallAlignment() {
  stopMotorsOnly();
  delay(100);

  readToFsAveraged();

  bool hasLeftWall  = leftDist  < WALL_THRESHOLD;
  bool hasRightWall = rightDist < WALL_THRESHOLD;

  Serial.print("Left: ");
  Serial.print(leftDist);
  Serial.print(" mm | Right: ");
  Serial.print(rightDist);
  Serial.print(" mm | ");

  if (hasLeftWall && hasRightWall) {
    int error = leftDist - rightDist;

    Serial.print("Both walls | Center error: ");
    Serial.println(error);

  } else if (hasLeftWall) {
    int error = leftDist - SIDE_TARGET_MM;

    Serial.print("Left wall | Distance error: ");
    Serial.println(error);

  } else if (hasRightWall) {
    int error = SIDE_TARGET_MM - rightDist;

    Serial.print("Right wall | Distance error: ");
    Serial.println(error);

  } else {
    Serial.println("No side walls");
  }
}

// CONTINUOUS FORWARD
void moveContinuousForward() {
  continuousForward = true;
  motionActive = true;
  statusText = "CONTINUOUS FWD";

  resetEncoders();

  float prevError = 0;
  float integral = 0;
  float lastError = 0;

  while (continuousForward && !abortFlag) {
    server.handleClient();
    readToFs();

    long dL;
    long dR;

    // Read encoder values safely
    noInterrupts();
    dL = leftTicks;
    dR = rightTicks;
    interrupts();

    int wallCorr = getWallCorrection();

    float error = dL + dR;

    // Ignore very small encoder differences
    if (abs(error) < 3) {
      error = 0;
    }

    // Reset integral if the error direction changes
    if (error * lastError < 0) {
      integral = 0;
    }

    lastError = error;

    // Calculate and limit integral
    integral += error;
    integral = constrain(integral, -20.0f, 20.0f);

    // Calculate derivative
    float derivative = error - prevError;

    // PID + motor bias + wall correction
    float correction =
        (Kp * error) +
        (Ki * integral) +
        (Kd * derivative) +
        BASE_CORRECTION +
        wallCorr;

    correction = constrain(
        correction,
        -(float)MAX_CORRECTION,
        (float)MAX_CORRECTION
    );

    int leftSpeed = constrain(
        BASE_SPEED - (int)correction,
        MIN_SPEED,
        255
    );

    int rightSpeed = constrain(
        BASE_SPEED + (int)correction,
        MIN_SPEED,
        255
    );

    setMotor(leftSpeed, rightSpeed);

    prevError = error;

    delay(20);
  }

  stopMotorsOnly();
  motionActive = false;
  statusText = "STOPPED";
}

// ===== MOVEMENT - ONE CELL =====
bool moveOneCell() {
  stopMotorsOnly();
  delay(100);

  resetEncoders();

  motionActive = true;
  statusText = "MOVING";

  bool arrived = false;

  float prevError = 0.0;
  float integral = 0.0;
  float lastError = 0.0;

  int debugCount = 0;

  while (!abortFlag) {
    server.handleClient();
    readToFs();

    long dL;
    long dR;

    // Read both encoders safely
    noInterrupts();
    dL = leftTicks;
    dR = rightTicks;
    interrupts();

    long avgTicks = (abs(dL) + abs(dR)) / 2;

    if (debugCount++ % 50 == 0) {
      Serial.print("Cell L: ");
      Serial.print(dL);

      Serial.print(" | R: ");
      Serial.print(dR);

      Serial.print(" | Average: ");
      Serial.print(avgTicks);

      Serial.print(" | Target: ");
      Serial.println(CELL_TICKS);
    }

    // Full cell distance completed
    if (avgTicks >= CELL_TICKS) {
      arrived = true;
      break;
    }

    // A wall was detected before finishing the cell
    if (frontDist < FRONT_STOP) {
      statusText = "BLOCKED";
      arrived = false;
      break;
    }

    int wallCorr = getWallCorrection();

    float error = dL + dR;

    // Ignore very small encoder error
    if (abs(error) < 3) {
      error = 0;
    }

    // Reset integral when error changes direction
    if (error * lastError < 0) {
      integral = 0;
    }

    lastError = error;

    integral += error;
    integral = constrain(integral, -20.0f, 20.0f);

    float derivative = error - prevError;

    float correction =
        (Kp * error) +
        (Ki * integral) +
        (Kd * derivative) +
        BASE_CORRECTION +
        wallCorr;

    correction = constrain(
        correction,
        -(float)MAX_CORRECTION,
        (float)MAX_CORRECTION
    );

    int leftSpeed = constrain(
        BASE_SPEED - (int)correction,
        MIN_SPEED,
        255
    );

    int rightSpeed = constrain(
        BASE_SPEED + (int)correction,
        MIN_SPEED,
        255
    );

    setMotor(leftSpeed, rightSpeed);

    prevError = error;

    delay(20);
  }

  stopMotorsOnly();
  motionActive = false;

  if (arrived && !abortFlag) {
    wallAlignment();
    statusText = "CELL DONE";
    return true;
  }

  if (abortFlag) {
    statusText = "MOVEMENT ABORTED";
  }

  return false;
}

// Allows the web STOP command to work during a turn
bool runTurnForTime(int durationMs) {
  unsigned long startTime = millis();

  while ((millis() - startTime) < durationMs) {
    server.handleClient();

    if (abortFlag) {
      stopMotorsOnly();
      return false;
    }

    delay(5);
  }

  return true;
}

bool pivotLeft() {
  stopMotorsOnly();
  delay(100);

  resetEncoders();

  motionActive = true;
  statusText = "TURNING LEFT";

  setMotor(-TURN_SPEED, TURN_SPEED);

  bool completed = runTurnForTime(TURN_DELAY_LEFT);

  stopMotorsOnly();
  delay(150);

  if (!completed || abortFlag) {
    motionActive = false;
    statusText = "LEFT ABORTED";
    return false;
  }

  postTurnCalibration();

  motionActive = false;
  statusText = "LEFT DONE";

  return true;
}

bool pivotRight() {
  stopMotorsOnly();
  delay(100);

  resetEncoders();

  motionActive = true;
  statusText = "TURNING RIGHT";

  setMotor(TURN_SPEED, -TURN_SPEED);

  bool completed = runTurnForTime(TURN_DELAY_RIGHT);

  stopMotorsOnly();
  delay(150);

  if (!completed || abortFlag) {
    motionActive = false;
    statusText = "RIGHT ABORTED";
    return false;
  }

  postTurnCalibration();

  motionActive = false;
  statusText = "RIGHT DONE";

  return true;
}

bool pivot180() {
  stopMotorsOnly();
  delay(100);

  resetEncoders();

  motionActive = true;
  statusText = "TURNING 180";

  setMotor(TURN_SPEED, -TURN_SPEED);

  bool completed = runTurnForTime(TURN_DELAY_180);

  stopMotorsOnly();
  delay(150);

  if (!completed || abortFlag) {
    motionActive = false;
    statusText = "180 ABORTED";
    return false;
  }

  postTurnCalibration();

  motionActive = false;
  statusText = "180 DONE";

  return true;
}

void turnLeft90() {
  if (pivotLeft()) {
    robotDir = (robotDir + 3) & 3;
    statusText = "LEFT DONE";
  }
}

void turnRight90() {
  if (pivotRight()) {
    robotDir = (robotDir + 1) & 3;
    statusText = "RIGHT DONE";
  }
}

void turn180() {
  if (pivot180()) {
    robotDir = (robotDir + 2) & 3;
    statusText = "180 DONE";
  }
}

bool inMaze(int x, int y) {
  return x >= 0 && x < MAZE &&
         y >= 0 && y < MAZE;
}

bool hasWall(int x, int y, int dir) {
  if (!inMaze(x, y) || dir < 0 || dir > 3) {
    return true;
  }

  return (wallsGrid[y][x] & (1 << dir)) != 0;
}

void setWall(int x, int y, int dir) {
  if (!inMaze(x, y) || dir < 0 || dir > 3) {
    return;
  }

  wallsGrid[y][x] |= (1 << dir);

  int nx = x + DX[dir];
  int ny = y + DY[dir];

  // Add the opposite wall to the neighboring cell
  if (inMaze(nx, ny)) {
    int oppositeDir = (dir + 2) & 3;
    wallsGrid[ny][nx] |= (1 << oppositeDir);
  }
}

// Dynamic center goal based on maze size
bool isCenterGoal(int x, int y) {
  int centerLow  = (MAZE - 1) / 2;
  int centerHigh = MAZE / 2;

  // Odd-sized maze: one center cell
  if (MAZE % 2 == 1) {
    return x == centerHigh &&
           y == centerHigh;
  }

  // Even-sized maze: four center cells
  return (x == centerLow || x == centerHigh) &&
         (y == centerLow || y == centerHigh);
}

bool isFullyMapped() {
  for (int y = 0; y < MAZE; y++) {
    for (int x = 0; x < MAZE; x++) {
      if (!visitedGrid[y][x]) {
        return false;
      }
    }
  }

  return true;
}

int getMappedCount() {
  int count = 0;

  for (int y = 0; y < MAZE; y++) {
    for (int x = 0; x < MAZE; x++) {
      if (visitedGrid[y][x]) {
        count++;
      }
    }
  }

  return count;
}

void resetMaze() {
  for (int y = 0; y < MAZE; y++) {
    for (int x = 0; x < MAZE; x++) {
      wallsGrid[y][x] = 0;
      visitedGrid[y][x] = false;
      floodGrid[y][x] = 255;
      pathGrid[y][x] = 255;
    }
  }

  // Add outer maze walls
  for (int i = 0; i < MAZE; i++) {
    setWall(i, 0, 2);          // South wall
    setWall(i, MAZE - 1, 0);   // North wall
    setWall(0, i, 3);          // West wall
    setWall(MAZE - 1, i, 1);   // East wall
  }

  robotX = 0;
  robotY = 0;
  robotDir = 0;

  startX = 0;
  startY = 0;
  startDir = 0;

  mazeMapped = false;
  statusText = "MAZE RESET";

  Serial.println(
    "=== MAZE RESET: " +
    String(MAZE) + "x" +
    String(MAZE) + " ==="
  );
}

// flood fill
void floodFill(bool toStart, bool onlyVisited) {
  uint8_t qx[MAZE * MAZE];
  uint8_t qy[MAZE * MAZE];

  int head = 0;
  int tail = 0;

  // Reset all flood values
  for (int y = 0; y < MAZE; y++) {
    for (int x = 0; x < MAZE; x++) {
      floodGrid[y][x] = 255;
    }
  }

  // Flood toward the starting cell
  if (toStart) {
    floodGrid[startY][startX] = 0;

    qx[tail] = startX;
    qy[tail] = startY;
    tail++;
  }

  // Flood toward the center goal cells
  else {
    int centerLow  = (MAZE - 1) / 2;
    int centerHigh = MAZE / 2;

    // Odd maze size: one center cell
    if (MAZE % 2 == 1) {
      floodGrid[centerHigh][centerHigh] = 0;

      qx[tail] = centerHigh;
      qy[tail] = centerHigh;
      tail++;
    }

    // Even maze size: four center cells
    else {
      int goalX[4] = {
        centerLow,
        centerHigh,
        centerLow,
        centerHigh
      };

      int goalY[4] = {
        centerLow,
        centerLow,
        centerHigh,
        centerHigh
      };

      for (int i = 0; i < 4; i++) {
        floodGrid[goalY[i]][goalX[i]] = 0;

        qx[tail] = goalX[i];
        qy[tail] = goalY[i];
        tail++;
      }
    }
  }

  // Breadth-first search
  while (head < tail) {
    int x = qx[head];
    int y = qy[head];
    head++;

    for (int d = 0; d < 4; d++) {
      // Do not cross a known wall
      if (hasWall(x, y, d)) {
        continue;
      }

      int nx = x + DX[d];
      int ny = y + DY[d];

      if (!inMaze(nx, ny)) {
        continue;
      }

      // When requested, use only visited cells
      if (
        onlyVisited &&
        !visitedGrid[ny][nx] &&
        !(nx == robotX && ny == robotY)
      ) {
        continue;
      }

      // Cell was already assigned a flood value
      if (floodGrid[ny][nx] != 255) {
        continue;
      }

      floodGrid[ny][nx] = floodGrid[y][x] + 1;

      qx[tail] = nx;
      qy[tail] = ny;
      tail++;
    }
  }

  Serial.println("=== FLOOD FILL RESULTS ===");

  Serial.print("Robot at (");
  Serial.print(robotX);
  Serial.print(",");
  Serial.print(robotY);
  Serial.println(")");

  Serial.print("Flood value: ");
  Serial.println(floodGrid[robotY][robotX]);
}

void findShortestPath() {
  // Clear any old path
  for (int y = 0; y < MAZE; y++) {
    for (int x = 0; x < MAZE; x++) {
      pathGrid[y][x] = 255;
    }
  }
  floodFill(false, true);

  int x = startX;
  int y = startY;
  int step = 0;

  while (
    !isCenterGoal(x, y) &&
    step < MAZE * MAZE
  ) {
    int bestDir = -1;
    int bestVal = 255;

    for (int d = 0; d < 4; d++) {
      if (hasWall(x, y, d)) {
        continue;
      }

      int nx = x + DX[d];
      int ny = y + DY[d];

      if (!inMaze(nx, ny)) {
        continue;
      }

      // Speed run must stay in mapped cells
      if (!visitedGrid[ny][nx] &&
          !isCenterGoal(nx, ny)) {
        continue;
      }

      if (floodGrid[ny][nx] < bestVal) {
        bestVal = floodGrid[ny][nx];
        bestDir = d;
      }
    }

    if (bestDir < 0 || bestVal == 255) {
      Serial.println(
        "ERROR: No valid mapped path to center"
      );
      break;
    }

    pathGrid[y][x] = bestDir;

    x += DX[bestDir];
    y += DY[bestDir];

    step++;
  }

  // Mark the final goal cell as the end of the path
  if (inMaze(x, y)) {
    pathGrid[y][x] = 255;
  }

  Serial.println("=== SHORTEST PATH FOUND ===");

  Serial.print("Path length: ");
  Serial.println(step);

  if (!isCenterGoal(x, y)) {
    Serial.println(
      "WARNING: Path did not reach the center"
    );
  }
}

// WALL DETECTION
void senseAndUpdateWalls() {
  Serial.println("=== SENSING WALLS ===");

  readToFsAveraged();

  int dF = robotDir;
  int dL = (robotDir + 3) & 3;
  int dR = (robotDir + 1) & 3;

  // Get sensor readings
  bool fWall = frontWall();
  bool lWall = leftWall();
  bool rWall = rightWall();

  // Check if robot is at maze boundary and force walls
  bool atWestBoundary = (robotX == 0);
  bool atEastBoundary = (robotX == MAZE - 1);
  bool atSouthBoundary = (robotY == 0);
  bool atNorthBoundary = (robotY == MAZE - 1);

  // Force left wall if at boundary and left direction points outside
  if (atWestBoundary && dL == 3) {
    lWall = true;
    Serial.println("At West boundary - forcing LEFT wall");
  }
  if (atEastBoundary && dL == 1) {
    lWall = true;
    Serial.println("At East boundary - forcing LEFT wall");
  }
  if (atSouthBoundary && dL == 2) {
    lWall = true;
    Serial.println("At South boundary - forcing LEFT wall");
  }
  if (atNorthBoundary && dL == 0) {
    lWall = true;
    Serial.println("At North boundary - forcing LEFT wall");
  }

  // Force right wall if at boundary and right direction points outside
  if (atWestBoundary && dR == 3) {
    rWall = true;
    Serial.println("At West boundary - forcing RIGHT wall");
  }
  if (atEastBoundary && dR == 1) {
    rWall = true;
    Serial.println("At East boundary - forcing RIGHT wall");
  }
  if (atSouthBoundary && dR == 2) {
    rWall = true;
    Serial.println("At South boundary - forcing RIGHT wall");
  }
  if (atNorthBoundary && dR == 0) {
    rWall = true;
    Serial.println("At North boundary - forcing RIGHT wall");
  }

  // Force front wall if at boundary and front direction points outside
  if (atWestBoundary && dF == 3) {
    fWall = true;
    Serial.println("At West boundary - forcing FRONT wall");
  }
  if (atEastBoundary && dF == 1) {
    fWall = true;
    Serial.println("At East boundary - forcing FRONT wall");
  }
  if (atSouthBoundary && dF == 2) {
    fWall = true;
    Serial.println("At South boundary - forcing FRONT wall");
  }
  if (atNorthBoundary && dF == 0) {
    fWall = true;
    Serial.println("At North boundary - forcing FRONT wall");
  }

  Serial.print("Front: ");
  Serial.print(frontDist);
  Serial.print(" mm (Wall: ");
  Serial.print(fWall ? "YES" : "NO");
  Serial.print(") | Left: ");
  Serial.print(leftDist);
  Serial.print(" mm (Wall: ");
  Serial.print(lWall ? "YES" : "NO");
  Serial.print(") | Right: ");
  Serial.print(rightDist);
  Serial.print(" mm (Wall: ");
  Serial.print(rWall ? "YES" : "NO");
  Serial.println(")");

  if (fWall) {
    setWall(robotX, robotY, dF);
  }
  if (lWall) {
    setWall(robotX, robotY, dL);
  }
  if (rWall) {
    setWall(robotX, robotY, dR);
  }

  visitedGrid[robotY][robotX] = true;

  Serial.print("Cell (");
  Serial.print(robotX);
  Serial.print(",");
  Serial.print(robotY);
  Serial.print(") walls: ");
  if (hasWall(robotX, robotY, 0)) Serial.print("N ");
  if (hasWall(robotX, robotY, 1)) Serial.print("E ");
  if (hasWall(robotX, robotY, 2)) Serial.print("S ");
  if (hasWall(robotX, robotY, 3)) Serial.print("W ");
  Serial.println();
}

// get the best direction
int getBestDirection() {
  int bestDir = -1;
  int bestFlood = 255;
  bool bestVisited = true;

  Serial.println("=== EVALUATING DIRECTIONS ===");

  for (int d = 0; d < 4; d++) {
    if (hasWall(robotX, robotY, d)) {
      Serial.print("Direction ");
      Serial.print(d);
      Serial.println(": BLOCKED");
      continue;
    }

    int nx = robotX + DX[d];
    int ny = robotY + DY[d];

    if (!inMaze(nx, ny)) {
      Serial.print("Direction ");
      Serial.print(d);
      Serial.println(": OUT OF MAZE");
      continue;
    }

    int floodVal = floodGrid[ny][nx];
    bool wasVisited = visitedGrid[ny][nx];

    if (floodVal == 255) {
      Serial.print("Direction ");
      Serial.print(d);
      Serial.println(": UNREACHABLE");
      continue;
    }

    Serial.print("Direction ");
    Serial.print(d);
    Serial.print(" -> (");
    Serial.print(nx);
    Serial.print(",");
    Serial.print(ny);
    Serial.print(") flood=");
    Serial.print(floodVal);
    Serial.print(" visited=");
    Serial.println(wasVisited ? "YES" : "NO");

    if (floodVal < bestFlood) {
      bestFlood = floodVal;
      bestVisited = wasVisited;
      bestDir = d;
    } else if (floodVal == bestFlood &&
               bestVisited &&
               !wasVisited) {
      bestVisited = false;
      bestDir = d;
    }
  }

  if (bestDir < 0) {
    Serial.println("No valid direction found");
  }

  return bestDir;
}

// solver
bool atTarget() {
  if (runMode == MODE_RETURN) {
    return robotX == startX && robotY == startY;
  }

  return isCenterGoal(robotX, robotY);
}

void logDecision(String d) {
  lastDecision = d;
  decisionSeq++;
}

// Start a new operating mode
void startMode(RunMode m) {
  abortFlag = false;

  if (m == MODE_SOLVE) {
    runMode = MODE_SOLVE;
    mazeMapped = false;

    statusText = "EXPLORING...";
    logDecision("START EXPLORE");
  }

  else if (m == MODE_RETURN) {
    runMode = MODE_RETURN;

    statusText = "RETURNING...";
    logDecision("START RETURN");
  }

  else if (m == MODE_SPEEDRUN) {
    if (!mazeMapped) {
      statusText = "MAP NOT READY!";
      logDecision("MAP NOT READY");
      runMode = MODE_IDLE;
      return;
    }

    // Robot must physically be at the starting cell
    if (robotX != startX || robotY != startY) {
      statusText = "ROBOT NOT AT START!";
      logDecision("NOT AT START");
      runMode = MODE_IDLE;
      return;
    }

    runMode = MODE_SPEEDRUN;

    statusText = "SPEED RUN!";
    logDecision("START SPEED RUN");
  }

  else {
    runMode = MODE_IDLE;
  }
}

// Stop all robot activity
void stopEverything() {
  abortFlag = true;
  continuousForward = false;
  motionActive = false;
  runMode = MODE_IDLE;

  stopMotorsOnly();

  statusText = "STOP";
  logDecision("STOP");
}

// Turn robot toward a required absolute direction
bool turnToDirection(int targetDir, bool speedRunMode) {
  int rel = (targetDir - robotDir + 4) & 3;

  if (rel == 0) {
    logDecision(
      speedRunMode ?
      "SPEED FORWARD" :
      "FORWARD"
    );

    return true;
  }

  if (rel == 1) {
    logDecision(
      speedRunMode ?
      "SPEED TURN RIGHT" :
      "TURN RIGHT"
    );

    turnRight90();
  }

  else if (rel == 3) {
    logDecision(
      speedRunMode ?
      "SPEED TURN LEFT" :
      "TURN LEFT"
    );

    turnLeft90();
  }

  else if (rel == 2) {
    logDecision(
      speedRunMode ?
      "SPEED TURN AROUND" :
      "TURN AROUND"
    );

    turn180();
  }

  if (abortFlag) {
    return false;
  }

  robotDir = targetDir;

  stopMotorsOnly();
  delay(100);

  return true;
}

// Move one cell in a selected absolute direction
bool moveToDirection(int targetDir, bool speedRunMode) {
  if (!turnToDirection(targetDir, speedRunMode)) {
    return false;
  }

  int nx = robotX + DX[targetDir];
  int ny = robotY + DY[targetDir];

  if (!inMaze(nx, ny)) {
    statusText = "OUT OF MAZE";
    logDecision("OUT OF MAZE");
    return false;
  }

  if (moveOneCell()) {
    robotX = nx;
    robotY = ny;

    visitedGrid[robotY][robotX] = true;

    Serial.print("Moved to (");
    Serial.print(robotX);
    Serial.print(",");
    Serial.print(robotY);
    Serial.println(")");

    return true;
  }

  if (!abortFlag) {

    setWall(robotX, robotY, targetDir);

    statusText =
      speedRunMode ?
      "SPEED RUN FAILED" :
      "WALL FOUND LATE";

    logDecision(
      speedRunMode ?
      "SPEED RUN FAILED" :
      "WALL FOUND LATE"
    );
  }

  return false;
}

// Perform one solver operation
void solverStep() {
  if (atTarget()) {
    stopMotorsOnly();

    // Robot returned physically to start
    if (runMode == MODE_RETURN) {
      robotDir = startDir;

      statusText = "HOME!";
      logDecision("HOME");

      runMode = MODE_IDLE;
      return;
    }

    // Robot reached maze center during exploration
    if (runMode == MODE_SOLVE) {
      mazeMapped = true;

      statusText = "CENTER REACHED!";
      logDecision("CENTER REACHED");

      findShortestPath();

      statusText = "RETURN HOME BEFORE SPEED RUN";
      logDecision("PATH READY");

      runMode = MODE_IDLE;
      return;
    }

    // Speed run completed
    if (runMode == MODE_SPEEDRUN) {
      statusText = "SPEED RUN COMPLETE!";
      logDecision("SPEED RUN COMPLETE");

      runMode = MODE_IDLE;
      return;
    }
  }

  // exploration mood
  if (runMode == MODE_SOLVE) {
    senseAndUpdateWalls();

    // Calculate flood values toward maze center
    floodFill(false, false);

    if (floodGrid[robotY][robotX] == 255) {
      stopMotorsOnly();

      statusText = "NO PATH TO CENTER";
      logDecision("NO PATH");

      runMode = MODE_IDLE;
      return;
    }

    int bestDir = getBestDirection();

    if (bestDir < 0) {
      stopMotorsOnly();

      statusText = "STUCK";
      logDecision("STUCK");

      runMode = MODE_IDLE;
      return;
    }

    Serial.print("BEST DIRECTION: ");
    Serial.println(bestDir);

    if (!moveToDirection(bestDir, false)) {
      if (!abortFlag) {
        statusText = "RECALCULATING";
      }

      return;
    }

    delay(100);
    return;
  }

  // return mode
  if (runMode == MODE_RETURN) {
    senseAndUpdateWalls();
    floodFill(true, true);

    if (floodGrid[robotY][robotX] == 255) {
      stopMotorsOnly();

      statusText = "NO RETURN PATH";
      logDecision("NO RETURN PATH");

      runMode = MODE_IDLE;
      return;
    }

    int bestDir = getBestDirection();

    if (bestDir < 0) {
      stopMotorsOnly();

      statusText = "RETURN STUCK";
      logDecision("RETURN STUCK");

      runMode = MODE_IDLE;
      return;
    }

    if (!moveToDirection(bestDir, false)) {
      if (!abortFlag) {
        statusText = "RETURN RECALCULATING";
      }

      return;
    }

    delay(100);
    return;
  }

  // speed run mode
  if (runMode == MODE_SPEEDRUN) {
    if (!mazeMapped) {
      statusText = "MAP NOT READY!";
      logDecision("MAP NOT READY");

      runMode = MODE_IDLE;
      return;
    }

    if (!inMaze(robotX, robotY)) {
      statusText = "INVALID POSITION";
      logDecision("INVALID POSITION");

      runMode = MODE_IDLE;
      return;
    }

    int nextDir = pathGrid[robotY][robotX];

    // 255 means the path ends at this cell
    if (nextDir == 255) {
      stopMotorsOnly();

      if (isCenterGoal(robotX, robotY)) {
        statusText = "SPEED RUN COMPLETE!";
        logDecision("SPEED RUN COMPLETE");
      } else {
        statusText = "INVALID SPEED PATH";
        logDecision("INVALID SPEED PATH");
      }

      runMode = MODE_IDLE;
      return;
    }

    if (nextDir < 0 || nextDir > 3) {
      stopMotorsOnly();

      statusText = "INVALID DIRECTION";
      logDecision("INVALID DIRECTION");

      runMode = MODE_IDLE;
      return;
    }

    if (!moveToDirection(nextDir, true)) {
      if (!abortFlag) {
        statusText = "SPEED RUN FAILED";
        logDecision("SPEED RUN FAILED");
        runMode = MODE_IDLE;
      }

      return;
    }

    delay(100);
    return;
  }
}

// ===== WEB INTERFACE =====
void handleRoot() {
  String page;
  page.reserve(24000);

  page = R"TOOTsSYS(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>TOOts sys | Micromouse Control</title>
  <style>
    :root {
      --pink-900:#7a164d;
      --pink-700:#b72c73;
      --pink-600:#d6418d;
      --pink-500:#ec5aa5;
      --pink-300:#f6a7cf;
      --pink-100:#ffe5f2;
      --pink-050:#fff7fb;
      --ink:#2d1b26;
      --muted:#78616f;
      --white:#ffffff;
      --success:#18866b;
      --danger:#d63055;
      --warning:#be6b00;
      --shadow:0 10px 28px rgba(122,22,77,.12);
    }

    *{box-sizing:border-box}
    body{
      margin:0;
      min-height:100vh;
      font-family:Arial,Helvetica,sans-serif;
      background:linear-gradient(145deg,var(--pink-050),var(--pink-100));
      color:var(--ink);
    }

    .topbar{
      position:sticky;
      top:0;
      z-index:20;
      display:flex;
      align-items:center;
      justify-content:space-between;
      gap:16px;
      padding:14px 20px;
      color:var(--white);
      background:linear-gradient(110deg,var(--pink-900),var(--pink-600));
      box-shadow:0 5px 18px rgba(122,22,77,.22);
    }

    .brand{display:flex;align-items:center;gap:12px;text-align:left}
    .brand-mark{
      width:42px;height:42px;border-radius:13px;
      display:grid;place-items:center;
      background:rgba(255,255,255,.18);
      border:1px solid rgba(255,255,255,.35);
      font-size:22px;
    }
    .brand h1{font-size:21px;margin:0;letter-spacing:.3px}
    .brand p{font-size:12px;margin:3px 0 0;opacity:.82}
    .connection{font-size:12px;font-weight:700;padding:8px 11px;border-radius:999px;background:rgba(255,255,255,.17)}

    .container{width:min(1100px,calc(100% - 24px));margin:18px auto 36px}
    .hero{
      display:grid;
      grid-template-columns:1.35fr .65fr;
      gap:14px;
      margin-bottom:14px;
    }

    .card{
      background:rgba(255,255,255,.94);
      border:1px solid rgba(214,65,141,.17);
      border-radius:18px;
      padding:17px;
      box-shadow:var(--shadow);
    }

    .status-card{display:flex;flex-direction:column;justify-content:center;min-height:150px;text-align:left}
    .eyebrow{font-size:12px;color:var(--pink-700);font-weight:800;text-transform:uppercase;letter-spacing:1px}
    .status{font-size:28px;font-weight:800;color:var(--pink-900);margin:7px 0 10px;word-break:break-word}
    .subline{font-size:13px;color:var(--muted);line-height:1.5}

    .phase-card{text-align:center;display:flex;flex-direction:column;align-items:center;justify-content:center}
    .phase-badge{display:inline-flex;align-items:center;justify-content:center;padding:9px 14px;border-radius:999px;font-size:13px;font-weight:800}
    .phase-ready{background:var(--pink-100);color:var(--pink-900)}
    .phase-explore{background:#e8efff;color:#315ba8}
    .phase-return{background:#fff0d8;color:#9b5900}
    .phase-speed{background:#e2f7ef;color:#176d57}
    .maze-size{font-size:14px;color:var(--muted);margin-top:12px}

    .section-title{display:flex;align-items:center;justify-content:space-between;gap:10px;margin:0 0 13px;text-align:left}
    .section-title h2{font-size:17px;margin:0;color:var(--pink-900)}
    .section-title span{font-size:12px;color:var(--muted)}

    .sensor-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}
    .sensor{
      text-align:center;
      padding:14px 8px;
      border-radius:15px;
      background:var(--pink-050);
      border:1px solid var(--pink-100);
    }
    .sensor-label{font-size:11px;font-weight:800;color:var(--muted);letter-spacing:.7px}
    .sensor-value{font-size:25px;font-weight:800;color:var(--pink-700);margin:6px 0}
    .wall-indicator{display:inline-block;padding:4px 9px;border-radius:999px;font-size:11px;font-weight:800}
    .wall-yes{background:#ffe0e7;color:var(--danger)}
    .wall-no{background:#ddf5ed;color:var(--success)}

    .stats{display:grid;grid-template-columns:repeat(6,1fr);gap:8px;margin-top:12px}
    .stat{background:#fff;border:1px solid var(--pink-100);border-radius:13px;padding:10px 7px;text-align:center}
    .stat small{display:block;color:var(--muted);font-size:10px;margin-bottom:5px}
    .stat strong{font-size:14px;color:var(--ink);word-break:break-word}
    .mapped{margin-top:11px;font-size:12px;color:var(--muted);text-align:left}
    .progress{height:8px;background:var(--pink-100);border-radius:999px;overflow:hidden;margin-top:6px}
    .progress-fill{height:100%;width:0;background:linear-gradient(90deg,var(--pink-500),var(--pink-900));transition:width .25s}

    .control-grid{display:grid;grid-template-columns:1fr 1fr;gap:14px;margin-top:14px}
    .button-grid{display:grid;grid-template-columns:repeat(2,1fr);gap:9px}
    button{
      border:0;border-radius:12px;padding:12px 10px;font-size:14px;font-weight:800;cursor:pointer;
      transition:transform .12s,box-shadow .12s,opacity .12s;
    }
    button:hover{transform:translateY(-1px);box-shadow:0 6px 14px rgba(122,22,77,.13)}
    button:active{transform:translateY(0)}
    .primary{background:linear-gradient(110deg,var(--pink-700),var(--pink-500));color:#fff}
    .secondary{background:var(--pink-100);color:var(--pink-900)}
    .neutral{background:#f4edf1;color:#59434f}
    .danger{background:var(--danger);color:#fff}
    .success{background:var(--success);color:#fff}
    .warning{background:#f8d9ac;color:#7b4500}
    .wide{grid-column:1/-1}

    details{margin-top:14px}
    summary{
      list-style:none;cursor:pointer;font-weight:800;color:var(--pink-900);
      display:flex;justify-content:space-between;align-items:center;
    }
    summary::-webkit-details-marker{display:none}
    summary::after{content:'+';font-size:22px;color:var(--pink-600)}
    details[open] summary::after{content:'−'}
    .settings-grid{display:grid;grid-template-columns:repeat(2,1fr);gap:14px;margin-top:16px}
    .settings-group{background:var(--pink-050);border:1px solid var(--pink-100);border-radius:15px;padding:14px}
    .settings-group h3{font-size:14px;color:var(--pink-900);margin:0 0 13px}
    .setting{margin:0 0 15px}
    .setting:last-child{margin-bottom:0}
    .setting-label{display:flex;justify-content:space-between;gap:8px;font-size:12px;font-weight:700;color:var(--muted);margin-bottom:7px}
    .setting-value{color:var(--pink-700)}
    input[type=range]{width:100%;accent-color:var(--pink-600);cursor:pointer}
    .hint{font-size:11px;line-height:1.5;color:var(--muted);margin-top:13px;padding:10px;background:#fff;border-radius:11px}

    .footer{text-align:center;color:var(--muted);font-size:11px;margin-top:18px}
    .toast{
      position:fixed;right:16px;bottom:16px;z-index:30;
      max-width:300px;padding:11px 14px;border-radius:12px;
      background:var(--pink-900);color:#fff;font-size:13px;font-weight:700;
      opacity:0;transform:translateY(12px);pointer-events:none;transition:.2s;
      box-shadow:var(--shadow);
    }
    .toast.show{opacity:1;transform:translateY(0)}

    @media(max-width:760px){
      .hero,.control-grid,.settings-grid{grid-template-columns:1fr}
      .stats{grid-template-columns:repeat(3,1fr)}
      .topbar{padding:12px 14px}
      .connection{display:none}
    }
    @media(max-width:430px){
      .container{width:min(100% - 14px,1100px);margin-top:10px}
      .card{padding:13px;border-radius:15px}
      .sensor-grid{gap:6px}
      .sensor-value{font-size:20px}
      .button-grid{grid-template-columns:1fr}
      .wide{grid-column:auto}
      .stats{grid-template-columns:repeat(2,1fr)}
      .status{font-size:23px}
    }
  </style>
</head>
<body>
  <header class="topbar">
    <div class="brand">
      <div class="brand-mark">T</div>
      <div>
        <h1>TOOTs sys</h1>
        <p>Micromouse control center</p>
      </div>
    </div>
    <div class="connection" id="connection">Connecting...</div>
  </header>

  <main class="container">
    <section class="hero">
      <div class="card status-card">
        <div class="eyebrow">Robot status</div>
        <div class="status" id="status">Loading...</div>
        <div class="subline">Use the controls below to explore, return home, test movement, or run the saved path.</div>
      </div>
      <div class="card phase-card">
        <div id="phaseBadge" class="phase-badge phase-ready">READY</div>
        <div class="maze-size">Maze: )TOOTsSYS";

  page += String(MAZE);
  page += " × ";
  page += String(MAZE);

  page += R"TOOTsSYS( cells<br>Goal: dynamic center</div>
      </div>
    </section>

    <section class="card">
      <div class="section-title">
        <h2>Live sensors</h2>
        <span>Updated automatically</span>
      </div>

      <div class="sensor-grid">
        <div class="sensor">
          <div class="sensor-label">LEFT</div>
          <div class="sensor-value" id="leftTof">---</div>
          <div id="leftWall" class="wall-indicator wall-no">Clear</div>
        </div>
        <div class="sensor">
          <div class="sensor-label">FRONT</div>
          <div class="sensor-value" id="frontTof">---</div>
          <div id="frontWall" class="wall-indicator wall-no">Clear</div>
        </div>
        <div class="sensor">
          <div class="sensor-label">RIGHT</div>
          <div class="sensor-value" id="rightTof">---</div>
          <div id="rightWall" class="wall-indicator wall-no">Clear</div>
        </div>
      </div>

      <div class="stats">
        <div class="stat"><small>Position</small><strong id="pos">-</strong></div>
        <div class="stat"><small>Heading</small><strong id="head">-</strong></div>
        <div class="stat"><small>Mode</small><strong id="mode">-</strong></div>
        <div class="stat"><small>Left encoder</small><strong id="lenc">0</strong></div>
        <div class="stat"><small>Right encoder</small><strong id="renc">0</strong></div>
        <div class="stat"><small>Last decision</small><strong id="decision">-</strong></div>
      </div>

      <div class="mapped">Mapped cells: <b><span id="mappedCount">0</span>/)TOOTsSYS";

  page += String(MAZE * MAZE);

  page += R"TOOTsSYS(</b>
        <div class="progress"><div class="progress-fill" id="mapProgress"></div></div>
      </div>
    </section>

    <section class="control-grid">
      <div class="card">
        <div class="section-title"><h2>Maze operations</h2><span>Main workflow</span></div>
        <div class="button-grid">
          <button class="primary" onclick="sendCmd('solve')">Start explore</button>
          <button class="success" onclick="sendCmd('speedrun')">Start speed run</button>
          <button class="warning" onclick="sendCmd('return')">Return home</button>
          <button class="danger" onclick="sendCmd('s')">Emergency stop</button>
          <button class="neutral" onclick="sendCmd('resetmaze')">Reset maze</button>
          <button class="neutral" onclick="sendCmd('resetpose')">Reset pose</button>
        </div>
      </div>

      <div class="card">
        <div class="section-title"><h2>Manual testing</h2><span>Calibration tools</span></div>
        <div class="button-grid">
          <button class="primary" onclick="sendCmd('startfwd')">Move forward</button>
          <button class="danger" onclick="sendCmd('stopfwd')">Stop forward</button>
          <button class="secondary" onclick="sendCmd('celltof')">Move one cell</button>
          <button class="neutral" onclick="sendCmd('reset')">Reset encoders</button>
          <button class="secondary" onclick="sendCmd('left90')">Turn left 90°</button>
          <button class="secondary" onclick="sendCmd('right90')">Turn right 90°</button>
          <button class="secondary wide" onclick="sendCmd('turn180')">Turn 180°</button>
        </div>
      </div>
    </section>

    <details class="card">
      <summary>Calibration settings</summary>
      <div class="settings-grid">
        <div class="settings-group">
          <h3>Speed and movement</h3>
          <div class="setting">
            <div class="setting-label"><span>Base speed</span><span class="setting-value" id="baseSpeedVal">150</span></div>
            <input type="range" min="60" max="255" step="1" id="baseSpeed" oninput="queueSet('baseSpeed',this.value)">
          </div>
          <div class="setting">
            <div class="setting-label"><span>Minimum speed</span><span class="setting-value" id="minSpeedVal">100</span></div>
            <input type="range" min="30" max="150" step="1" id="minSpeed" oninput="queueSet('minSpeed',this.value)">
          </div>
          <div class="setting">
            <div class="setting-label"><span>Turn speed</span><span class="setting-value" id="turnSpeedVal">140</span></div>
            <input type="range" min="60" max="255" step="1" id="turnSpeed" oninput="queueSet('turnSpeed',this.value)">
          </div>
          <div class="setting">
            <div class="setting-label"><span>Base correction</span><span class="setting-value" id="baseCorrVal">3</span></div>
            <input type="range" min="-20" max="20" step="1" id="baseCorr" oninput="queueSet('baseCorr',this.value)">
          </div>
        </div>

        <div class="settings-group">
          <h3>Turn timing</h3>
          <div class="setting">
            <div class="setting-label"><span>Left turn</span><span><span class="setting-value" id="leftDelayVal">345</span> ms</span></div>
            <input type="range" min="200" max="700" step="5" id="leftDelay" oninput="queueSet('leftDelay',this.value)">
          </div>
          <div class="setting">
            <div class="setting-label"><span>Right turn</span><span><span class="setting-value" id="rightDelayVal">325</span> ms</span></div>
            <input type="range" min="200" max="700" step="5" id="rightDelay" oninput="queueSet('rightDelay',this.value)">
          </div>
          <div class="setting">
            <div class="setting-label"><span>180° turn</span><span><span class="setting-value" id="turn180DelayVal">600</span> ms</span></div>
            <input type="range" min="400" max="1000" step="5" id="turn180Delay" oninput="queueSet('turn180Delay',this.value)">
          </div>
        </div>

        <div class="settings-group">
          <h3>Cell and sensors</h3>
          <div class="setting">
            <div class="setting-label"><span>Cell ticks</span><span class="setting-value" id="cellTicksVal">340</span></div>
            <input type="range" min="50" max="600" step="5" id="cellTicks" oninput="queueSet('cellTicks',this.value)">
          </div>
          <div class="setting">
            <div class="setting-label"><span>Left offset</span><span><span class="setting-value" id="leftOffsetVal">-70</span> mm</span></div>
            <input type="range" min="-120" max="0" step="1" id="leftOffset" oninput="queueSet('leftOffset',this.value)">
          </div>
          <div class="setting">
            <div class="setting-label"><span>Right offset</span><span><span class="setting-value" id="rightOffsetVal">-10</span> mm</span></div>
            <input type="range" min="-120" max="0" step="1" id="rightOffset" oninput="queueSet('rightOffset',this.value)">
          </div>
          <div class="setting">
            <div class="setting-label"><span>Side target</span><span><span class="setting-value" id="sideTargetVal">45</span> mm</span></div>
            <input type="range" min="20" max="100" step="1" id="sideTarget" oninput="queueSet('sideTarget',this.value)">
          </div>
          <div class="setting">
            <div class="setting-label"><span>Wall correction strength</span><span class="setting-value" id="wallStrVal">2</span></div>
            <input type="range" min="1" max="10" step="1" id="wallStr" oninput="queueSet('wallStr',this.value)">
          </div>
          <div class="setting">
            <div class="setting-label"><span>Wall threshold</span><span><span class="setting-value" id="wallThreshVal">200</span> mm</span></div>
            <input type="range" min="100" max="400" step="5" id="wallThresh" oninput="queueSet('wallThresh',this.value)">
          </div>
          <div class="setting">
            <div class="setting-label"><span>Front stop distance</span><span><span class="setting-value" id="frontStopVal">100</span> mm</span></div>
            <input type="range" min="40" max="200" step="5" id="frontStop" oninput="queueSet('frontStop',this.value)">
          </div>
        </div>

        <div class="settings-group">
          <h3>PID controller</h3>
          <div class="setting">
            <div class="setting-label"><span>Kp</span><span class="setting-value" id="kpVal">3.0</span></div>
            <input type="range" min="0.5" max="10" step="0.1" id="kp" oninput="queueSet('kp',this.value)">
          </div>
          <div class="setting">
            <div class="setting-label"><span>Ki</span><span class="setting-value" id="kiVal">0.005</span></div>
            <input type="range" min="0.001" max="0.02" step="0.001" id="ki" oninput="queueSet('ki',this.value)">
          </div>
          <div class="setting">
            <div class="setting-label"><span>Kd</span><span class="setting-value" id="kdVal">0.5</span></div>
            <input type="range" min="0.1" max="2.0" step="0.1" id="kd" oninput="queueSet('kd',this.value)">
          </div>
          <div class="hint">Explore first, return the physical robot to the start, then begin the speed run. Change one calibration value at a time.</div>
        </div>
      </div>
    </details>

    <div class="footer">TOOTs sys · ESP32 Micromouse dashboard</div>
  </main>

  <div class="toast" id="toast"></div>

  <script>
    const totalCells = )TOOTsSYS";

  page += String(MAZE * MAZE);

  page += R"TOOTsSYS(;
    let updateBusy = false;
    const setTimers = {};

    function showToast(message){
      const toast = document.getElementById('toast');
      toast.textContent = message;
      toast.classList.add('show');
      clearTimeout(showToast.timer);
      showToast.timer = setTimeout(() => toast.classList.remove('show'), 1800);
    }

    function sendCmd(cmd){
      document.getElementById('status').textContent = 'Sending command...';
      fetch('/cmd?c=' + encodeURIComponent(cmd))
        .then(r => r.text())
        .then(text => {
          document.getElementById('status').textContent = text;
          showToast(text);
        })
        .catch(() => {
          document.getElementById('status').textContent = 'Connection error';
          showToast('Could not reach the robot');
        });
    }

    function queueSet(key,value){
      const label = document.getElementById(key + 'Val');
      if(label) label.textContent = value;
      clearTimeout(setTimers[key]);
      setTimers[key] = setTimeout(() => sendSet(key,value), 140);
    }

    function sendSet(key,value){
      fetch('/set?k=' + encodeURIComponent(key) + '&v=' + encodeURIComponent(value))
        .catch(() => showToast('Setting was not saved'));
    }

    function syncControl(id,value){
      const control = document.getElementById(id);
      const label = document.getElementById(id + 'Val');
      if(control && document.activeElement !== control) control.value = value;
      if(label) label.textContent = value;
    }

    function setWallState(elementId,hasWall){
      const element = document.getElementById(elementId);
      element.textContent = hasWall ? 'Wall' : 'Clear';
      element.className = 'wall-indicator ' + (hasWall ? 'wall-yes' : 'wall-no');
    }

    function update(){
      if(updateBusy) return;
      updateBusy = true;

      fetch('/data')
        .then(r => {
          if(!r.ok) throw new Error('Bad response');
          return r.json();
        })
        .then(d => {
          document.getElementById('connection').textContent = 'Connected';
          if(d.status) document.getElementById('status').textContent = d.status;

          document.getElementById('leftTof').textContent = d.leftTof + ' mm';
          document.getElementById('frontTof').textContent = d.frontTof + ' mm';
          document.getElementById('rightTof').textContent = d.rightTof + ' mm';
          document.getElementById('mappedCount').textContent = d.mapped || 0;
          document.getElementById('mapProgress').style.width = Math.min(100,((d.mapped || 0)/totalCells)*100) + '%';
          document.getElementById('pos').textContent = '(' + d.x + ',' + d.y + ')';
          document.getElementById('head').textContent = ['North','East','South','West'][d.dir] || '-';
          document.getElementById('mode').textContent = ['Idle','Explore','Return','Speed run'][d.mode] || '-';
          document.getElementById('lenc').textContent = d.lenc;
          document.getElementById('renc').textContent = d.renc;
          document.getElementById('decision').textContent = d.decision || '-';

          syncControl('baseSpeed',d.baseSpeed);
          syncControl('turnSpeed',d.turnSpeed);
          syncControl('minSpeed',d.minSpeed);
          syncControl('baseCorr',d.baseCorr);
          syncControl('leftDelay',d.leftDelay);
          syncControl('rightDelay',d.rightDelay);
          syncControl('turn180Delay',d.turn180Delay);
          syncControl('cellTicks',d.cellTicks);
          syncControl('leftOffset',d.leftOffset);
          syncControl('rightOffset',d.rightOffset);
          syncControl('sideTarget',d.sideTarget);
          syncControl('wallStr',d.wallStr);
          syncControl('wallThresh',d.wallThresh);
          syncControl('frontStop',d.frontStop);
          syncControl('kp',d.kp);
          syncControl('ki',d.ki);
          syncControl('kd',d.kd);

          const badge = document.getElementById('phaseBadge');
          if(d.mode === 1){
            badge.textContent = 'EXPLORING';
            badge.className = 'phase-badge phase-explore';
          }else if(d.mode === 2){
            badge.textContent = 'RETURNING';
            badge.className = 'phase-badge phase-return';
          }else if(d.mode === 3){
            badge.textContent = 'SPEED RUN';
            badge.className = 'phase-badge phase-speed';
          }else{
            badge.textContent = 'READY';
            badge.className = 'phase-badge phase-ready';
          }

          const threshold = d.wallThresh;
          setWallState('leftWall',d.leftTof < threshold);
          setWallState('frontWall',d.frontTof < threshold);
          setWallState('rightWall',d.rightTof < threshold);
        })
        .catch(() => {
          document.getElementById('connection').textContent = 'Disconnected';
        })
        .finally(() => updateBusy = false);
    }

    setInterval(update,500);
    update();
  </script>
</body>
</html>
)TOOTsSYS";

  server.send(200, "text/html", page);
}

void handleData() {
  readToFs();
  int mapped = getMappedCount();

  String json = "{";
  json += "\"status\":\"" + statusText + "\",";
  json += "\"decision\":\"" + lastDecision + "\",";
  json += "\"seq\":" + String(decisionSeq) + ",";
  json += "\"mode\":" + String((int)runMode) + ",";
  json += "\"x\":" + String(robotX) + ",";
  json += "\"y\":" + String(robotY) + ",";
  json += "\"dir\":" + String(robotDir) + ",";
  json += "\"lenc\":" + String(leftTicks) + ",";
  json += "\"renc\":" + String(rightTicks) + ",";
  json += "\"leftTof\":" + String(leftDist) + ",";
  json += "\"frontTof\":" + String(frontDist) + ",";
  json += "\"rightTof\":" + String(rightDist) + ",";
  json += "\"mapped\":" + String(mapped) + ",";
  json += "\"baseSpeed\":" + String(BASE_SPEED) + ",";
  json += "\"turnSpeed\":" + String(TURN_SPEED) + ",";
  json += "\"minSpeed\":" + String(MIN_SPEED) + ",";
  json += "\"baseCorr\":" + String(BASE_CORRECTION) + ",";
  json += "\"leftDelay\":" + String(TURN_DELAY_LEFT) + ",";
  json += "\"rightDelay\":" + String(TURN_DELAY_RIGHT) + ",";
  json += "\"turn180Delay\":" + String(TURN_DELAY_180) + ",";
  json += "\"cellTicks\":" + String(CELL_TICKS) + ",";
  json += "\"leftOffset\":" + String(LEFT_OFFSET) + ",";
  json += "\"rightOffset\":" + String(RIGHT_OFFSET) + ",";
  json += "\"sideTarget\":" + String(SIDE_TARGET_MM) + ",";
  json += "\"wallStr\":" + String(WALL_CORR_STRENGTH) + ",";
  json += "\"wallThresh\":" + String(WALL_THRESHOLD) + ",";
  json += "\"frontStop\":" + String(FRONT_STOP) + ",";
  json += "\"kp\":" + String(Kp) + ",";
  json += "\"ki\":" + String(Ki) + ",";
  json += "\"kd\":" + String(Kd);
  json += "}";
  server.send(200, "application/json", json);
}

void handleCmd() {
  String c = server.arg("c");
  Serial.println("=== CMD: " + c + " ===");

  if (c == "debug") {
    String info = "Status: " + statusText + "\n";
    info += "Mode: " + String((int)runMode) + "\n";
    info += "Mapped: " + String(mazeMapped ? "YES" : "NO") + "\n";
    info += "Pos: (" + String(robotX) + "," + String(robotY) + ")\n";
    info += "Dir: " + String(robotDir) + "\n";
    info += "Cells: " + String(getMappedCount()) + "/" + String(MAZE*MAZE);
    server.send(200, "text/plain", info);
    return;
  }

  if (c == "s") { stopEverything(); server.send(200, "text/plain", "STOPPED"); return; }
  if (c == "startfwd") {
    abortFlag = false;
    continuousForward = true;
    resetEncoders();
    statusText = "CONTINUOUS FWD";
    server.send(200, "text/plain", "FWD STARTED");
    return;
  }

  if (c == "stopfwd") { continuousForward = false; stopMotorsOnly(); statusText = "STOPPED"; server.send(200, "text/plain", "FWD STOPPED"); return; }
  if (c == "left90") { abortFlag = false; runMode = MODE_IDLE; turnLeft90(); server.send(200, "text/plain", "LEFT DONE"); return; }
  if (c == "right90") { abortFlag = false; runMode = MODE_IDLE; turnRight90(); server.send(200, "text/plain", "RIGHT DONE"); return; }
  if (c == "turn180") { abortFlag = false; runMode = MODE_IDLE; turn180(); server.send(200, "text/plain", "180 DONE"); return; }
  if (c == "celltof") { abortFlag = false; runMode = MODE_IDLE; if(moveOneCell()){int nx=robotX+DX[robotDir]; int ny=robotY+DY[robotDir]; if(inMaze(nx,ny)){robotX=nx; robotY=ny;}} server.send(200, "text/plain", "CELL DONE"); return; }
  if (c == "reset") { resetEncoders(); statusText = "ENCODERS RESET"; server.send(200, "text/plain", "RESET OK"); return; }
  if (c == "resetpose") { robotX = 0; robotY = 0; robotDir = 0; statusText = "POSE RESET"; server.send(200, "text/plain", "POSE RESET"); return; }
  if (c == "resetmaze") { stopEverything(); resetMaze(); server.send(200, "text/plain", "MAZE RESET"); return; }
  if (c == "solve") { mazeMapped = false; statusText = "STARTING EXPLORE"; startMode(MODE_SOLVE); server.send(200, "text/plain", "EXPLORING STARTED"); return; }
  if (c == "speedrun") {
    if (!mazeMapped) {
      statusText = "MAP NOT READY!";
      server.send(200, "text/plain", "MAP NOT READY - Explore first!");
      return;
    }

    if (robotX != startX || robotY != startY) {
      statusText = "RETURN HOME FIRST";
      server.send(200, "text/plain", "ROBOT NOT AT START - Return home first!");
      return;
    }

    startMode(MODE_SPEEDRUN);
    server.send(200, "text/plain", "SPEED RUN STARTED");
    return;
  }

  if (c == "return") { floodFill(true, false); startMode(MODE_RETURN); server.send(200, "text/plain", "RETURNING HOME"); return; }

  server.send(200, "text/plain", "Unknown: " + c);
}

void handleSet() {
  String k = server.arg("k");
  float v = server.arg("v").toFloat();

  Serial.println("Set: " + k + " = " + String(v));

  if (k == "baseSpeed") { BASE_SPEED = constrain((int)v, 60, 255); statusText = "Base: " + String(BASE_SPEED); }
  else if (k == "turnSpeed") { TURN_SPEED = constrain((int)v, 60, 255); statusText = "Turn: " + String(TURN_SPEED); }
  else if (k == "minSpeed") { MIN_SPEED = constrain((int)v, 30, 150); statusText = "Min: " + String(MIN_SPEED); }
  else if (k == "baseCorr") { BASE_CORRECTION = constrain((int)v, -20, 20); statusText = "Corr: " + String(BASE_CORRECTION); }
  else if (k == "leftDelay") { TURN_DELAY_LEFT = constrain((int)v, 200, 700); statusText = "Left: " + String(TURN_DELAY_LEFT); }
  else if (k == "rightDelay") { TURN_DELAY_RIGHT = constrain((int)v, 200, 700); statusText = "Right: " + String(TURN_DELAY_RIGHT); }
  else if (k == "turn180Delay") { TURN_DELAY_180 = constrain((int)v, 400, 1000); statusText = "180: " + String(TURN_DELAY_180); }
  else if (k == "cellTicks") { CELL_TICKS = constrain((long)v, 50, 600); statusText = "Cell: " + String(CELL_TICKS); }
  else if (k == "leftOffset") { LEFT_OFFSET = constrain((int)v, -120, 0); statusText = "L offset: " + String(LEFT_OFFSET); }
  else if (k == "rightOffset") { RIGHT_OFFSET = constrain((int)v, -120, 0); statusText = "R offset: " + String(RIGHT_OFFSET); }
  else if (k == "sideTarget") { SIDE_TARGET_MM = constrain((int)v, 20, 100); statusText = "Target: " + String(SIDE_TARGET_MM); }
  else if (k == "wallStr") { WALL_CORR_STRENGTH = constrain((int)v, 1, 10); statusText = "Strength: " + String(WALL_CORR_STRENGTH); }
  else if (k == "wallThresh") { WALL_THRESHOLD = constrain((int)v, 100, 400); statusText = "Threshold: " + String(WALL_THRESHOLD); }
  else if (k == "frontStop") { FRONT_STOP = constrain((int)v, 40, 200); statusText = "Front: " + String(FRONT_STOP); }
  else if (k == "kp") { Kp = constrain(v, 0.5, 10.0); statusText = "Kp: " + String(Kp); }
  else if (k == "ki") { Ki = constrain(v, 0.001, 0.02); statusText = "Ki: " + String(Ki); }
  else if (k == "kd") { Kd = constrain(v, 0.1, 2.0); statusText = "Kd: " + String(Kd); }

  server.send(200, "text/plain", "OK");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("========================================");
  Serial.println("=== BISMILLAH - MICROMOUSE STARTING ===");
  Serial.println("========================================");

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotorsOnly();

  pinMode(ENC_LEFT_C1,  INPUT);
  pinMode(ENC_LEFT_C2,  INPUT);
  pinMode(ENC_RIGHT_C1, INPUT);
  pinMode(ENC_RIGHT_C2, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_C1),  leftEncoderISR,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_C1), rightEncoderISR, RISING);

  Wire.begin(TOF_SDA, TOF_SCL);
  setupToFs();
  resetMaze();

  WiFiManager wm;

  // Uncomment the next line to force the setup portal again on next boot
  // (useful while testing, or give this as instructions for changing WiFi):
  // wm.resetSettings();

  wm.setConfigPortalTimeout(180); // give up and retry boot after 3 min if nobody configures it

  bool wifiOk = wm.autoConnect(AP_SETUP_NAME);

  if (!wifiOk) {
    Serial.println("WiFi Failed! Restarting to try again...");
    delay(2000);
    ESP.restart();
  } else {
    Serial.println("WiFi Connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  }

  if (MDNS.begin(MDNS_HOSTNAME)) {
    Serial.println("mDNS responder started: http://" + String(MDNS_HOSTNAME) + ".local");
  } else {
    Serial.println("mDNS setup failed");
  }

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/cmd", handleCmd);
  server.on("/set", handleSet);
  server.begin();

  Serial.println("Server Started!");
  Serial.println("========================================");
  Serial.println("Open: http://" + WiFi.localIP().toString());
  Serial.println("Or:   http://" + String(MDNS_HOSTNAME) + ".local");
  Serial.println("========================================");
  Serial.println();
  Serial.println("Press 'EXPLORE' to map the maze");
  Serial.println("Press 'SPEED RUN' after mapping");
  Serial.println("========================================");

  statusText = "READY";
}

void loop() {
  server.handleClient();
  if (runMode != MODE_IDLE && !abortFlag) {
    solverStep();
  }
  delay(20);
}
