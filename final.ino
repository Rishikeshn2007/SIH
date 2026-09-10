/*
 * =====================================================================================
 * Autonomous 4WD Skid-Steer UGV Grid Navigation Controller (SIH-26126)
 * File: final.ino
 * Target Hardware: ESP8266 NodeMCU + L298N Motor Driver + MPU6050 (MPU6050_light)
 * =====================================================================================
 * 
 * STRICT CALIBRATION PARAMETERS (From esp_navigation_controller.ino):
 * ------------------------------------------------------------------
 * • BASE_PWM             : 130  (Drive PWM)
 * • CORRECTION_PWM        : 140  (Turn pulse correction breakaway torque)
 * • CELL_MS              : 776  ms (30 cm translation per cell)
 * • TURN_90_MS           : 1150 ms (Base 90-degree turn duration)
 * • GYRO_SIGN            : 1.0f (Configurable gyro sign convention)
 * • KP_STRAIGHT          : 1.8f (Proportional gain for straight line heading drift)
 * • MAX_STRAIGHT_CORR    : 15   (Max PWM trim; restricts motor speeds to 115..145)
 * • STRAIGHT_CORR_SIGN   : 1.0f
 * • TURN_DEADBAND_DEG    : 1.8f deg (Turn deadband)
 * • MAX_CORRECTION_STEPS : 5 attempts
 * • MIN_CORRECTION_MS    : 65 ms
 * • MAX_CORRECTION_MS    : 240 ms
 * • MS_PER_DEGREE        : 12.5f ms/deg
 * • SETTLE_DELAY_MS      : 130 ms
 * 
 * MOTOR PINS & VERIFIED POLARITIES:
 * ---------------------------------
 * Left Motors:
 *   ENA = GPIO14 (D5)  |  IN1 = GPIO5 (D1)  |  IN2 = GPIO4 (D2)
 *   Left Forward : IN1=HIGH, IN2=LOW  |  Left Backward : IN1=LOW, IN2=HIGH
 * Right Motors:
 *   ENB = GPIO15 (D8)  |  IN3 = GPIO12 (D6) |  IN4 = GPIO13 (D7)
 *   Right Forward: IN3=LOW,  IN4=HIGH |  Right Backward: IN3=HIGH, IN4=LOW
 * 
 * I2C MPU6050:
 *   SDA = GPIO0 (D3)   |  SCL = GPIO2 (D4)  | VCC = 3.3V | GND = GND
 * 
 * INTEGRATION MODES:
 * ------------------
 * 1. SERVER CLOSED-LOOP (Flask server.py at http://172.21.3.69:5000/car/state)
 *    - Posts JSON telemetry { x, y, heading, running }
 *    - Receives & executes {"command", "speed", "turn_angle", "next_x", "next_y"}
 * 2. STANDALONE SERIAL CLI (Works anytime via Serial Monitor at 115200 baud):
 *    - S : Emergency Stop
 *    - Z : Reset State
 *    - P : Print Status & Heading
 *    - L : Turn Left 90 deg (Hybrid Corrected)
 *    - D : Turn Right 90 deg (Hybrid Corrected)
 *    - U : Turn 180 deg (Two 90s Hybrid Corrected)
 *    - G X Y : Move to Target Cell (e.g. G 6 1)
 *    - +, -, T <ms> : Tune base turn duration live
 * =====================================================================================
 */

#include <Wire.h>
#include <MPU6050_light.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>

// =====================================================================================
// 1. PIN CONFIGURATION
// =====================================================================================

// Left Motors (L298N)
#define ENA_PIN 14  // GPIO14 / D5
#define IN1_PIN 5   // GPIO5  / D1
#define IN2_PIN 4   // GPIO4  / D2

// Right Motors (L298N)
#define ENB_PIN 15  // GPIO15 / D8
#define IN3_PIN 12  // GPIO12 / D6
#define IN4_PIN 13  // GPIO13 / D7

// I2C Pins for MPU6050
#define SDA_PIN 0   // GPIO0  / D3
#define SCL_PIN 2   // GPIO2  / D4

// =====================================================================================
// 2. CALIBRATION CONSTANTS (STRICTLY FROM esp_navigation_controller.ino)
// =====================================================================================

const int   BASE_PWM             = 130;   // Verified driving PWM
int         CORRECTION_PWM       = 140;   // PWM with adequate breakaway torque for skid-steer
const unsigned long CELL_MS      = 776;   // 776 ms = 30 cm cell translation at PWM 130
unsigned long TURN_90_MS         = 1150;  // Base 90-degree turn duration (tunable live)

// =====================================================================================
// 3. MPU6050 & HYBRID CORRECTION CONFIGURATION
// =====================================================================================

/*
 * GYRO_SIGN CONVENTION:
 * - Turning LEFT (CCW) -> increases angle (+).
 * - Turning RIGHT (CW) -> decreases angle (-).
 * If on your car Left turn decreases angle, change GYRO_SIGN to -1.0f.
 */
const float GYRO_SIGN            = 1.0f;

// Straight line drift compensation (locks to ideal grid heading)
const float KP_STRAIGHT          = 1.8f;  // Proportional gain for heading drift
const int   MAX_STRAIGHT_CORR    = 15;    // Max PWM trim (keeps base PWM within 115..145)
const float STRAIGHT_CORR_SIGN   = 1.0f;  // Set to -1.0f if steering corrects away from line

// Hybrid Turn Correction Parameters
const float TURN_DEADBAND_DEG    = 1.8f;  // Tightened deadband for high precision
const int   MAX_CORRECTION_STEPS = 5;     // Maximum correction attempts
const int   MIN_CORRECTION_MS    = 65;    // Minimum pulse to reliably overcome static friction
const int   MAX_CORRECTION_MS    = 240;   // Maximum single correction pulse
const float MS_PER_DEGREE        = 12.5f; // (1150 ms / 90 deg ≈ 12.78 ms/deg)
const int   SETTLE_DELAY_MS      = 130;   // Settling time after stopping before MPU read

// =====================================================================================
// 4. NETWORK & SERVER CONFIGURATION (PROJECT INTEGRATION)
// =====================================================================================

const bool  ENABLE_WIFI          = true;                     // Set false for purely offline Serial testing
const char* WIFI_SSID            = "GMU_Staff";              // Wi-Fi SSID
const char* WIFI_PASSWORD        = "GMU@2025@";              // Wi-Fi Password
const char* SERVER_URL           = "http://172.21.3.69:5000/car/state"; // Flask Server Endpoint

bool wifiConnected = false;
unsigned long lastServerPoll = 0;
const unsigned long SERVER_POLL_INTERVAL_MS = 350;           // Closed-loop polling rate

// =====================================================================================
// 5. OBJECTS & NAVIGATION STATE
// =====================================================================================

MPU6050 mpu(Wire);

// Relative zero offset for MPU yaw
float angleZeroOffset = 0.0f;

// Grid navigation state (Local Cartesian)
int posX    = 0;
int posY    = 0;
int heading = 0; // 0: +X, 1: +Y, 2: -X, 3: -Y

// Server heading convention: 0=North, 90=East(+X), 180=South(+Y), 270=West(-X)
int serverHeading = 90; // Default facing +X (East)
bool is_running   = true;

// Global cardinal heading angle tracker:
// Eliminates cumulative drift by locking each turn and straight segment to the ideal grid axis
float gridTargetAngle = 0.0f;

const char* HEADING_NAMES[] = { "+X", "+Y", "-X", "-Y" };

// Serial input buffer
String inputString = "";

// Forward declarations
void printPosition();
void stopMotors();

// =====================================================================================
// 6. LOW-LEVEL MOTOR DRIVER FUNCTIONS (PHYSICALLY VERIFIED POLARITIES)
// =====================================================================================

void setMotorPWM(int leftPWM, int rightPWM) {
  analogWrite(ENA_PIN, constrain(leftPWM, 0, 255));
  analogWrite(ENB_PIN, constrain(rightPWM, 0, 255));
}

void leftForward() {
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
}

void leftBackward() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
}

void rightForward() {
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);
}

void rightBackward() {
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
}

void stopMotors() {
  setMotorPWM(0, 0);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
}

void setMotorsForward() {
  leftForward();
  rightForward();
}

void setMotorsTurnLeft() {
  leftBackward();
  rightForward();
}

void setMotorsTurnRight() {
  leftForward();
  rightBackward();
}

// =====================================================================================
// 7. SENSOR HELPER FUNCTIONS
// =====================================================================================

// Returns calibrated, normalized Z angle in degrees
float getAngle() {
  mpu.update();
  return (GYRO_SIGN * mpu.getAngleZ()) - angleZeroOffset;
}

// Non-blocking delay that keeps MPU integration and background tasks alive
void delayWithMpu(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    mpu.update();
    delay(2);
    yield();
  }
}

// Zero current angle without full recalibration
void resetZeroAngle() {
  mpu.update();
  angleZeroOffset = GYRO_SIGN * mpu.getAngleZ();
}

// =====================================================================================
// 8. HYBRID TURN CONTROLLER (CARDINAL GRID LOCKED)
// =====================================================================================

/*
 * Executes turn directly towards the target cardinal grid angle.
 * Completely prevents cumulative error accumulation across successive turns.
 */
void executeHybridTurnToTarget(float targetAbsoluteAngle, const char* turnName) {
  Serial.println();
  Serial.println(F("===================================================="));
  Serial.print(F("COMMAND: ")); Serial.print(turnName);
  Serial.print(F(" | TARGET GRID ANGLE: ")); Serial.print(targetAbsoluteAngle, 2);
  Serial.println(F(" deg"));
  Serial.println(F("===================================================="));

  float startAngle = getAngle();
  float turnDelta = targetAbsoluteAngle - startAngle;

  Serial.print(F("START ANGLE           : "));
  Serial.print(startAngle, 2);
  Serial.println(F(" deg"));
  Serial.print(F("ROTATION NEEDED       : "));
  Serial.print(turnDelta, 2);
  Serial.println(F(" deg"));

  // --- STEP 1: BASE TIMED TURN ---
  setMotorPWM(BASE_PWM, BASE_PWM);
  if (turnDelta > 0.0f) {
    setMotorsTurnLeft();
  } else {
    setMotorsTurnRight();
  }

  // Base turn timing proportional to rotation needed
  unsigned long baseDuration = (unsigned long)((abs(turnDelta) / 90.0f) * TURN_90_MS);
  delayWithMpu(baseDuration);

  // Stop and let chassis inertia settle
  stopMotors();
  delayWithMpu(SETTLE_DELAY_MS);

  // --- STEP 2: MEASURE BASE TURN RESULT ---
  float baseAngle = getAngle();
  float actualDelta = baseAngle - startAngle;
  float error = targetAbsoluteAngle - baseAngle;

  Serial.print(F("BASE TURN DURATION    : "));
  Serial.print(baseDuration);
  Serial.println(F(" ms"));
  Serial.print(F("BASE TURN ANGLE       : "));
  Serial.print(baseAngle, 2);
  Serial.println(F(" deg"));
  Serial.print(F("ACTUAL BASE ROTATION  : "));
  Serial.print(actualDelta, 2);
  Serial.println(F(" deg"));
  Serial.print(F("RESIDUAL ERROR        : "));
  Serial.print(error, 2);
  Serial.print(F(" deg ("));
  if (abs(error) <= TURN_DEADBAND_DEG) {
    Serial.println(F("Within Deadband)"));
  } else if (error > 0.0f) {
    Serial.println(F("Needs CCW / LEFT correction)"));
  } else {
    Serial.println(F("Needs CW / RIGHT correction)"));
  }

  // --- STEP 3: HYBRID PULSE CORRECTION LOOP WITH ANTI-STICTION BOOST ---
  int attempt = 0;
  float lastAngle = baseAngle;
  while (attempt < MAX_CORRECTION_STEPS) {
    float currentAngle = getAngle();
    error = targetAbsoluteAngle - currentAngle;

    if (abs(error) <= TURN_DEADBAND_DEG) {
      Serial.print(F("--> DEADBAND ACHIEVED (Error: "));
      Serial.print(error, 2);
      Serial.println(F(" deg). Turn complete!"));
      break;
    }

    attempt++;

    // Calculate pulse duration proportional to error
    int pulseMs = (int)(abs(error) * MS_PER_DEGREE);
    pulseMs = constrain(pulseMs, MIN_CORRECTION_MS, MAX_CORRECTION_MS);

    // Stiction boost: If previous pulse barely moved vehicle (<0.8 deg), increase duration
    if (attempt > 1 && abs(currentAngle - lastAngle) < 0.8f) {
      pulseMs = constrain(pulseMs + 45, MIN_CORRECTION_MS, MAX_CORRECTION_MS);
      Serial.print(F("[STICTION BOOST] "));
    }
    lastAngle = currentAngle;

    Serial.print(F("[CORRECTION ")); Serial.print(attempt);
    Serial.print(F("] ERROR: ")); Serial.print(error, 2);
    Serial.print(F(" deg | DIR: "));

    setMotorPWM(CORRECTION_PWM, CORRECTION_PWM);
    if (error > 0.0f) {
      Serial.print(F("LEFT"));
      setMotorsTurnLeft();
    } else {
      Serial.print(F("RIGHT"));
      setMotorsTurnRight();
    }

    Serial.print(F(" | PULSE: "));
    Serial.print(pulseMs);
    Serial.println(F(" ms"));

    delayWithMpu(pulseMs);
    stopMotors();
    delayWithMpu(SETTLE_DELAY_MS);
  }

  // --- STEP 4: FINAL DIAGNOSTICS REPORT ---
  float finalAngle = getAngle();
  float totalRotation = finalAngle - startAngle;
  float finalError = targetAbsoluteAngle - finalAngle;

  Serial.println(F("----------------------------------------------------"));
  Serial.print(F("FINAL ANGLE           : "));
  Serial.print(finalAngle, 2);
  Serial.println(F(" deg"));
  Serial.print(F("TOTAL ACTUAL ROTATION : "));
  Serial.print(totalRotation, 2);
  Serial.println(F(" deg"));
  Serial.print(F("FINAL RESIDUAL ERROR  : "));
  Serial.print(finalError, 2);
  Serial.println(F(" deg"));
  Serial.print(F("CORRECTION ATTEMPTS   : "));
  Serial.println(attempt);
  Serial.println(F("===================================================="));
}

// Left 90° Turn (locks to exact +90 deg grid cardinal)
void turnLeft90() {
  gridTargetAngle += 90.0f;
  executeHybridTurnToTarget(gridTargetAngle, "L (LEFT 90)");
  heading = (heading + 1) % 4;
  serverHeading = (serverHeading - 90 + 360) % 360;
  printPosition();
}

// Right 90° Turn (locks to exact -90 deg grid cardinal)
void turnRight90() {
  gridTargetAngle -= 90.0f;
  executeHybridTurnToTarget(gridTargetAngle, "D (RIGHT 90)");
  heading = (heading + 3) % 4;
  serverHeading = (serverHeading + 90) % 360;
  printPosition();
}

// 180° Turn using two consecutive cardinal locked 90° turns
void turn180() {
  Serial.println(F(">>> EXECUTING 180 TURN (2 x 90 DEG HYBRID) <<<"));
  turnLeft90();
  delayWithMpu(100);
  turnLeft90();
}

// =====================================================================================
// 9. STRAIGHT LINE MOVEMENT (776 ms LOCKED TO GRID CARDINAL HEADING)
// =====================================================================================

void moveOneCellForward() {
  unsigned long startTime = millis();

  setMotorsForward();

  Serial.println();
  Serial.print(F(">>> MOVING 1 CELL (30cm) | HEADING: "));
  Serial.print(HEADING_NAMES[heading]);
  Serial.print(F(" | TARGET GRID ANGLE: "));
  Serial.print(gridTargetAngle, 2);
  Serial.print(F(" deg | CURRENT ANGLE: "));
  Serial.print(getAngle(), 2);
  Serial.println(F(" deg"));

  while (millis() - startTime < CELL_MS) {
    mpu.update();
    float currentAngle = getAngle();

    // Lock directly to ideal grid target angle to eliminate lateral drift
    float drift = currentAngle - gridTargetAngle;

    float correction = STRAIGHT_CORR_SIGN * drift * KP_STRAIGHT;
    correction = constrain(correction, -MAX_STRAIGHT_CORR, MAX_STRAIGHT_CORR);

    int leftPWM  = constrain(BASE_PWM + (int)correction, 0, 255);
    int rightPWM = constrain(BASE_PWM - (int)correction, 0, 255);

    setMotorPWM(leftPWM, rightPWM);

    delay(10);
    yield();
  }

  stopMotors();
  delayWithMpu(100);

  // Update coordinate according to heading if running standalone
  if (heading == 0)      posX++; // +X
  else if (heading == 1) posY++; // +Y
  else if (heading == 2) posX--; // -X
  else if (heading == 3) posY--; // -Y

  float endAngle = getAngle();
  Serial.print(F(">>> CELL MOVED. END ANGLE: "));
  Serial.print(endAngle, 2);
  Serial.print(F(" deg | RESIDUAL DRIFT FROM GRID: "));
  Serial.print(endAngle - gridTargetAngle, 2);
  Serial.println(F(" deg"));

  printPosition();
}

// =====================================================================================
// 10. GRID NAVIGATION LOGIC (G X Y)
// =====================================================================================

void turnToHeading(int desiredHeading) {
  desiredHeading = (desiredHeading % 4 + 4) % 4;
  int diff = (desiredHeading - heading + 4) % 4;

  if (diff == 0) {
    return; // Already facing target direction
  } else if (diff == 1) {
    turnLeft90();
  } else if (diff == 2) {
    turn180();
  } else if (diff == 3) {
    turnRight90();
  }
}

void navigateToGrid(int targetX, int targetY) {
  Serial.println();
  Serial.println(F("****************************************************"));
  Serial.print(F("NAVIGATE REQUEST: ("));
  Serial.print(posX); Serial.print(F(",")); Serial.print(posY);
  Serial.print(F(") -> ("));
  Serial.print(targetX); Serial.print(F(",")); Serial.print(targetY);
  Serial.println(F(")"));
  Serial.println(F("****************************************************"));

  // Step 1: Move along X axis
  if (targetX > posX) {
    turnToHeading(0); // Face +X
    while (posX < targetX) {
      moveOneCellForward();
    }
  } else if (targetX < posX) {
    turnToHeading(2); // Face -X
    while (posX > targetX) {
      moveOneCellForward();
    }
  }

  // Step 2: Move along Y axis
  if (targetY > posY) {
    turnToHeading(1); // Face +Y
    while (posY < targetY) {
      moveOneCellForward();
    }
  } else if (targetY < posY) {
    turnToHeading(3); // Face -Y
    while (posY > targetY) {
      moveOneCellForward();
    }
  }

  Serial.println();
  Serial.println(F(">>> TARGET GRID CELL REACHED! <<<"));
  printPosition();
}

// =====================================================================================
// 11. DIAGNOSTICS & STATUS PRINTING
// =====================================================================================

void printPosition() {
  float currentAng = getAngle();
  Serial.print(F("STATUS -> POSITION: ("));
  Serial.print(posX);
  Serial.print(F(", "));
  Serial.print(posY);
  Serial.print(F(") | HEADING: "));
  Serial.print(HEADING_NAMES[heading]);
  Serial.print(F(" (Idx: "));
  Serial.print(heading);
  Serial.print(F(", Server: "));
  Serial.print(serverHeading);
  Serial.print(F(" deg) | TARGET: "));
  Serial.print(gridTargetAngle, 1);
  Serial.print(F(" deg | MPU: "));
  Serial.print(currentAng, 2);
  Serial.print(F(" deg | DRIFT: "));
  Serial.print(currentAng - gridTargetAngle, 2);
  Serial.println(F(" deg"));
}

void resetState() {
  posX = 0;
  posY = 0;
  heading = 0;
  serverHeading = 90; // Default facing East (+X)
  gridTargetAngle = 0.0f;
  resetZeroAngle();
  stopMotors();

  Serial.println();
  Serial.println(F("===================================================="));
  Serial.println(F("STATE RESET: Position (0,0), Heading +X, Server 90 deg, Angle 0.0"));
  Serial.println(F("===================================================="));
  printPosition();
}

// =====================================================================================
// 12. FLASK SERVER CLOSED-LOOP COMMUNICATION
// =====================================================================================

void pollServerAndExecute() {
  if (!ENABLE_WIFI || !wifiConnected || WiFi.status() != WL_CONNECTED) {
    return;
  }

  WiFiClient client;
  HTTPClient http;

  if (!http.begin(client, SERVER_URL)) {
    return;
  }

  http.addHeader("Content-Type", "application/json");

  // Step 1: Transmit Current Closed-Loop State
#if ARDUINOJSON_VERSION_MAJOR >= 7
  JsonDocument txDoc;
#else
  StaticJsonDocument<128> txDoc;
#endif
  txDoc["x"]       = posX;
  txDoc["y"]       = posY;
  txDoc["heading"] = serverHeading;
  txDoc["running"] = is_running;

  String txPayload;
  serializeJson(txDoc, txPayload);

  int httpCode = http.POST(txPayload);

  // Step 2: Parse & Execute Server Movement Command
  if (httpCode == HTTP_CODE_OK) {
    String rxPayload = http.getString();

#if ARDUINOJSON_VERSION_MAJOR >= 7
    JsonDocument rxDoc;
#else
    StaticJsonDocument<256> rxDoc;
#endif
    DeserializationError error = deserializeJson(rxDoc, rxPayload);

    if (!error) {
      const char* command = rxDoc["command"] | "stop";
      int speed           = rxDoc["speed"] | BASE_PWM;
      int turn_angle      = rxDoc["turn_angle"] | 90;
      int next_x          = rxDoc["next_x"] | posX;
      int next_y          = rxDoc["next_y"] | posY;

      if (strcmp(command, "move_forward") == 0) {
        Serial.printf("[SERVER] CMD: move_forward -> Target: (%d, %d)\n", next_x, next_y);
        moveOneCellForward();
        // Server coordinates are authoritative for route waypoints:
        posX = next_x;
        posY = next_y;
      }
      else if (strcmp(command, "turn_left") == 0) {
        Serial.printf("[SERVER] CMD: turn_left by %d deg\n", turn_angle);
        if (turn_angle == 180) {
          turn180();
        } else {
          turnLeft90();
        }
      }
      else if (strcmp(command, "turn_right") == 0) {
        Serial.printf("[SERVER] CMD: turn_right by %d deg\n", turn_angle);
        if (turn_angle == 180) {
          turn180();
        } else {
          turnRight90();
        }
      }
      else if (strcmp(command, "stop") == 0) {
        stopMotors();
      }
    }
  }

  http.end();
}

// =====================================================================================
// 13. COMMAND PARSER (SERIAL MONITOR)
// =====================================================================================

void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  char c = toupper(cmd.charAt(0));

  if (c == 'S') {
    stopMotors();
    is_running = false;
    Serial.println(F(">> MOTORS EMERGENCY STOP"));
  }
  else if (c == 'Z') {
    is_running = true;
    resetState();
  }
  else if (c == 'P') {
    printPosition();
  }
  else if (c == 'L') {
    turnLeft90();
  }
  else if (c == 'D') {
    turnRight90();
  }
  else if (c == 'U') {
    turn180();
  }
  else if (c == '+') {
    TURN_90_MS += 25;
    Serial.print(F(">> TURN_90_MS INCREASED TO: "));
    Serial.print(TURN_90_MS);
    Serial.println(F(" ms"));
  }
  else if (c == '-') {
    if (TURN_90_MS > 200) TURN_90_MS -= 25;
    Serial.print(F(">> TURN_90_MS DECREASED TO: "));
    Serial.print(TURN_90_MS);
    Serial.println(F(" ms"));
  }
  else if (c == 'T') {
    int spaceIdx = cmd.indexOf(' ');
    if (spaceIdx != -1) {
      unsigned long val = cmd.substring(spaceIdx + 1).toInt();
      if (val >= 200 && val <= 3000) {
        TURN_90_MS = val;
        Serial.print(F(">> TURN_90_MS SET TO: "));
        Serial.print(TURN_90_MS);
        Serial.println(F(" ms"));
      }
    } else {
      Serial.print(F("Current TURN_90_MS = "));
      Serial.print(TURN_90_MS);
      Serial.println(F(" ms. Use 'T <ms>' (e.g. T 1200) or '+' / '-'"));
    }
  }
  else if (c == 'G') {
    int firstSpace = cmd.indexOf(' ');
    if (firstSpace == -1) firstSpace = cmd.indexOf(',');
    
    if (firstSpace != -1) {
      String rest = cmd.substring(firstSpace + 1);
      rest.trim();
      int secondSpace = rest.indexOf(' ');
      if (secondSpace == -1) secondSpace = rest.indexOf(',');

      if (secondSpace != -1) {
        int targetX = rest.substring(0, secondSpace).toInt();
        int targetY = rest.substring(secondSpace + 1).toInt();
        navigateToGrid(targetX, targetY);
      } else {
        Serial.println(F("ERR: Invalid format. Use: G X Y (e.g., G 6 1)"));
      }
    } else {
      Serial.println(F("ERR: Invalid format. Use: G X Y (e.g., G 6 1)"));
    }
  }
  else {
    Serial.print(F("ERR: Unknown command '"));
    Serial.print(cmd);
    Serial.println(F("'. Available: S, Z, P, L, D, U, +, -, T <ms>, G X Y"));
  }
}

// =====================================================================================
// 14. SETUP & LOOP
// =====================================================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println(F("===================================================="));
  Serial.println(F("  Autonomous 4WD Skid-Steer UGV Controller (SIH)    "));
  Serial.println(F("  Baud Rate: 115200                                 "));
  Serial.println(F("===================================================="));

  // Configure Motor Driver Pins
  pinMode(ENA_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);

  stopMotors();

  // Initialize I2C and MPU6050
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // 100kHz standard robust I2C speed

  Serial.println(F("Initializing MPU6050..."));
  byte status = mpu.begin();
  if (status != 0) {
    Serial.println();
    Serial.println(F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
    Serial.print(F("ERROR: MPU6050 init failed with status: "));
    Serial.println(status);
    Serial.println(F("Check wiring: VCC->3.3V, GND->GND, SDA->D3(GPIO0), SCL->D4(GPIO2)"));
    Serial.println(F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
    
    // Non-freezing retry loop so serial output stays active
    while (status != 0) {
      Serial.println(F("Retrying MPU6050 connection in 2 seconds..."));
      delay(2000);
      status = mpu.begin();
      yield();
    }
  }

  Serial.println(F("MPU6050 connected successfully."));
  Serial.println(F("Calculating gyro offsets. DO NOT MOVE VEHICLE..."));
  delay(1000);
  mpu.calcOffsets();
  Serial.println(F("MPU6050 calibration complete."));

  resetState();

  // Connect to Wi-Fi if enabled
  if (ENABLE_WIFI) {
    Serial.print(F("[WIFI] Connecting to SSID: "));
    Serial.println(WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 15) {
      delay(500);
      Serial.print(F("."));
      retries++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      wifiConnected = true;
      Serial.println();
      Serial.print(F("[WIFI] Connected! IP: "));
      Serial.println(WiFi.localIP());
      Serial.print(F("[WIFI] Server Endpoint: "));
      Serial.println(SERVER_URL);
    } else {
      wifiConnected = false;
      Serial.println();
      Serial.println(F("[WIFI] Could not connect to Wi-Fi network."));
      Serial.println(F("[WIFI] Proceeding in STANDALONE SERIAL COMMAND MODE."));
    }
  }

  Serial.println();
  Serial.println(F("===================================================="));
  Serial.println(F("SYSTEM READY. Interactive Serial Commands:"));
  Serial.println(F("  S       : Emergency Stop"));
  Serial.println(F("  Z       : Reset State to (0,0), +X, Angle 0"));
  Serial.println(F("  P       : Print Position & Heading"));
  Serial.println(F("  L       : Turn Left 90 deg (Hybrid Corrected)"));
  Serial.println(F("  D       : Turn Right 90 deg (Hybrid Corrected)"));
  Serial.println(F("  U       : Turn 180 deg (Two 90s Hybrid Corrected)"));
  Serial.println(F("  G X Y   : Navigate to Target Cell (e.g. G 6 1)"));
  Serial.println(F("  + / -   : Fine tune TURN_90_MS by +/- 25ms"));
  Serial.println(F("  T <ms>  : Set TURN_90_MS directly"));
  Serial.println(F("===================================================="));
  Serial.print(F("UGV> "));
}

void loop() {
  // Keep MPU filter running continuously
  mpu.update();

  // Process incoming Serial commands
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();

    if (inputString.length() == 0 && (inChar == '\n' || inChar == '\r' || inChar == ' ')) {
      continue;
    }

    char upper = toupper(inChar);
    if (inputString.length() == 0 && (upper == 'S' || upper == 'Z' || upper == 'P' || 
                                      upper == 'L' || upper == 'D' || upper == 'U' ||
                                      upper == '+' || upper == '-')) {
      Serial.print(F(">> CMD RECEIVED: "));
      Serial.println(upper);
      processCommand(String(upper));
      inputString = "";
      Serial.print(F("\nUGV> "));
      continue;
    }

    if (inChar == '\n' || inChar == '\r') {
      if (inputString.length() > 0) {
        Serial.print(F(">> CMD RECEIVED: "));
        Serial.println(inputString);
        processCommand(inputString);
        inputString = "";
        Serial.print(F("\nUGV> "));
      }
    } else {
      inputString += inChar;
    }
  }

  // Automatic timeout dispatch for multi-character commands
  if (inputString.length() > 0 && Serial.available() == 0) {
    delay(30);
    if (Serial.available() == 0) {
      Serial.print(F(">> CMD RECEIVED: "));
      Serial.println(inputString);
      processCommand(inputString);
      inputString = "";
      Serial.print(F("\nUGV> "));
    }
  }

  // Periodic closed-loop telemetry poll to Python navigation server
  if (ENABLE_WIFI && wifiConnected && is_running) {
    if (millis() - lastServerPoll >= SERVER_POLL_INTERVAL_MS) {
      lastServerPoll = millis();
      pollServerAndExecute();
    }
  }

  yield();
}
