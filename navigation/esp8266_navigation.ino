/**
 * esp8266_navigation.ino
 * ======================
 * Closed-Loop UGV Navigation Firmware for ESP8266 (NodeMCU / Wemos D1 Mini)
 * 
 * Communicates with Flask Navigation Server (server.py) via HTTP POST /car/state.
 * Controls standard dual-motor H-bridge driver (L298N, L293D, TB6612FNG, etc.).
 *
 * Dependencies (install via Arduino Library Manager):
 *   - ESP8266WiFi (built into ESP8266 core)
 *   - ESP8266HTTPClient (built into ESP8266 core)
 *   - ArduinoJson (v6 or v7 by Benoit Blanchon)
 */

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>

// ============================================================================
// 1. PIN CONFIGURATION (ESP8266 NodeMCU / D1 Mini)
// Adjust these pins to match your physical motor driver wiring:
// ============================================================================
// Left Motor Pins
#define ENA_PIN   D1   // GPIO5  - PWM Speed Control Left Motor
#define IN1_PIN   D2   // GPIO4  - Left Motor Direction Forward
#define IN2_PIN   D3   // GPIO0  - Left Motor Direction Backward

// Right Motor Pins
#define IN3_PIN   D5   // GPIO14 - Right Motor Direction Forward
#define IN4_PIN   D6   // GPIO12 - Right Motor Direction Backward
#define ENB_PIN   D7   // GPIO13 - PWM Speed Control Right Motor

// Optional Status LED
#define STATUS_LED D4  // Built-in LED on NodeMCU (active LOW)

// ============================================================================
// 2. NETWORK & SERVER CONFIGURATION
// ============================================================================
const char* WIFI_SSID     = "YOUR_WIFI_SSID";       // <-- Change to your Wi-Fi SSID
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";   // <-- Change to your Wi-Fi Password

// Replace with your PC's local IP running server.py (find via 'ipconfig' on Windows)
const char* SERVER_URL    = "http://192.168.1.100:5000/car/state";

// Movement calibration: duration (ms) required to move 1 grid cell or turn 90°
// Adjust these based on your vehicle's physical speed, wheel diameter, and arena size
const unsigned long CELL_STEP_DURATION_MS = 1200; // Time to traverse 1 cell (~10 cm)
const unsigned long TURN_90_DURATION_MS   = 650;  // Time to execute a 90° pivot turn
const unsigned long TURN_180_DURATION_MS  = 1300; // Time to execute a 180° pivot turn

// ============================================================================
// 3. VEHICLE STATE (Single Source of Truth on Microcontroller)
// ============================================================================
int current_x = 0;          // Column index (0-based)
int current_y = 0;          // Row index (0-based)
int current_heading = 0;    // 0=North (up), 90=East (right), 180=South (down), 270=West (left)
bool is_running = true;     // Vehicle operational flag

// ============================================================================
// 4. MOTOR CONTROL FUNCTIONS
// ============================================================================
void initMotors() {
  pinMode(ENA_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);

  // ESP8266 PWM range is 0-1023 by default. Server sends speed 0-200.
  // We will map 0-200 -> 0-1023 in setMotorPwm().
  stopMotors();
}

void setMotorPwm(int speed_0_to_200) {
  // Map server speed (0-200) to ESP8266 analogWrite range (0-1023)
  int pwm = map(constrain(speed_0_to_200, 0, 200), 0, 200, 0, 1023);
  analogWrite(ENA_PIN, pwm);
  analogWrite(ENB_PIN, pwm);
}

void stopMotors() {
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  analogWrite(ENA_PIN, 0);
  analogWrite(ENB_PIN, 0);
  digitalWrite(STATUS_LED, HIGH); // LED off
}

void driveForward(int speed) {
  Serial.printf("[MOVE] Driving forward at speed %d...\n", speed);
  setMotorPwm(speed);
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
  digitalWrite(STATUS_LED, LOW); // LED on

  delay(CELL_STEP_DURATION_MS);
  stopMotors();
  delay(150); // Small settling delay
}

void turnLeft(int speed, int angle) {
  Serial.printf("[TURN] Turning LEFT by %d deg at speed %d...\n", angle, speed);
  setMotorPwm(speed);
  // Left wheel backward, Right wheel forward (pivot turn)
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
  digitalWrite(STATUS_LED, LOW);

  unsigned long duration = (angle == 180) ? TURN_180_DURATION_MS : TURN_90_DURATION_MS;
  delay(duration);
  stopMotors();
  delay(150);
}

void turnRight(int speed, int angle) {
  Serial.printf("[TURN] Turning RIGHT by %d deg at speed %d...\n", angle, speed);
  setMotorPwm(speed);
  // Left wheel forward, Right wheel backward (pivot turn)
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);
  digitalWrite(STATUS_LED, LOW);

  unsigned long duration = (angle == 180) ? TURN_180_DURATION_MS : TURN_90_DURATION_MS;
  delay(duration);
  stopMotors();
  delay(150);
}

// ============================================================================
// 5. SERVER COMMUNICATION (CLOSED-LOOP CYCLE)
// ============================================================================
void sendTelemetryAndExecute() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WIFI] Disconnected. Attempting reconnect...");
    WiFi.reconnect();
    return;
  }

  WiFiClient client;
  HTTPClient http;

  if (!http.begin(client, SERVER_URL)) {
    Serial.println("[HTTP] Unable to connect to server URL");
    return;
  }

  http.addHeader("Content-Type", "application/json");

  // ── Step 1: Serialize Telemetry JSON (Vehicle -> Server) ─────────────────
  // { "x": 0, "y": 0, "heading": 0, "running": true }
  StaticJsonDocument<128> txDoc;
  txDoc["x"] = current_x;
  txDoc["y"] = current_y;
  txDoc["heading"] = current_heading;
  txDoc["running"] = is_running;

  String txBuffer;
  serializeJson(txDoc, txBuffer);

  Serial.println("\n-------------------------------------------");
  Serial.printf("[TX -> Server] Pos: (%d, %d), Hdg: %d°, Running: %s\n",
                current_x, current_y, current_heading, is_running ? "true" : "false");

  // ── Step 2: Send HTTP POST Heartbeat ─────────────────────────────────────
  int httpCode = http.POST(txBuffer);

  // ── Step 3: Handle Response (Server -> Vehicle) ──────────────────────────
  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    Serial.printf("[RX <- Server] %s\n", payload.c_str());

    StaticJsonDocument<256> rxDoc;
    DeserializationError error = deserializeJson(rxDoc, payload);

    if (error) {
      Serial.printf("[JSON] Deserialization failed: %s\n", error.c_str());
      http.end();
      return;
    }

    const char* command = rxDoc["command"] | "stop";
    int speed = rxDoc["speed"] | 0;
    int turn_angle = rxDoc["turn_angle"] | 90;
    int next_x = rxDoc["next_x"] | current_x;
    int next_y = rxDoc["next_y"] | current_y;

    // ── Step 4: Execute Command & Update Closed-Loop Pose ───────────────────
    if (strcmp(command, "move_forward") == 0) {
      driveForward(speed);

      // IMPORTANT: Only update coordinates AFTER physical movement completes
      current_x = next_x;
      current_y = next_y;
      Serial.printf("[CLOSED-LOOP] Position updated to: (%d, %d)\n", current_x, current_y);

    } else if (strcmp(command, "turn_left") == 0) {
      turnLeft(speed, turn_angle);

      // Update heading: counter-clockwise by turn_angle
      current_heading = (current_heading - turn_angle + 360) % 360;
      Serial.printf("[CLOSED-LOOP] Heading updated to: %d°\n", current_heading);

    } else if (strcmp(command, "turn_right") == 0) {
      turnRight(speed, turn_angle);

      // Update heading: clockwise by turn_angle
      current_heading = (current_heading + turn_angle) % 360;
      Serial.printf("[CLOSED-LOOP] Heading updated to: %d°\n", current_heading);

    } else if (strcmp(command, "stop") == 0) {
      stopMotors();
      Serial.println("[HALT] Vehicle stopped (Arrived or Navigation Idle/Stopped).");
    }

  } else {
    Serial.printf("[HTTP] Error on POST, code: %d (%s)\n", httpCode, http.errorToString(httpCode).c_str());
    stopMotors(); // Failsafe: stop vehicle if server drops
  }

  http.end();
}

// ============================================================================
// 6. SETUP & LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== ESP8266 UGV Autonomous Navigation Client ===");

  initMotors();

  // Connect to Wi-Fi
  Serial.printf("[WIFI] Connecting to %s", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 30) {
    delay(500);
    Serial.print(".");
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WIFI] Connected successfully!");
    Serial.printf("[WIFI] IP Address: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n[WIFI] Connection failed! Check credentials & router.");
  }
}

void loop() {
  // Main closed-loop cycle
  sendTelemetryAndExecute();

  // Polling rate / heartbeat period (300 ms between actions)
  delay(300);
}
