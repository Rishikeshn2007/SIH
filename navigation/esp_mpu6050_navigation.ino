/**
 * esp_mpu6050_navigation.ino
 * ===========================
 * Closed-Loop UGV Navigation Firmware with Integrated MPU6050 IMU
 * Compatible with BOTH ESP32 and ESP8266 (NodeMCU / Wemos D1).
 * 
 * Hardware Connections:
 * -------------------------------------------------------------
 * Device              ESP Pin Label   ESP GPIO    Function
 * -------------------------------------------------------------
 * MPU6050 SDA         D3              GPIO0       I2C Data
 * MPU6050 SCL         D4              GPIO2       I2C Clock
 * Left Motor ENA      D5              GPIO14      PWM Speed (Left)
 * Left Motor IN1      D1              GPIO5       Direction Fwd (Left)
 * Left Motor IN2      D2              GPIO4       Direction Bwd (Left)
 * Right Motor ENB     D8              GPIO15      PWM Speed (Right)
 * Right Motor IN3     D6              GPIO12      Direction Fwd (Right)
 * Right Motor IN4     D7              GPIO13      Direction Bwd (Right)
 * MPU6050 VCC         3.3V            3.3V        Power
 * MPU6050 GND         GND             GND         Common Ground
 * -------------------------------------------------------------
 * 
 * Dependencies:
 *   - ArduinoJson (v6 or v7)
 *   - Wire (Built-in)
 *   - WiFi & HTTPClient (Built-in to ESP32 / ESP8266 board packages)
 */

#if defined(ESP8266)
  #include <ESP8266WiFi.h>
  #include <ESP8266HTTPClient.h>
  #include <WiFiClient.h>
#elif defined(ESP32)
  #include <WiFi.h>
  #include <HTTPClient.h>
#endif

#include <Wire.h>
#include <ArduinoJson.h>

// ============================================================================
// 1. PIN DEFINITIONS (Exact match to requested connection table)
// ============================================================================
#define PIN_MPU_SDA   0   // D3 (GPIO0)
#define PIN_MPU_SCL   2   // D4 (GPIO2)

#define PIN_LEFT_ENA  14  // D5 (GPIO14) - PWM Left
#define PIN_LEFT_IN1  5   // D1 (GPIO5)  - Left Forward
#define PIN_LEFT_IN2  4   // D2 (GPIO4)  - Left Backward

#define PIN_RIGHT_ENB 15  // D8 (GPIO15) - PWM Right
#define PIN_RIGHT_IN3 12  // D6 (GPIO12) - Right Forward
#define PIN_RIGHT_IN4 13  // D7 (GPIO13) - Right Backward

// ESP32 PWM Channels (ignored on ESP8266)
#define PWM_CH_LEFT   0
#define PWM_CH_RIGHT  1
#define PWM_FREQ      1000
#define PWM_RES_BITS  8

// ============================================================================
// 2. NETWORK & SERVER CONFIGURATION
// ============================================================================
const char* WIFI_SSID     = "GMU_Staff";     // <-- Set your Wi-Fi SSID
const char* WIFI_PASSWORD = "GMU@2025@"; // <-- Set your Wi-Fi Password

// URL to your computer running server.py
const char* SERVER_URL    = "http://http://172.21.3.69:5000/car/state";

// Step traversal timing for 30x30 cm grid cell
const unsigned long CELL_TRAVERSE_MS = 1400; // Time to travel 30 cm at cruise speed

// ============================================================================
// 3. MPU6050 CONSTANTS & VARIABLES
// ============================================================================
#define MPU_ADDR 0x68
float gyro_z_offset = 0.0;
float current_yaw = 0.0;
unsigned long last_imu_time = 0;

// ============================================================================
// 4. VEHICLE NAVIGATION STATE
// ============================================================================
int current_x = 0;          // Column (0 to 4 in 20x5 grid)
int current_y = 0;          // Row (0 to 19 in 20x5 grid)
int current_heading = 0;    // Cardinal heading: 0=North, 90=East, 180=South, 270=West
bool is_running = true;

// ============================================================================
// 5. MOTOR CONTROLLER (PWM & DIRECTION)
// ============================================================================
void initMotors() {
  pinMode(PIN_LEFT_IN1, OUTPUT);
  pinMode(PIN_LEFT_IN2, OUTPUT);
  pinMode(PIN_RIGHT_IN3, OUTPUT);
  pinMode(PIN_RIGHT_IN4, OUTPUT);

#if defined(ESP32)
  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcAttach(PIN_LEFT_ENA, PWM_FREQ, PWM_RES_BITS);
    ledcAttach(PIN_RIGHT_ENB, PWM_FREQ, PWM_RES_BITS);
  #else
    ledcSetup(PWM_CH_LEFT, PWM_FREQ, PWM_RES_BITS);
    ledcAttachPin(PIN_LEFT_ENA, PWM_CH_LEFT);
    ledcSetup(PWM_CH_RIGHT, PWM_FREQ, PWM_RES_BITS);
    ledcAttachPin(PIN_RIGHT_ENB, PWM_CH_RIGHT);
  #endif
#else
  pinMode(PIN_LEFT_ENA, OUTPUT);
  pinMode(PIN_RIGHT_ENB, OUTPUT);
#endif

  stopMotors();
}

void setMotorPwm(int speed_0_to_200) {
  int speed = constrain(speed_0_to_200, 0, 200);

#if defined(ESP32)
  // Scale 0-200 -> 0-255 (8-bit)
  int duty = map(speed, 0, 200, 0, 255);
  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcWrite(PIN_LEFT_ENA, duty);
    ledcWrite(PIN_RIGHT_ENB, duty);
  #else
    ledcWrite(PWM_CH_LEFT, duty);
    ledcWrite(PWM_CH_RIGHT, duty);
  #endif
#else
  // Scale 0-200 -> 0-1023 (10-bit ESP8266)
  int duty = map(speed, 0, 200, 0, 1023);
  analogWrite(PIN_LEFT_ENA, duty);
  analogWrite(PIN_RIGHT_ENB, duty);
#endif
}

void stopMotors() {
  digitalWrite(PIN_LEFT_IN1, LOW);
  digitalWrite(PIN_LEFT_IN2, LOW);
  digitalWrite(PIN_RIGHT_IN3, LOW);
  digitalWrite(PIN_RIGHT_IN4, LOW);
  setMotorPwm(0);
}

// ============================================================================
// 6. MPU6050 DRIVER & GYROSCOPE INTEGRATION
// ============================================================================
void initMPU6050() {
  // Wire.begin(SDA, SCL)
  Wire.begin(PIN_MPU_SDA, PIN_MPU_SCL);
  Wire.setClock(100000); // 100kHz I2C

  // Wake up MPU6050 (PWR_MGMT_1 register = 0)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  byte error = Wire.endTransmission();

  if (error != 0) {
    Serial.println("[MPU6050] Error connecting! Check wiring and 3.3V/GND.");
    return;
  }
  Serial.println("[MPU6050] Initialized successfully. Calibrating gyro Z...");

  // Calibrate Gyro Z offset (Keep vehicle completely still)
  float sum = 0;
  const int SAMPLES = 200;
  for (int i = 0; i < SAMPLES; i++) {
    sum += readRawGyroZ();
    delay(5);
  }
  gyro_z_offset = sum / SAMPLES;
  last_imu_time = millis();
  Serial.printf("[MPU6050] Calibration complete. Gyro Z offset: %.3f deg/s\n", gyro_z_offset);
}

float readRawGyroZ() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47); // GYRO_ZOUT_H register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);

  if (Wire.available() < 2) return 0.0;
  int16_t raw = (Wire.read() << 8) | Wire.read();
  // Standard ±250 deg/s range has sensitivity of 131 LSB/(deg/s)
  return ((float)raw) / 131.0;
}

void updateIMU() {
  unsigned long now = millis();
  float dt = (now - last_imu_time) / 1000.0;
  last_imu_time = now;

  float gz = readRawGyroZ() - gyro_z_offset;
  // Threshold small noise jitter (< 0.2 deg/s)
  if (abs(gz) > 0.2) {
    current_yaw += gz * dt;
  }
}

// ============================================================================
// 7. CLOSED-LOOP MOTION CONTROLS (USING GYROSCOPE)
// ============================================================================
void driveForward(int speed) {
  Serial.printf("[MOVE] Driving forward 1 cell (30cm) at speed %d...\n", speed);
  setMotorPwm(speed);
  digitalWrite(PIN_LEFT_IN1, HIGH);
  digitalWrite(PIN_LEFT_IN2, LOW);
  digitalWrite(PIN_RIGHT_IN3, HIGH);
  digitalWrite(PIN_RIGHT_IN4, LOW);

  unsigned long start = millis();
  while (millis() - start < CELL_TRAVERSE_MS) {
    updateIMU();
    delay(10);
  }
  stopMotors();
  delay(150);
}

// Precise Gyro-Controlled Turn
void executeGyroTurn(int speed, int targetAngle, bool turnRight) {
  Serial.printf("[TURN] Gyro-monitored turn %s by %d°...\n",
                turnRight ? "RIGHT" : "LEFT", targetAngle);

  setMotorPwm(speed);
  if (turnRight) {
    // Pivot Right: Left forward, Right backward
    digitalWrite(PIN_LEFT_IN1, HIGH);
    digitalWrite(PIN_LEFT_IN2, LOW);
    digitalWrite(PIN_RIGHT_IN3, LOW);
    digitalWrite(PIN_RIGHT_IN4, HIGH);
  } else {
    // Pivot Left: Left backward, Right forward
    digitalWrite(PIN_LEFT_IN1, LOW);
    digitalWrite(PIN_LEFT_IN2, HIGH);
    digitalWrite(PIN_RIGHT_IN3, HIGH);
    digitalWrite(PIN_RIGHT_IN4, LOW);
  }

  float startYaw = current_yaw;
  float rotated = 0.0;
  unsigned long turnTimeout = millis();

  // Rotate until gyroscope confirms requested angle (with 2.5s safety timeout)
  while (abs(rotated) < (targetAngle - 3.0) && (millis() - turnTimeout < 2500)) {
    updateIMU();
    rotated = current_yaw - startYaw;
    delay(5);
  }

  stopMotors();
  delay(150);
  Serial.printf("[TURN] Turn finished. Measured rotation: %.1f°\n", abs(rotated));
}

// ============================================================================
// 8. TELEMETRY & COMMAND PROTOCOL
// ============================================================================
void sendTelemetryAndExecute() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.reconnect();
    return;
  }

#if defined(ESP8266)
  WiFiClient client;
  HTTPClient http;
  if (!http.begin(client, SERVER_URL)) return;
#elif defined(ESP32)
  HTTPClient http;
  if (!http.begin(SERVER_URL)) return;
#endif

  http.addHeader("Content-Type", "application/json");

  // Step 1: Send telemetry packet
  StaticJsonDocument<128> txDoc;
  txDoc["x"] = current_x;
  txDoc["y"] = current_y;
  txDoc["heading"] = current_heading;
  txDoc["running"] = is_running;

  String txBuffer;
  serializeJson(txDoc, txBuffer);

  Serial.println("\n------------------------------------------------");
  Serial.printf("[TX -> Server] Pos: (X=%d, Y=%d) | Heading: %d° | Running: %s\n",
                current_x, current_y, current_heading, is_running ? "true" : "false");

  int httpCode = http.POST(txBuffer);

  // Step 2: Parse server command
  if (httpCode == 200) {
    String payload = http.getString();
    Serial.printf("[RX <- Server] %s\n", payload.c_str());

    StaticJsonDocument<256> rxDoc;
    deserializeJson(rxDoc, payload);

    const char* command = rxDoc["command"] | "stop";
    int speed = rxDoc["speed"] | 0;
    int turn_angle = rxDoc["turn_angle"] | 90;
    int next_x = rxDoc["next_x"] | current_x;
    int next_y = rxDoc["next_y"] | current_y;

    // Step 3: Execute movement and update state only upon completion
    if (strcmp(command, "move_forward") == 0) {
      driveForward(speed);
      current_x = next_x;
      current_y = next_y;
      Serial.printf("[POSE] Advanced to: (X=%d, Y=%d)\n", current_x, current_y);

    } else if (strcmp(command, "turn_left") == 0) {
      executeGyroTurn(speed, turn_angle, false);
      current_heading = (current_heading - turn_angle + 360) % 360;
      Serial.printf("[POSE] New Heading: %d°\n", current_heading);

    } else if (strcmp(command, "turn_right") == 0) {
      executeGyroTurn(speed, turn_angle, true);
      current_heading = (current_heading + turn_angle) % 360;
      Serial.printf("[POSE] New Heading: %d°\n", current_heading);

    } else if (strcmp(command, "stop") == 0) {
      stopMotors();
      Serial.println("[HALT] Vehicle stopped.");
    }
  } else {
    Serial.printf("[HTTP] Communication error: %d\n", httpCode);
    stopMotors();
  }

  http.end();
}

// ============================================================================
// 9. SETUP & MAIN LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== UGV Autonomous Navigation (ESP32 / ESP8266 + MPU6050) ===");

  initMotors();
  initMPU6050();

  // Connect WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("[WIFI] Connecting to %s", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.println("\n[WIFI] Connected! IP: " + WiFi.localIP().toString());
}

void loop() {
  updateIMU();
  sendTelemetryAndExecute();
  delay(300); // 300 ms closed-loop polling rate
}
