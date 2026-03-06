/**
 * ============================================================================
 * ROBOT MASTER CODE (ESP32-S3 N16R8)
 * ============================================================================
 * RESPONSIBILITIES:
 * 1. Read Sensors: 4x Drive Encoders, 2x Elevator Encoders, 4x HC-SR04, 1x MPU6050
 * 2. Localization: Kalman Filter (Fusion of Encoders + Gyro)
 * 3. Logic:
 * - 0-60s: Autonomous Path Following (PID)
 * - 60s+: Teleoperated via WiFi (UDP)
 * 4. Safety: Elevator Limits, Obstacle detection (optional structure included)
 * 5. Output: Sends kinematics (Wheel Speeds) to Slave via UART
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"
#include <BasicLinearAlgebra.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

using namespace BLA;

// ================= USER CONFIG =================
const char* ssid = "Fox-2";
const char* password = "Kyra2bin9";
const int localPort = 4210;

// Waypoints (X cm, Y cm) - Cleared as requested
struct Waypoint { float x; float y; };
Waypoint waypoints[] = {
  {0.0, 0.0},     // Start
  {100.0, 0.0},   // Example Point 1
  {100.0, 100.0}, // Example Point 2
  {0.0, 0.0}      // Return
};
int waypointCount = sizeof(waypoints) / sizeof(Waypoint);

// ================= PIN DEFINITIONS =================
// Drive Encoders
#define RL_A 1
#define RL_B 2
#define RR_A 4
#define RR_B 5
#define FL_A 6
#define FL_B 7
#define FR_A 8
#define FR_B 9

// Elevator Encoders
#define EL_A 10
#define EL_B 11
#define ER_A 12
#define ER_B 13

// Ultrasonics
const int TRIG_PIN = 14;
const int ECHO_FRONT = 15;
const int ECHO_RIGHT = 16;
const int ECHO_LEFT = 17;
const int ECHO_REAR = 18;

// I2C & UART
const int SDA_PIN = 41;
const int SCL_PIN = 42;
const int RX_PIN = 47; // To Slave TX (if needed)
const int TX_PIN = 20; // To Slave RX

// ================= VARIABLES & OBJECTS =================
MPU6050 mpu;
WiFiUDP udp;
char packetBuffer[255];

// Volatile Encoder Counts
volatile long cntRL = 0, cntRR = 0, cntFL = 0, cntFR = 0;
volatile long cntElevator = 0; // Combined average or single side tracking

// Robot Physical Constants
const float wheelCircumference = 31.4; // cm
const int ticksPerRev = 48; // Main Motor
const float tickToCm = wheelCircumference / ticksPerRev;
const float wheelbase = 31.75; // cm
const float gyroScale = 131.0;
const float gyroOffset = -135.0; // Needs calibration

// Elevator Limits
const int ELEVATOR_MAX_PULSES = 133; // ~2 seconds travel at 200rpm
const int ELEVATOR_MIN_PULSES = 0;

// State Variables
unsigned long startTime;
bool isAutonomous = true;
int currentWaypointIndex = 0;

// Teleop Data (Protected by Mutex ideally, but bool/atomic logic used here)
float axis0=0, axis2=0, axis4=0, axis5=0; // Slide, Turn, Fwd, Bwd
int btn6=0, btn12=0, btn13=0; // Servo, Down, Up

// Kalman State X = [x, y, theta, v, omega]
BLA::Matrix<5,1> X = {0,0,0,0,0};
BLA::Matrix<5,5> P = BLA::Identity<5,5>() * 0.1;
BLA::Matrix<5,5> Q = BLA::Identity<5,5>() * 0.01;

// ================= INTERRUPT SERVICE ROUTINES =================
void IRAM_ATTR isrRL() { if(digitalRead(RL_A)==digitalRead(RL_B)) cntRL++; else cntRL--; }
void IRAM_ATTR isrRR() { if(digitalRead(RR_A)==digitalRead(RR_B)) cntRR++; else cntRR--; }
void IRAM_ATTR isrFL() { if(digitalRead(FL_A)==digitalRead(FL_B)) cntFL++; else cntFL--; }
void IRAM_ATTR isrFR() { if(digitalRead(FR_A)==digitalRead(FR_B)) cntFR++; else cntFR--; }

// Elevator ISR (Tracking one side is usually enough if synced, or average both)
// Assuming EL is the master for height tracking
void IRAM_ATTR isrEL() { 
  // Determine direction based on phase. Assuming A leading B = UP
  if(digitalRead(EL_A)==digitalRead(EL_B)) cntElevator++; else cntElevator--; 
}

// ================= FUNCTIONS =================

// Send Command to Slave
// Format: <FL_PWM>,<FR_PWM>,<RL_PWM>,<RR_PWM>,<EL_CMD>,<SERVO_CMD>
// EL_CMD: 0=Stop, 1=Up, 2=Down
// SERVO_CMD: 0=Close, 1=Open
void sendToSlave(int fl, int fr, int rl, int rr, int el_cmd, int servo_cmd) {
  Serial2.printf("<%d,%d,%d,%d,%d,%d>\n", fl, fr, rl, rr, el_cmd, servo_cmd);
}

// Ultrasonic Read (Simple non-blocking approach suggested, but blocking used for simplicity here)
long readDistance(int echoPin) {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(echoPin, HIGH, 5000); // 5ms timeout (fast)
  if (duration == 0) return 999;
  return duration * 0.034 / 2;
}

// Kalman Prediction Step
void kalmanPredict(float dt, float gz) {
  // Simplified Odometry
  long dRL = cntRL; cntRL = 0; // Reset deltas
  long dRR = cntRR; cntRR = 0;
  long dFL = cntFL; cntFL = 0;
  long dFR = cntFR; cntFR = 0;

  float distL = ((dRL + dFL)/2.0) * tickToCm;
  float distR = ((dRR + dFR)/2.0) * tickToCm;
  float dist = (distL + distR) / 2.0;
  
  float gyroZ = (gz - gyroOffset) / gyroScale; 
  float gyroRad = gyroZ * (PI / 180.0);

  // Update State
  X(0) += dist * cos(X(2)); // x
  X(1) += dist * sin(X(2)); // y
  X(2) += gyroRad * dt;     // theta
  // Normalize theta
  while(X(2) > PI) X(2) -= 2*PI;
  while(X(2) < -PI) X(2) += 2*PI;
}

// Task: WiFi Communication (Core 0)
void TaskWiFi(void *pvParameters) {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); }
  udp.begin(localPort);

  while(true) {
    int packetSize = udp.parsePacket();
    if (packetSize) {
      int len = udp.read(packetBuffer, 255);
      if (len > 0) packetBuffer[len] = 0;
      String msg = String(packetBuffer);
      
      // Parse Logic (Simple string parsing)
      // Expected Format: "AXIS 0 1.0" or "BUTTON 12 1"
      // We update global variables here
      
      if(msg.startsWith("AXIS")) {
        int id = msg.substring(5, 6).toInt();
        float val = msg.substring(7).toFloat();
        if(id == 0) axis0 = val; // Strafe
        if(id == 2) axis2 = val; // Turn
        if(id == 4) axis4 = val; // Forward
        if(id == 5) axis5 = val; // Backward
      }
      if(msg.startsWith("BUTTON")) {
        int id = msg.substring(7, 9).toInt(); // Handles 2 digit ID
        int val = msg.substring(10).toInt();
        if(id == 6) btn6 = val;
        if(id == 12) btn12 = val;
        if(id == 13) btn13 = val;
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN); // UART to Slave

  // Encoder Init
  pinMode(RL_A, INPUT); pinMode(RL_B, INPUT);
  pinMode(RR_A, INPUT); pinMode(RR_B, INPUT);
  pinMode(FL_A, INPUT); pinMode(FL_B, INPUT);
  pinMode(FR_A, INPUT); pinMode(FR_B, INPUT);
  pinMode(EL_A, INPUT); pinMode(EL_B, INPUT); // Elevator
  
  attachInterrupt(RL_A, isrRL, RISING);
  attachInterrupt(RR_A, isrRR, RISING);
  attachInterrupt(FL_A, isrFL, RISING);
  attachInterrupt(FR_A, isrFR, RISING);
  attachInterrupt(EL_A, isrEL, RISING); // Elevator Tracking

  // MPU Init
  Wire.begin(SDA_PIN, SCL_PIN);
  mpu.initialize();

  // Ultrasonics
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_FRONT, INPUT); pinMode(ECHO_RIGHT, INPUT);
  pinMode(ECHO_LEFT, INPUT); pinMode(ECHO_REAR, INPUT);

  // WiFi Task on Core 0
  xTaskCreatePinnedToCore(TaskWiFi, "WiFiTask", 10000, NULL, 1, NULL, 0);

  Serial.println("MASTER N16R8 READY");
  startTime = millis();
}

// ================= LOOP (CORE 1) =================
void loop() {
  static unsigned long lastLoop = 0;
  unsigned long now = millis();
  float dt = (now - lastLoop) / 1000.0;
  lastLoop = now;

  // 1. Localization Update
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  kalmanPredict(dt, gz);

  // 2. Debug Output (Periodically)
  if (now % 500 == 0) {
    Serial.printf("POS X:%.1f Y:%.1f TH:%.2f | EL_CNT:%ld | SENS: F%ld R%ld\n", 
                  X(0), X(1), X(2), cntElevator, readDistance(ECHO_FRONT), readDistance(ECHO_RIGHT));
  }

  // 3. Control Logic
  int fl=0, fr=0, rl=0, rr=0;
  int el_cmd = 0; // 0:Stop, 1:Up, 2:Down
  int servo_cmd = (btn6 == 1) ? 1 : 0;

  // --- TIME CHECK ---
  if (now - startTime > 60000) isAutonomous = false;

  if (isAutonomous) {
    // --- AUTONOMOUS MODE (PID) ---
    // Simple logic: Drive to current waypoint
    if (currentWaypointIndex < waypointCount) {
      float dx = waypoints[currentWaypointIndex].x - X(0);
      float dy = waypoints[currentWaypointIndex].y - X(1);
      float distError = sqrt(dx*dx + dy*dy);
      float targetAngle = atan2(dy, dx);
      float angleError = targetAngle - X(2);

      // Simple P-Controller
      float v = 0.5 * distError; // Forward speed
      float w = 1.0 * angleError; // Turn speed
      
      // Mecanum Kinematics for V + W
      // (Simplified for Auto: No strafing, just turn & move)
      fl = v - w; fr = v + w;
      rl = v - w; rr = v + w;

      if(distError < 5.0) currentWaypointIndex++; // Reached
    } else {
      // Finished Waypoints
      fl=0; fr=0; rl=0; rr=0;
    }

  } else {
    // --- TELEOP MODE (WiFi) ---
    // Inputs: axis0(Slide), axis2(Turn), axis4(Fwd), axis5(Bwd)
    // NO MIXING requested: Priority Chain
    
    int spd = 200; // Base speed PWM

    if (axis4 > 0.1) { // FORWARD
      fl = spd; fr = spd; rl = spd; rr = spd;
    } 
    else if (axis5 > 0.1) { // BACKWARD
      fl = -spd; fr = -spd; rl = -spd; rr = -spd;
    }
    else if (abs(axis0) > 0.1) { // SLIDE
      if (axis0 > 0) { // Right
        fl = spd; fr = -spd; rl = -spd; rr = spd;
      } else { // Left
        fl = -spd; fr = spd; rl = spd; rr = -spd;
      }
    }
    else if (abs(axis2) > 0.1) { // TURN
      if (axis2 > 0) { // Right
        fl = spd; fr = -spd; rl = spd; rr = -spd;
      } else { // Left
        fl = -spd; fr = spd; rl = -spd; rr = spd;
      }
    } 
    else {
      // IMMEDIATE STOP
      fl=0; fr=0; rl=0; rr=0;
    }

    // --- ELEVATOR CONTROL LOGIC ---
    // Button 13: UP, Button 12: DOWN
    // Limit Logic with Encoder
    
    if (btn13 == 1) { // Request UP
      if (cntElevator < ELEVATOR_MAX_PULSES) {
        el_cmd = 1; // Allow UP
      } else {
        el_cmd = 0; // Stop, limit reached
      }
    } 
    else if (btn12 == 1) { // Request DOWN
      if (cntElevator > ELEVATOR_MIN_PULSES) {
        el_cmd = 2; // Allow DOWN
      } else {
        el_cmd = 0; // Stop, limit reached
      }
    }
  }

  // 4. Send to Slave
  sendToSlave(fl, fr, rl, rr, el_cmd, servo_cmd);

  delay(20); // Loop rate ~50Hz
}