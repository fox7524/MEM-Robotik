#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"
#include <BasicLinearAlgebra.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Core 0: WiFi Command Processing & UART Communication
// Core 1: Autonomous PID Navigation & Sensor Processing

using namespace BLA;

// === STATE ESTIMATION ===
BLA::Matrix<5,1> X = {0,0,0,0,0};  // [x, y, theta, v, omega]
BLA::Matrix<5,5> P = BLA::Identity<5,5>() * 0.1;
BLA::Matrix<5,5> Q = BLA::Identity<5,5>() * 0.01;
BLA::Matrix<5,5> R = BLA::Identity<5,5>() * 0.5;

// === PIN DEFINITIONS ===
#define RL_A 1  #define RL_B 2
#define RR_A 4  #define RR_B 5
#define FL_A 6  #define FL_B 7
#define FR_A 8  #define FR_B 9
#define EL_A 10 #define EL_B 11
#define ER_A 12 #define ER_B 13

const int TRIG_PIN = 14;
const int ECHO_FRONT = 15, ECHO_RIGHT = 16, ECHO_LEFT = 17, ECHO_REAR = 18;
const int SDA_PIN = 41, SCL_PIN = 42;
const int RX_PIN = 47, TX_PIN = 20;  // UART to N8R2

// === CALIBRATION ===
const float wheelCircumference = 31.4;  // cm
const int mainTicksPerRev = 48;
const int elevTicksPerRev = 20;         // Elevator: 20 pulses/rotation
const float tickToCm = wheelCircumference / mainTicksPerRev;
const float wheelbase = 31.75;          // cm
const float gyroScale = 131.0;
const float gyroOffset = -135.0;

// Elevator limits: 200RPM * 2sec = 6.67 rotations * 20 pulses = 133 pulses total
const long ELEVATOR_MAX_PULSES = 133;
const long ELEVATOR_MIN_PULSES = 0;

// === ENCODER COUNTERS ===
volatile long RL_count = 0, RR_count = 0, FL_count = 0, FR_count = 0;
volatile long EL_count = 0, ER_count = 0;
volatile long prev_EL_count = 0, prev_ER_count = 0;

// === AUTONOMOUS MODE ===
bool autonomousMode = true;
unsigned long autonomousStartTime = 0;
const unsigned long AUTONOMOUS_DURATION = 60000;  // 60 seconds

struct Waypoint {
  float x, y;
};
Waypoint waypoints[] = {{0,0}};  // Clear - add your waypoints later
int currentWaypoint = 0;
int numWaypoints = 1;

// WiFi & UDP
WiFiUDP udp;
unsigned int localPort = 4210;
char packetBuffer[255];

// MPU6050
MPU6050 mpu;

// === ISR Functions ===
void IRAM_ATTR isrRL() { if (digitalRead(RL_A) == digitalRead(RL_B)) RL_count++; else RL_count--; }
void IRAM_ATTR isrRR() { if (digitalRead(RR_A) == digitalRead(RR_B)) RR_count++; else RR_count--; }
void IRAM_ATTR isrFL() { if (digitalRead(FL_A) == digitalRead(FL_B)) FL_count++; else FL_count--; }
void IRAM_ATTR isrFR() { if (digitalRead(FR_A) == digitalRead(FR_B)) FR_count++; else FR_count--; }
void IRAM_ATTR isrEL() { if (digitalRead(EL_A) == digitalRead(EL_B)) EL_count++; else EL_count--; }
void IRAM_ATTR isrER() { if (digitalRead(ER_A) == digitalRead(ER_B)) ER_count++; else ER_count--; }

// === Core 0: WiFi & Manual Control Task ===
void wifiControlTask(void *parameter) {
  while (1) {
    if (!autonomousMode) {
      // Read WiFi UDP commands and send to N8R2 via UART
      int packetSize = udp.parsePacket();
      if (packetSize) {
        int len = udp.read(packetBuffer, 255);
        if (len > 0) packetBuffer[len] = 0;
        
        // Forward raw commands to N8R2
        Serial2.print(packetBuffer);
        Serial2.print("\n");
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// === Core 1: Autonomous PID & Sensors Task ===
void pidSensorTask(void *parameter) {
  unsigned long lastTime = 0;
  int16_t gx, gy, gz;
  
  while (1) {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;
    if (dt < 0.01) { vTaskDelay(1 / portTICK_PERIOD_MS); continue; }
    
    // Read sensors
    mpu.getRotation(&gx, &gy, &gz);
    
    // Kalman Filter Update
    kalmanPredict(dt, gz, RL_count, RR_count, FL_count, FR_count);
    
    // Autonomous Navigation
    if (autonomousMode && autonomousStartTime > 0) {
      if (millis() - autonomousStartTime > AUTONOMOUS_DURATION) {
        autonomousMode = false;  // Switch to manual
        Serial.println("Autonomous complete - Manual mode active");
      } else {
        navigateToWaypoint();
      }
    }
    
    // Send sensor data & position via Serial for debug
    Serial.printf("POS:%.1f,%.1f,%.1f | RPM:RL%.1f,RR%.1f,FL%.1f,FR%.1f | ELEV:EL%d,ER%d | US:F%d,R%d,L%d,B%d\n",
      X(0), X(1), X(2)*180/PI,
      calcRPM(RL_count), calcRPM(RR_count), calcRPM(FL_count), calcRPM(FR_count),
      EL_count, ER_count,
      medianDistance(ECHO_FRONT), medianDistance(ECHO_RIGHT),
      medianDistance(ECHO_LEFT), medianDistance(ECHO_REAR));
    
    lastTime = currentTime;
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

// === Navigation Functions ===
float calcRPM(long pulses) {
  return (pulses / (float)mainTicksPerRev) * 60.0;
}

float medianDistance(int echoPin) {
  float vals[3];
  for (int i = 0; i < 3; i++) {
    vals[i] = readUltrasonic(echoPin);
    delay(2);
  }
  std::sort(vals, vals+3);
  return vals[1];
}

float readUltrasonic(int echoPin) {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  return pulseIn(echoPin, HIGH, 30000) * 0.034 / 2;
}

void kalmanPredict(float dt, int16_t gz, long RL, long RR, long FL, long FR) {
  float dL = ((RL + FL) / 2.0) * tickToCm;
  float dR = ((RR + FR) / 2.0) * tickToCm;
  float d = (dL + dR) / 2.0;
  float dTheta = (dR - dL) / wheelbase;
  
  float gyroZrad = ((gz - gyroOffset) / gyroScale) * (PI/180.0) * dt;
  X(0) += d * cos(X(2)); X(1) += d * sin(X(2)); X(2) += dTheta + gyroZrad;
  X(3) = d / dt; X(4) = gyroZrad;
  
  // Covariance prediction (simplified)
  P = P + Q;
}

void navigateToWaypoint() {
  float targetX = waypoints[currentWaypoint].x;
  float targetY = waypoints[currentWaypoint].y;
  float dx = targetX - X(0), dy = targetY - X(1);
  float distance = sqrt(dx*dx + dy*dy);
  float angleError = atan2(dy, dx) - X(2);
  
  if (distance < 5.0) {  // Reached waypoint
    currentWaypoint = (currentWaypoint + 1) % numWaypoints;
  }
  
  // Simple PID output (send to N8R2 later)
  float turnCmd = constrain(angleError * 2.0, -1.0, 1.0);
  float fwdCmd = constrain(distance * 0.1, 0.0, 1.0);
  
  Serial2.printf("AUTONOMOUS: TURN%.2f FWD%.2f\n", turnCmd, fwdCmd);
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  
  // Encoder pins
  pinMode(RL_A, INPUT); pinMode(RL_B, INPUT);
  pinMode(RR_A, INPUT); pinMode(RR_B, INPUT);
  pinMode(FL_A, INPUT); pinMode(FL_B, INPUT);
  pinMode(FR_A, INPUT); pinMode(FR_B, INPUT);
  pinMode(EL_A, INPUT); pinMode(EL_B, INPUT);
  pinMode(ER_A, INPUT); pinMode(ER_B, INPUT);
  
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(RL_A), isrRL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RR_A), isrRR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FL_A), isrFL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FR_A), isrFR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EL_A), isrEL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ER_A), isrER, CHANGE);
  
  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  for (int pin : {ECHO_FRONT, ECHO_RIGHT, ECHO_LEFT, ECHO_REAR}) pinMode(pin, INPUT);
  
  // MPU6050
  Wire.begin(SDA_PIN, SCL_PIN);
  mpu.initialize();
  
  // WiFi
  WiFi.begin("Fox-2", "Kyra2bin9");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  udp.begin(localPort);
  
  Serial.println("N16R8 Master Started - Autonomous Mode");
  
  // Start tasks
  xTaskCreatePinnedToCore(wifiControlTask, "WiFiTask", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(pidSensorTask, "PIDTask", 8192, NULL, 2, NULL, 1);
  
  autonomousStartTime = millis();
}

void loop() {
  // Main loop empty - all work in FreeRTOS tasks
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
