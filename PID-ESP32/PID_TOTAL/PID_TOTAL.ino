#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"
#include <BasicLinearAlgebra.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
using namespace BLA;

// --- State vector X = [x, y, theta, v, omega] ---
BLA::Matrix<5,1> X = {0,0,0,0,0};
BLA::Matrix<5,5> P = BLA::Identity<5,5>() * 0.1;

// Noise matrices
BLA::Matrix<5,5> Q = BLA::Identity<5,5>() * 0.01;
BLA::Matrix<5,5> Rultra= BLA::Identity<5,5>() * 0.5;

// --- Pinouts ---
// Encoders
#define RL_A 1
#define RL_B 2
#define RR_A 4
#define RR_B 5
#define FL_A 6
#define FL_B 7
#define FR_A 8
#define FR_B 9
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

// MPU6050 I2C
const int SDA_PIN = 41;
const int SCL_PIN = 42;

//RX-TX to N8R2 esp32
const int RX_PIN = 47;
const int TX_PIN = 20;

// --- Calibration constants ---
const float wheelCircumference = 31.4; // cm
const int ticksPerRev = 48;
const float tickToCm = wheelCircumference / ticksPerRev;
const float wheelbase = 31.75; // cm
const float gyroScale = 131.0;   // LSB/°/s for ±250 range
const float gyroOffset = -135.0; // measured offset

// --- Hall counters ---
volatile long RL_count = 0, RR_count = 0, FL_count = 0, FR_count = 0;

// === Timing ===
unsigned long lastTime = 0;

// === Flags for Continuous Output ===
bool showFRRPM = false;
bool showFLRPM = false;
bool showRRRPM = false;
bool showRLRPM = false;

bool showFRHCSR = false;
bool showRTHCSR = false;
bool showLTHCSR = false;
bool showRRHCSR = false;

// === Encoder Counts ===
volatile long countRL = 0;
volatile long countRR = 0;
volatile long countFL = 0;
volatile long countFR = 0;

// === Timing ===
unsigned long lastTime = 0;

// === ISRs ===
void IRAM_ATTR isrRL() { if (digitalRead(RL_A) == digitalRead(RL_B)) countRL++; else countRL--; }
void IRAM_ATTR isrRR() { if (digitalRead(RR_A) == digitalRead(RR_B)) countRR++; else countRR--; }
void IRAM_ATTR isrFL() { if (digitalRead(FL_A) == digitalRead(FL_B)) countFL++; else countFL--; }
void IRAM_ATTR isrFR() { if (digitalRead(FR_A) == digitalRead(FR_B)) countFR++; else countFR--; }

// === RPM Calculation ===
float calcRPM(long pulses) {
  const int PPR = 100; // replace with your encoder’s pulses per revolution
  return (pulses / (float)PPR) * 60.0;
}


// --- NETWORKING ---
WiFiUDP udp;
unsigned int localPort = 4210;  // UDP port for receiving commands
char packetBuffer[255];         // Buffer for incoming UDP packets
 

// --- PID ---
struct Waypoint {
  float x, y;
};

// Add your waypoints here (x, y in cm)
Waypoint waypoints[] = {
  {50, 80},  // Example waypoint 1
  // {x2, y2},  // Add more waypoints as needed
  // {x3, y3},
};

int currentWaypoint = 0;
int numWaypoints = sizeof(waypoints) / sizeof(Waypoint);
float Kp_turn = 0.5;
float Kp_forward = 0.02;

// --- ISRs ---
void IRAM_ATTR isrRL() { if (digitalRead(RL_A) == digitalRead(RL_B)) RL_count++; else RL_count--; }
void IRAM_ATTR isrRR() { if (digitalRead(RR_A) == digitalRead(RR_B)) RR_count++; else RR_count--; }
void IRAM_ATTR isrFL() { if (digitalRead(FL_A) == digitalRead(FL_B)) FL_count++; else FL_count--; }
void IRAM_ATTR isrFR() { if (digitalRead(FR_A) == digitalRead(FR_B)) FR_count++; else FR_count--; }


// === RPM Calculation ===
float calcRPM(long pulses) {
  const int PPR = 100; // replace with your encoder’s pulses per revolution
  return (pulses / (float)PPR) * 60.0;
}

// === HC-SR04 Distance Function ===
long readHCSR(int echoPin) {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // timeout 30ms
  long distance = duration * 0.034 / 2;          // cm
  return distance;
}

// Median filter wrapper
float medianFilter(int echoPin) {
  float vals[5];
  for (int i=0; i<5; i++) {
    vals[i] = readHCSR(echoPin);
    delay(5);
  }
  std::sort(vals, vals+5);
  return vals[2];
}


// --- Kalman Prediction ---
void kalmanPredict(float dt, int16_t gz, long RL, long RR, long FL, long FR) {
  float dL = ((RL + FL) / 2.0) * tickToCm;
  float dR = ((RR + FR) / 2.0) * tickToCm;
  float d = (dL + dR) / 2.0;
  float dTheta = (dR - dL) / wheelbase;

  // Drift compensation: if slip detected, trust gyro more
  if (abs(dL - dR) > 10.0) { // slip threshold in cm
    dTheta = ((gz - gyroOffset) / gyroScale) * (PI/180.0) * dt;
  }

  // Gyro corrected
  float gyroZ = (gz - gyroOffset) / gyroScale; // °/s
  float gyroZrad = gyroZ * (PI/180.0);

  // Predict state
  X(0) += d * cos(X(2));
  X(1) += d * sin(X(2));
  X(2) += dTheta + gyroZrad * dt;
  X(3) = d / dt;
  X(4) = gyroZrad;

  // Predict covariance
  BLA::Matrix<5,5> F = BLA::Identity<5,5>();
  F(0,2) = -X(3)*dt*sin(X(2));
  F(0,3) = dt*cos(X(2));
  F(1,2) =  X(3)*dt*cos(X(2));
  F(1,3) = dt*sin(X(2));
  F(2,4) = dt;
  P = F * P * ~F + Q;
}

// --- Kalman Update ---
void kalmanUpdate(BLA::Matrix<5,1> Z, BLA::Matrix<5,5> H, BLA::Matrix<5,5> R) {
  BLA::Matrix<5,1> y = Z - H*X;
  BLA::Matrix<5,5> S = H*P*~H + R;
  BLA::Matrix<5,5> K = P*~H*Inverse(S);
  X = X + K*y;
  P = (BLA::Identity<5,5>() - K*H)*P;
}

// --- Send axis via UART ---
void sendAxis(int id, float val) {
  Serial2.print("AXIS ");
  Serial2.print(id);
  Serial2.print(" ");
  Serial2.println(val, 3);
}

// --- Setup ---
MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  // Encoders
  pinMode(RL_D1, INPUT); pinMode(RL_D2, INPUT);
  pinMode(RR_D1, INPUT); pinMode(RR_D2, INPUT);
  pinMode(FL_D1, INPUT); pinMode(FL_D2, INPUT);
  pinMode(FR_D1, INPUT); pinMode(FR_D2, INPUT);
  pinMode(EL_D1, INPUT); pinMode(EL_D2, INPUT);
  pinMode(ER_D1, INPUT); pinMode(ER_D2, INPUT);

  attachInterrupt(digitalPinToInterrupt(RL_D1), RL_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RR_D1), RR_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(FL_D1), FL_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(FR_D1), FR_ISR, RISING);

  // Ultrasonics
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(ECHO_RIGHT, INPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(ECHO_REAR, INPUT);

  // MPU6050
  Wire.begin(SDA_PIN, SCL_PIN);
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 bağlantı hatası!");
    while (1);
  } else {
    Serial.println("MPU6050 bağlantısı başarılı.");
  }

  // WiFi
  WiFi.begin("Fox-2", "Kyra2bin9");
  Serial.print("WiFi Baglaniyor");
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 20) {
    delay(500);
    Serial.print(".");
    timeout++;
  }
  Serial.println("\nWiFi OK!");
  Serial.print("IP: "); Serial.println(WiFi.localIP());
  udp.begin(localPort);
  Serial.println("UDP Basladi");
}