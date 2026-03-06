/*
 * ESP32-S3-N16R8 MASTER CONTROLLER CODE
 * 
 * This is the main robot controller that:
 * - Receives WiFi commands from computer (Core 0)
 * - Reads all sensors and performs localization (Core 1)
 * - Implements Kalman filter for position tracking
 * - Runs PID-based waypoint navigation for first 60 seconds
 * - Passes through WiFi commands after 60-second autonomous mode
 * - Communicates with slave board (N8R2) via UART for motor control
 * 
 * CORE 0: WiFi & Command Processing
 * CORE 1: Sensor Reading, Kalman Filtering, PID Navigation
 */

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

// ============================================================================
// DEFINES & CONSTANTS
// ============================================================================

// Elevator pulse threshold calculation:
// 20 pulses/rotation × (200 RPM / 60 sec) × 2 sec = 133 pulses
const int ELEVATOR_PULSE_LIMIT = 133;

// Autonomous mode duration (seconds)
const unsigned long AUTONOMOUS_DURATION = 60000; // 60 seconds in milliseconds

// ============================================================================
// KALMAN FILTER STATE
// ============================================================================

// State vector X = [x, y, theta, v, omega]
// x, y: position in cm
// theta: heading angle in radians
// v: linear velocity in cm/s
// omega: angular velocity in rad/s
BLA::Matrix<5,1> X = {0, 0, 0, 0, 0};

// Covariance matrix P (uncertainty in state estimates)
BLA::Matrix<5,5> P = BLA::Identity<5,5>() * 0.1;

// Process noise matrix Q (how much we trust the model)
BLA::Matrix<5,5> Q = BLA::Identity<5,5>() * 0.01;

// Measurement noise matrix R for ultrasonic sensors
BLA::Matrix<5,5> R_ultra = BLA::Identity<5,5>() * 0.5;

// ============================================================================
// PIN DEFINITIONS - ENCODERS (Main Chassis Motors)
// ============================================================================

// Rear Left (RL) motor encoder pins
#define RL_A 1
#define RL_B 2

// Rear Right (RR) motor encoder pins
#define RR_A 4
#define RR_B 5

// Front Left (FL) motor encoder pins
#define FL_A 6
#define FL_B 7

// Front Right (FR) motor encoder pins
#define FR_A 8
#define FR_B 9

// ============================================================================
// PIN DEFINITIONS - ELEVATOR ENCODERS
// ============================================================================

// Elevator Left (EL) motor encoder pins
#define EL_A 10
#define EL_B 11

// Elevator Right (ER) motor encoder pins
#define ER_A 12
#define ER_B 13

// ============================================================================
// PIN DEFINITIONS - ULTRASONIC SENSORS (HC-SR04)
// ============================================================================

// Common trigger pin for all 4 HC-SR04 sensors
const int TRIG_PIN = 14;

// Echo pins for each direction
const int ECHO_FRONT = 15;  // Front sensor
const int ECHO_RIGHT = 16;  // Right sensor
const int ECHO_LEFT = 17;   // Left sensor
const int ECHO_REAR = 18;   // Rear sensor

// ============================================================================
// PIN DEFINITIONS - MPU6050 (IMU - Inertial Measurement Unit)
// ============================================================================

// I2C communication pins for MPU6050
const int SDA_PIN = 41;
const int SCL_PIN = 42;

// ============================================================================
// PIN DEFINITIONS - UART TO SLAVE BOARD (N8R2)
// ============================================================================

// Serial2 pins for communicating with N8R2
const int RX_PIN = 47;  // Receive from slave
const int TX_PIN = 20;  // Transmit to slave

// ============================================================================
// CALIBRATION CONSTANTS
// ============================================================================

// Main chassis wheel specifications
const float WHEEL_CIRCUMFERENCE = 31.4;  // cm (for mecanum wheels)
const int MAIN_TICKS_PER_REV = 48;       // pulses per rotation of motor shaft
const float TICK_TO_CM = WHEEL_CIRCUMFERENCE / MAIN_TICKS_PER_REV;
const float WHEELBASE = 31.75;            // distance between front-rear axles in cm

// MPU6050 gyroscope calibration
const float GYRO_SCALE = 131.0;           // LSB/°/s for ±250 range
const float GYRO_OFFSET = -135.0;         // measured offset (gyro drift compensation)

// ============================================================================
// VOLATILE VARIABLES - ENCODER COUNTERS (for ISRs)
// ============================================================================

// Main chassis encoder pulse counts
volatile long count_RL = 0;  // Rear Left counter
volatile long count_RR = 0;  // Rear Right counter
volatile long count_FL = 0;  // Front Left counter
volatile long count_FR = 0;  // Front Right counter

// Elevator encoder pulse counts
volatile long count_EL = 0;  // Elevator Left counter
volatile long count_ER = 0;  // Elevator Right counter

// ============================================================================
// VARIABLES - WIFI & COMMAND PROCESSING
// ============================================================================

// WiFi UDP for receiving commands from computer
WiFiUDP udp;
unsigned int WIFI_LOCAL_PORT = 4210;
char wifi_packet_buffer[255];

// Current command values received via WiFi
float cmd_axis_0 = 0;   // Turn command (-1 to 1)
float cmd_axis_2 = 0;   // Strafe command (-1 to 1)
float cmd_axis_4 = 0;   // Backward command (-1 to 1)
float cmd_axis_5 = 0;   // Forward command (-1 to 1)

int cmd_button_6 = 0;   // Servo door toggle
int cmd_button_12 = 0;  // Elevator down
int cmd_button_13 = 0;  // Elevator up

// ============================================================================
// VARIABLES - AUTONOMOUS MODE
// ============================================================================

// Waypoint structure for autonomous navigation
struct Waypoint {
  float x;  // X coordinate in cm
  float y;  // Y coordinate in cm
};

// Waypoint array (user fills in coordinates)
// Example: {100, 50} means go to position 100cm in X, 50cm in Y
Waypoint waypoints[] = {
  // EDIT THESE WAYPOINTS WITH YOUR TARGET COORDINATES
  // Format: {x_cm, y_cm}
  // {100, 0},    // Waypoint 1
  // {100, 100},  // Waypoint 2
  // {0, 100},    // Waypoint 3
  // {0, 0},      // Return to start
};

int current_waypoint = 0;
int num_waypoints = sizeof(waypoints) / sizeof(Waypoint);

// PID parameters for navigation
float Kp_forward = 0.02;   // Proportional gain for distance control
float Kp_turn = 0.5;       // Proportional gain for angle control

// Autonomous mode timing
unsigned long autonomous_start_time = 0;
bool autonomous_mode_active = true;  // True for first 60s, then false

// ============================================================================
// VARIABLES - TIMING
// ============================================================================

unsigned long last_time = 0;            // For Kalman filter dt calculation
unsigned long last_debug_print = 0;     // For debug output timing
const unsigned long DEBUG_INTERVAL = 500; // Print debug info every 500ms

// ============================================================================
// VARIABLES - MOTOR RPM & SENSOR DISPLAY FLAGS
// ============================================================================

bool show_main_rpm = false;   // Flag to enable/disable RPM display
bool show_elevator_rpm = false;
bool show_ultrasonic = false; // Flag to enable/disable ultrasonic display

// ============================================================================
// INTERRUPT SERVICE ROUTINES - MAIN CHASSIS ENCODERS
// ============================================================================

// These ISRs count encoder pulses in real-time
// Direction determined by comparing phase A and phase B signals

void IRAM_ATTR isr_RL() {
  if (digitalRead(RL_A) == digitalRead(RL_B)) {
    count_RL++;  // Forward
  } else {
    count_RL--;  // Reverse
  }
}

void IRAM_ATTR isr_RR() {
  if (digitalRead(RR_A) == digitalRead(RR_B)) {
    count_RR++;
  } else {
    count_RR--;
  }
}

void IRAM_ATTR isr_FL() {
  if (digitalRead(FL_A) == digitalRead(FL_B)) {
    count_FL++;
  } else {
    count_FL--;
  }
}

void IRAM_ATTR isr_FR() {
  if (digitalRead(FR_A) == digitalRead(FR_B)) {
    count_FR++;
  } else {
    count_FR--;
  }
}

// ============================================================================
// INTERRUPT SERVICE ROUTINES - ELEVATOR ENCODERS
// ============================================================================

void IRAM_ATTR isr_EL() {
  if (digitalRead(EL_A) == digitalRead(EL_B)) {
    count_EL++;  // Up movement
  } else {
    count_EL--;  // Down movement
  }
}

void IRAM_ATTR isr_ER() {
  if (digitalRead(ER_A) == digitalRead(ER_B)) {
    count_ER++;  // Up movement (mirrored from EL)
  } else {
    count_ER--;  // Down movement
  }
}

// ============================================================================
// SENSOR READING FUNCTIONS
// ============================================================================

/*
 * Read distance from one HC-SR04 ultrasonic sensor
 * Returns distance in cm, or -1 if timeout
 */
long readHCSR(int echo_pin) {
  // Trigger ultrasonic pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure echo duration (with 30ms timeout)
  long duration = pulseIn(echo_pin, HIGH, 30000);
  
  if (duration == 0) {
    return -1;  // Timeout, no valid reading
  }
  
  // Convert duration to distance: (duration × speed of sound) / 2
  // Speed of sound = 0.034 cm/µs
  long distance = duration * 0.034 / 2;
  return distance;
}

/*
 * Read 5 measurements and return median (noise filtering)
 * Eliminates outliers from a single noisy reading
 */
float medianFilterUltrasonic(int echo_pin) {
  float vals[5];
  
  // Take 5 readings
  for (int i = 0; i < 5; i++) {
    vals[i] = readHCSR(echo_pin);
    delay(5);  // Small delay between readings
  }
  
  // Sort values
  std::sort(vals, vals + 5);
  
  // Return median (middle value eliminates outliers)
  return vals[2];
}

/*
 * Read full IMU data (accelerometer + gyro)
 */
void readRawIMU(MPU6050 &mpu, int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  // getMotion6 returns raw accel (AX,AY,AZ) and gyro (GX,GY,GZ)
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
}

// Complementary filter: fuse accel + gyro for roll and pitch, and compute filtered gyroZ (rad/s)
void complementaryFilterUpdate(MPU6050 &mpu, float dt) {
  int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
  readRawIMU(mpu, ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw);

  // Convert accel to g (assumes default ±2g: 16384 LSB/g). Adjust if different.
  float ax = ax_raw / 16384.0f;
  float ay = ay_raw / 16384.0f;
  float az = az_raw / 16384.0f;

  // Compute accelerometer angles (radians)
  float roll_acc = atan2f(ay, az);
  float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az));

  // Convert gyro raw to deg/s and apply offsets
  float gx = (gx_raw - gyro_x_offset) / GYRO_SCALE; // deg/s
  float gy = (gy_raw - gyro_y_offset) / GYRO_SCALE; // deg/s
  float gz = (gz_raw - gyro_z_offset) / GYRO_SCALE; // deg/s

  // Convert to rad/s
  float gx_rad = gx * (PI / 180.0f);
  float gy_rad = gy * (PI / 180.0f);
  float gz_rad = gz * (PI / 180.0f);

  if (!imu_initialized) {
    // Initialize filter with accel angles to avoid large transients
    comp_roll = roll_acc;
    comp_pitch = pitch_acc;
    imu_initialized = true;
  } else {
    // Complementary filter update
    comp_roll = COMP_ALPHA * (comp_roll + gx_rad * dt) + (1.0f - COMP_ALPHA) * roll_acc;
    comp_pitch = COMP_ALPHA * (comp_pitch + gy_rad * dt) + (1.0f - COMP_ALPHA) * pitch_acc;
  }

  // Filtered yaw rate (we cannot get absolute yaw without magnetometer; use filtered rate)
  filtered_gz_rad = gz_rad;
}

// ============================================================================
// KALMAN FILTER FUNCTIONS
// ============================================================================

/*
 * Kalman Filter Prediction Step
 * 
 * Updates state estimate based on:
 * - Motor encoder readings (odometry)
 * - Gyroscope Z-axis reading (heading)
 * - Time since last update (dt)
 * 
 * Uses slip detection: if left and right wheel displacements differ
 * significantly, trusts gyro more than wheel difference for angle
 */
void kalmanPredict(float dt, float gz_rad, long RL, long RR, long FL, long FR) {
  // Calculate displacement from encoder counts
  float dL = ((RL + FL) / 2.0) * TICK_TO_CM;  // Left side avg displacement
  float dR = ((RR + FR) / 2.0) * TICK_TO_CM;  // Right side avg displacement
  float d = (dL + dR) / 2.0;                  // Average displacement
  float dTheta = (dR - dL) / WHEELBASE;       // Heading change from wheel difference

  // Slip detection: if difference > 10cm, wheel slipping occurred
  // When slipping, trust gyro more for angle estimate
  if (abs(dL - dR) > 10.0) {
    dTheta = gz_rad * dt; // gz_rad is already in rad/s
  }

  // Update state vector using motion model
  X(0) += d * cos(X(2));      // X position update
  X(1) += d * sin(X(2));      // Y position update
  X(2) += dTheta + gz_rad * dt;  // Theta (heading) update
  X(3) = d / dt;              // Linear velocity estimate
  X(4) = gz_rad;              // Angular velocity estimate

  // Predict covariance matrix (uncertainty grows with time)
  BLA::Matrix<5,5> F = BLA::Identity<5,5>();
  F(0, 2) = -X(3) * dt * sin(X(2));
  F(0, 3) = dt * cos(X(2));
  F(1, 2) = X(3) * dt * cos(X(2));
  F(1, 3) = dt * sin(X(2));
  F(2, 4) = dt;
  
  P = F * P * ~F + Q;  // Updated covariance
}

/*
 * Kalman Filter Update Step
 * 
 * Corrects state estimate using sensor measurements
 * (in this case: ultrasonic distances from walls)
 */
void kalmanUpdate(BLA::Matrix<5,1> Z, BLA::Matrix<5,5> H, BLA::Matrix<5,5> R) {
  // Innovation: difference between measured and predicted values
  BLA::Matrix<5,1> y = Z - H * X;
  
  // Innovation covariance
  BLA::Matrix<5,5> S = H * P * ~H + R;
  
  // Kalman gain (how much to trust the measurement)
  BLA::Matrix<5,5> K = P * ~H * Inverse(S);
  
  // Update state estimate
  X = X + K * y;
  
  // Update covariance (uncertainty decreases)
  P = (BLA::Identity<5,5>() - K * H) * P;
}

// ============================================================================
// PID NAVIGATION FUNCTIONS
// ============================================================================

/*
 * Calculate desired motor command to reach target waypoint
 * Uses simple proportional control (P only, no I or D for now)
 */
void navigationToPID(float target_x, float target_y) {
  // Calculate error to target
  float error_x = target_x - X(0);
  float error_y = target_y - X(1);
  float distance_to_target = sqrt(error_x * error_x + error_y * error_y);

  // Calculate desired heading to target
  float desired_angle = atan2(error_y, error_x);
  float angle_error = desired_angle - X(2);

  // Normalize angle error to [-π, π]
  while (angle_error > PI) angle_error -= 2 * PI;
  while (angle_error < -PI) angle_error += 2 * PI;

  // PID commands
  float forward_cmd = Kp_forward * distance_to_target;
  float turn_cmd = Kp_turn * angle_error;

  // Clamp to [-1, 1]
  forward_cmd = constrain(forward_cmd, -1.0, 1.0);
  turn_cmd = constrain(turn_cmd, -1.0, 1.0);

  // Send to slave board
  sendCommandToSlave(forward_cmd, 0, turn_cmd, 0, 0, 0, 0);  // Only forward + turn

  // Debug output
  Serial.printf("[NAV] Target:(%.1f,%.1f) Current:(%.1f,%.1f) Distance:%.1f Angle_Error:%.2f Cmd:F:%.2f T:%.2f\n",
                target_x, target_y, X(0), X(1), distance_to_target, angle_error, forward_cmd, turn_cmd);
}

// ============================================================================
// COMMUNICATION FUNCTIONS
// ============================================================================

/*
 * Send motor control command to slave board (N8R2) via UART
 * 
 * Command format: "MOV forward strafe turn servo_door elevator_up elevator_down spare"
 * 
 * Parameters are comma-separated floats from -1.0 to 1.0
 * Slave will ensure no mixing of movements
 */
void sendCommandToSlave(float forward, float strafe, float turn, 
                        float servo_door, float elevator_up, float elevator_down, float spare) {
  // Format: "MOV f s t d u d s\n"
  Serial2.printf("MOV %.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", 
                 forward, strafe, turn, servo_door, elevator_up, elevator_down, spare);
}

/*
 * Parse WiFi command received from computer
 * Format examples:
 *   "AXIS 4 0.8"    -> forward movement
 *   "AXIS 0 -0.5"   -> left turn
 *   "BUTTON 13 1"   -> elevator up pressed
 */
void parseWiFiCommand(String msg) {
  msg.trim();
  
  // Find spaces
  int space1 = msg.indexOf(' ');
  int space2 = msg.indexOf(' ', space1 + 1);
  
  if (space1 == -1 || space2 == -1) return;  // Invalid format
  
  String type = msg.substring(0, space1);
  String id_str = msg.substring(space1 + 1, space2);
  String val_str = msg.substring(space2 + 1);
  
  int id = id_str.toInt();
  
  if (type == "AXIS") {
    float val = val_str.toFloat();
    if (id == 0) cmd_axis_0 = val;      // Turn
    else if (id == 2) cmd_axis_2 = val; // Strafe
    else if (id == 4) cmd_axis_4 = val; // Backward
    else if (id == 5) cmd_axis_5 = val; // Forward
  } else if (type == "BUTTON") {
    int val = val_str.toInt();
    if (id == 6) cmd_button_6 = val;    // Servo
    else if (id == 12) cmd_button_12 = val;  // Elevator down
    else if (id == 13) cmd_button_13 = val;  // Elevator up
  }
}

// ============================================================================
// DEBUG OUTPUT FUNCTION
// ============================================================================

/*
 * Print comprehensive debug information about:
 * - Robot position and heading (from Kalman filter)
 * - Main motor RPM values
 * - Elevator position and limits
 * - Ultrasonic sensor distances
 * - MPU6050 gyro readings
 * - Current commands being executed
 */
void debugPrintAllSensors(MPU6050& mpu) {
  unsigned long now = millis();
  
  if (now - last_debug_print < DEBUG_INTERVAL) {
    return;  // Don't print too frequently
  }
  last_debug_print = now;

  // Calculate main motor RPM
  float rpm_RL = (count_RL / (float)MAIN_TICKS_PER_REV) * 60.0;
  float rpm_RR = (count_RR / (float)MAIN_TICKS_PER_REV) * 60.0;
  float rpm_FL = (count_FL / (float)MAIN_TICKS_PER_REV) * 60.0;
  float rpm_FR = (count_FR / (float)MAIN_TICKS_PER_REV) * 60.0;

  // Read gyro
  int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
  readRawIMU(mpu, ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw);

  // Read ultrasonic sensors
  float dist_front = medianFilterUltrasonic(ECHO_FRONT);
  float dist_right = medianFilterUltrasonic(ECHO_RIGHT);
  float dist_left = medianFilterUltrasonic(ECHO_LEFT);
  float dist_rear = medianFilterUltrasonic(ECHO_REAR);

  // Print all sensor data
  Serial.println("\n========== ROBOT SENSOR DEBUG ==========");
  
  Serial.printf("POSITION: X=%.1f cm, Y=%.1f cm, Theta=%.2f rad (%.1f°)\n",
                X(0), X(1), X(2), X(2) * 180.0 / PI);
  
  Serial.printf("VELOCITY: Linear=%.2f cm/s, Angular=%.4f rad/s\n",
                X(3), X(4));
  
  Serial.printf("ENCODERS: RL=%ld, RR=%ld, FL=%ld, FR=%ld, EL=%ld, ER=%ld\n",
                count_RL, count_RR, count_FL, count_FR, count_EL, count_ER);
  
  Serial.printf("RPM: RL=%.1f, RR=%.1f, FL=%.1f, FR=%.1f\n",
                rpm_RL, rpm_RR, rpm_FL, rpm_FR);
  
  Serial.printf("ELEVATOR: EL_Pulses=%ld, ER_Pulses=%ld, At_Limit=%s\n",
                count_EL, count_ER, (count_EL >= ELEVATOR_PULSE_LIMIT) ? "YES" : "NO");
  
  Serial.printf("ULTRASONIC: Front=%.1f cm, Right=%.1f cm, Left=%.1f cm, Rear=%.1f cm\n",
                dist_front, dist_right, dist_left, dist_rear);
  
  float gz_deg = (gz_raw - gyro_z_offset) / GYRO_SCALE;
  Serial.printf("GYRO_Z_RAW: %d (offset-corrected: %.2f °/s)\n",
                gz_raw, gz_deg);

  Serial.printf("IMU: Roll=%.2f°, Pitch=%.2f°, Filtered_GyroZ=%.2f °/s\n",
                comp_roll * 180.0 / PI, comp_pitch * 180.0 / PI, filtered_gz_rad * 180.0 / PI);
  
  Serial.printf("COMMANDS: Forward=%.2f, Strafe=%.2f, Turn=%.2f, Elevator_Up=%d, Elevator_Down=%d\n",
                cmd_axis_5, cmd_axis_2, cmd_axis_0, cmd_button_13, cmd_button_12);
  
  Serial.printf("AUTONOMOUS: Mode=%s, Time=%lu ms / %lu ms\n",
                autonomous_mode_active ? "ACTIVE" : "MANUAL",
                now - autonomous_start_time, AUTONOMOUS_DURATION);
  
  Serial.println("========================================\n");
}

// ============================================================================
// SETUP FUNCTION (RUNS ONCE)
// ============================================================================

MPU6050 mpu;

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n=== ESP32-S3-N16R8 MASTER STARTING ===\n");

  // Initialize Serial2 for slave communication
  Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(100);
  Serial.println("[INIT] Serial2 (to N8R2) initialized");

  // ========== PIN SETUP ==========
  
  // Main chassis encoder pins
  pinMode(RL_A, INPUT);
  pinMode(RL_B, INPUT);
  pinMode(RR_A, INPUT);
  pinMode(RR_B, INPUT);
  pinMode(FL_A, INPUT);
  pinMode(FL_B, INPUT);
  pinMode(FR_A, INPUT);
  pinMode(FR_B, INPUT);

  // Elevator encoder pins
  pinMode(EL_A, INPUT);
  pinMode(EL_B, INPUT);
  pinMode(ER_A, INPUT);
  pinMode(ER_B, INPUT);

  // Ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(ECHO_RIGHT, INPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(ECHO_REAR, INPUT);

  Serial.println("[INIT] All pins configured");

  // ========== ENCODER INTERRUPTS ==========
  
  // Attach main chassis encoder ISRs
  attachInterrupt(digitalPinToInterrupt(RL_A), isr_RL, RISING);
  attachInterrupt(digitalPinToInterrupt(RR_A), isr_RR, RISING);
  attachInterrupt(digitalPinToInterrupt(FL_A), isr_FL, RISING);
  attachInterrupt(digitalPinToInterrupt(FR_A), isr_FR, RISING);

  // Attach elevator encoder ISRs
  attachInterrupt(digitalPinToInterrupt(EL_A), isr_EL, RISING);
  attachInterrupt(digitalPinToInterrupt(ER_A), isr_ER, RISING);

  Serial.println("[INIT] Encoder interrupts attached");

  // ========== MPU6050 INITIALIZATION ==========
  
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100);
  
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("[ERROR] MPU6050 connection failed!");
    while (1);  // Halt if sensor not found
  }
  Serial.println("[INIT] MPU6050 initialized successfully");

  // ========== WIFI INITIALIZATION ==========
  
  WiFi.begin("Fox-2", "Kyra2bin9");
  Serial.print("[INIT] Connecting to WiFi");
  
  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 20) {
    delay(500);
    Serial.print(".");
    timeout++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[INIT] WiFi connected!");
    Serial.print("[INFO] IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n[WARN] WiFi connection timeout (will work in manual mode)");
  }

  // Start UDP
  udp.begin(WIFI_LOCAL_PORT);
  Serial.printf("[INIT] UDP listening on port %d\n", WIFI_LOCAL_PORT);

  // ========== SETUP COMPLETE ==========
  
  autonomous_start_time = millis();
  last_time = millis();
  
  Serial.println("\n=== INITIALIZATION COMPLETE ===");
  Serial.println("Autonomous mode: 60 seconds of waypoint navigation");
  Serial.println("After 60s: Will accept WiFi commands for manual control\n");
}

// ============================================================================
// CORE 0 TASK - WIFI COMMAND RECEPTION
// ============================================================================

void core0WiFiTask(void* pvParameters) {
  while (1) {
    // Check for incoming WiFi commands
    int packet_size = udp.parsePacket();
    if (packet_size > 0) {
      int len = udp.read(wifi_packet_buffer, 255);
      if (len > 0) {
        wifi_packet_buffer[len] = 0;  // Null terminate
        String cmd(wifi_packet_buffer);
        parseWiFiCommand(cmd);
      }
    }
    
    delay(10);  // Don't hog CPU
  }
}

// ============================================================================
// CORE 1 TASK - SENSOR READING & PID NAVIGATION (MAIN LOOP)
// ============================================================================

void core1SensorTask(void* pvParameters) {
  MPU6050 mpu;
  mpu.initialize();
  
  while (1) {
    unsigned long now = millis();
    float dt = (now - last_time) / 1000.0;  // Delta time in seconds
    
    if (dt < 0.01) {  // Minimum 10ms between updates
      delay(1);
      continue;
    }

    // IMU: update complementary filter (roll/pitch) and filtered yaw rate
    complementaryFilterUpdate(mpu, dt);

    // Safety: if robot is tipping (large roll/pitch), stop immediately and skip motion
    if (fabs(comp_roll) > 0.52f || fabs(comp_pitch) > 0.52f) { // ~30 degrees
      Serial.println("[SAFETY] Excessive tilt detected! Stopping motors.");
      // Emergency stop - send zero movement command to slave
      sendCommandToSlave(0, 0, 0, 0, 0, 0, 0);
      // Reset encoder counters and skip navigation this cycle
      count_RL = count_RR = count_FL = count_FR = 0;
      last_time = now;
      delay(10);
      continue;
    }

    // ========== KALMAN PREDICTION ==========
    // Update position estimate based on odometry and filtered gyro (rad/s)
    kalmanPredict(dt, filtered_gz_rad, count_RL, count_RR, count_FL, count_FR);

    // Reset encoder counts for next iteration
    count_RL = 0;
    count_RR = 0;
    count_FL = 0;
    count_FR = 0;

    // ========== DECISION LOGIC ==========
    
    unsigned long elapsed_autonomous = now - autonomous_start_time;
    
    if (autonomous_mode_active && elapsed_autonomous < AUTONOMOUS_DURATION) {
      // ===== AUTONOMOUS MODE (First 60 seconds) =====
      // Follow waypoints, ignore WiFi commands
      
      if (num_waypoints > 0) {
        navigationToPID(waypoints[current_waypoint].x, waypoints[current_waypoint].y);
      }
    } 
    else {
      // ===== MANUAL MODE (After 60 seconds) =====
      // Pass through WiFi commands directly
      
      if (!autonomous_mode_active) {
        autonomous_mode_active = false;  // Flag that we've transitioned
        Serial.println("\n[TRANSITION] Autonomous mode ended - accepting manual commands\n");
      }

      // Send WiFi commands directly to slave
      float forward = cmd_axis_5 - cmd_axis_4;  // 5=forward, 4=backward
      float strafe = cmd_axis_2;
      float turn = cmd_axis_0;
      
      sendCommandToSlave(forward, strafe, turn, cmd_button_6, cmd_button_13, cmd_button_12, 0);
    }

    // ========== DEBUG OUTPUT ==========
    debugPrintAllSensors(mpu);

    last_time = now;
    delay(5);  // Minimum loop delay
  }
}

// ============================================================================
// MAIN LOOP - CORE 0 (just kicks off the tasks)
// ============================================================================

void loop() {
  // This just runs the WiFi task on core 0
  // The sensor task runs on core 1
  // We'll create these tasks in setup with proper core affinity
  delay(1000);
}

// ============================================================================
// SETUP MODIFICATION - CREATE FREERTOS TASKS
// ============================================================================

// Modify setup() to add this before setup() ends:
/*
void createTasks() {
  // Create WiFi task on Core 0
  xTaskCreatePinnedToCore(
    core0WiFiTask,     // Task function
    "WiFi_Task",       // Task name
    4096,              // Stack size
    NULL,              // Parameter
    1,                 // Priority
    NULL,              // Task handle
    0                  // Core 0
  );

  // Create Sensor/PID task on Core 1
  xTaskCreatePinnedToCore(
    core1SensorTask,   // Task function
    "Sensor_Task",     // Task name
    8192,              // Stack size (larger for sensor + Kalman)
    NULL,              // Parameter
    1,                 // Priority
    NULL,              // Task handle
    1                  // Core 1
  );
}
*/

// ADD THIS TO END OF SETUP():
// createTasks();
