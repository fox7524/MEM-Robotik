#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

// ESP32-N8R2 MAIN CONTROLLER CODE
// Controls 4-wheel drive robot with elevator and servo via UDP from Python controller
 
// --- MOTOR PIN DEFINITIONS ---
//Rear Left Pins
const int RL_PWM = 7;   // PWM pin for rear left motor speed
const int RL_IN1 = 8;   // Direction control 1 for rear left
const int RL_IN2 = 9;   // Direction control 2 for rear left

//Rear Right Pins
const int RR_PWM = 12;  // PWM pin for rear right motor speed
const int RR_IN1 = 11;  // Direction control 1 for rear right
const int RR_IN2 = 13;  // Direction control 2 for rear right

//Front Left Pins
const int FL_PWM = 15;  // PWM pin for front left motor speed
const int FL_IN1 = 16;  // Direction control 1 for front left
const int FL_IN2 = 17;  // Direction control 2 for front left

//Front Right Pins
const int FR_PWM = 4;   // PWM pin for front right motor speed
const int FR_IN1 = 5;   // Direction control 1 for front right
const int FR_IN2 = 6;   // Direction control 2 for front right

// --- ELEVATOR LEFT MOTOR ---
const int EL_PWM = 18;  // PWM pin for elevator left motor
const int EL_IN1 = 38;  // Direction control 1 for elevator left
const int EL_IN2 = 39;  // Direction control 2 for elevator left

// --- ELEVATOR RIGHT MOTOR ---
const int ER_PWM = 1;   // PWM pin for elevator right motor
const int ER_IN1 = 2;   // Direction control 1 for elevator right
const int ER_IN2 = 42;  // Direction control 2 for elevator right

// --- SERVO LID ---
const int E_LID = 41;   // Servo pin for elevator lid
const int B_LID = 14;   // Unused servo pin
const int H_LID = 40;   // Unused servo pin

// --- NETWORKING ---
WiFiUDP udp;
unsigned int localPort = 4210;  // UDP port for receiving commands
char packetBuffer[255];         // Buffer for incoming UDP packets
unsigned long lastUpdate = 0;   // Timestamp for last update (unused)

// --- INPUT VARIABLES ---
// Parsed joystick axis values (-1.0 to 1.0)
float a0, a2, a4, a5;  // a0: turn, a2: strafe, a4: backward, a5: forward

// Parsed button states (0 or 1)
int b0, b11, b12, b13, b6;  // b6: servo toggle, b12: elevator down, b13: elevator up

// Servo control variables
int previous_b6 = 0;         // Previous state of button 6 for edge detection
bool e_lid_open = false;     // Current state of elevator lid servo

// Elevator control variables
unsigned long elevator_start_time = 0;  // Start time for elevator movement timing
int elevator_position = 0;              // Current elevator position in ms (0-3000)
bool elevator_moving = false;           // Flag indicating if elevator is currently moving

// Servo object
Servo e_lid_servo;

void setup() {
  //========PIN MODE=========
  // Set all motor and servo pins as outputs
  pinMode(RR_IN1, OUTPUT);
  pinMode(RR_IN2, OUTPUT);
  pinMode(RR_PWM, OUTPUT);

  pinMode(RL_IN1, OUTPUT);
  pinMode(RL_IN2, OUTPUT);
  pinMode(RL_PWM, OUTPUT);

  pinMode(FR_IN1, OUTPUT);
  pinMode(FR_IN2, OUTPUT);
  pinMode(FR_PWM, OUTPUT);

  pinMode(FL_IN1, OUTPUT);
  pinMode(FL_IN2, OUTPUT);
  pinMode(FL_PWM, OUTPUT);

  pinMode(EL_IN1, OUTPUT);
  pinMode(EL_IN2, OUTPUT);
  pinMode(EL_PWM, OUTPUT);

  pinMode(ER_IN1, OUTPUT);
  pinMode(ER_IN2, OUTPUT);
  pinMode(ER_PWM, OUTPUT);

  pinMode(E_LID, OUTPUT);
  pinMode(H_LID, OUTPUT);
  pinMode(B_LID, OUTPUT);

  // Attach servo to pin
  e_lid_servo.attach(E_LID);

  // Initialize serial communication for debugging
  Serial.begin(115200);
  delay(500);

  Serial.print("ESP-32s3 N8R2 Başlatildi");

  //Wifi Begin
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

void loop() {
  // Main control loop - runs continuously
  
  // Check WiFi connection and restart if disconnected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi Koptu, Restart...");
    ESP.restart();
  }

  // Check for incoming UDP packets
  int packetSize = udp.parsePacket();
  if (packetSize) {
    // Packet received - parse and process
    int len = udp.read(packetBuffer, 255);
    if (len > 0 && len < 255) {
      packetBuffer[len] = 0;
      String msg = String(packetBuffer);
      
      // Parse the incoming message (AXIS/BUTTON commands)
      parseInput(msg);
      
      // Debug output: print current axis and button values
      Serial.printf("A0:%.2f A2:%.2f A4:%.2f A5:%.2f B0:%d B6:%d B11:%d B12:%d B13:%d\n", a0, a2, a4, a5, b0, b6, b11, b12, b13);
      
      // Process movement based on parsed inputs
      processMovement();
    }
  } else {
    // No packet received - stop all motors and handle continuous functions
    stopAll();          // Emergency stop with braking
    controlElevator();  // Handle elevator timing and control
    controlServo();     // Handle servo control
  }
}



void anidur(){
  // Emergency stop with active braking
  // Sets all motor direction pins high to short brake, then stops
  digitalWrite(RR_IN1, HIGH);
  digitalWrite(RR_IN2, HIGH);
  
  digitalWrite(FR_IN1, HIGH);
  digitalWrite(FR_IN2, HIGH);
  
  digitalWrite(RL_IN1, HIGH);
  digitalWrite(RL_IN2, HIGH);
  
  digitalWrite(FL_IN1, HIGH);
  digitalWrite(FL_IN2, HIGH);
  
  analogWrite(RR_PWM, 255);
  analogWrite(FR_PWM, 255);
  analogWrite(RL_PWM, 255);
  analogWrite(FL_PWM, 255);
  
  delay(30);  // Braking duration
  
  // Stop all motors
  digitalWrite(RR_IN1, LOW);
  digitalWrite(RR_IN2, LOW);
  
  digitalWrite(FR_IN1, LOW);
  digitalWrite(FR_IN2, LOW);
  
  digitalWrite(RL_IN1, LOW);
  digitalWrite(RL_IN2, LOW);
  
  digitalWrite(FL_IN1, LOW);
  digitalWrite(FL_IN2, LOW);
  
  analogWrite(RR_PWM, 0);
  analogWrite(FR_PWM, 0);
  analogWrite(RL_PWM, 0);
  analogWrite(FL_PWM, 0);
}

void dur(){
  // Normal stop - set all motors to coast (no power)
  digitalWrite(RR_IN1, LOW);
  digitalWrite(RR_IN2, LOW);
  
  digitalWrite(FR_IN1, LOW);
  digitalWrite(FR_IN2, LOW);
  
  digitalWrite(RL_IN1, LOW);
  digitalWrite(RL_IN2, LOW);
  
  digitalWrite(FL_IN1, LOW);
  digitalWrite(FL_IN2, LOW);
  
  analogWrite(RR_PWM, 0);
  analogWrite(FR_PWM, 0);
  analogWrite(RL_PWM, 0);
  analogWrite(FL_PWM, 0);
}

void eyukari(){
  // Move elevator up - both motors forward
  digitalWrite(EL_IN1, HIGH);
  digitalWrite(EL_IN2, LOW);
  analogWrite(EL_PWM, 255);
  
  digitalWrite(ER_IN1, HIGH);
  digitalWrite(ER_IN2, LOW);
  analogWrite(ER_PWM, 255);
}

void easagi() {
  // Move elevator down - both motors reverse
  digitalWrite(EL_IN1, LOW);
  digitalWrite(EL_IN2, HIGH);
  analogWrite(EL_PWM, 255);
  
  digitalWrite(ER_IN1, LOW);
  digitalWrite(ER_IN2, HIGH);
  analogWrite(ER_PWM, 255);
}

// --- MISSING FUNCTIONS IMPLEMENTATION ---

void parseInput(String msg) {
  // Parse incoming UDP messages in format: "AXIS/BUTTON id value"
  // Examples: "AXIS 5 0.123" or "BUTTON 13 1"
  
  // Find spaces to split the message
  int space1 = msg.indexOf(' ');
  int space2 = msg.indexOf(' ', space1 + 1);
  
  // Invalid message format
  if (space1 == -1 || space2 == -1) return;
  
  // Extract type (AXIS/BUTTON), id, and value
  String type = msg.substring(0, space1);
  String idStr = msg.substring(space1 + 1, space2);
  String valStr = msg.substring(space2 + 1);
  
  int id = idStr.toInt();
  
  // Store axis values (float)
  if (type == "AXIS") {
    float val = valStr.toFloat();
    if (id == 0) a0 = val;      // Turn axis
    else if (id == 2) a2 = val; // Strafe axis
    else if (id == 4) a4 = val; // Backward axis
    else if (id == 5) a5 = val; // Forward axis
  } 
  // Store button values (int 0/1)
  else if (type == "BUTTON") {
    int val = valStr.toInt();
    if (id == 0) b0 = val;
    else if (id == 6) b6 = val;   // Servo toggle
    else if (id == 11) b11 = val;
    else if (id == 12) b12 = val; // Elevator down
    else if (id == 13) b13 = val; // Elevator up
  }
}

void processMovement() {
  // Process combined movement from all axes
  // Mixes forward/back, strafe, and turn into individual wheel PWMs
  
  // Calculate base PWMs with deadzones (ignore small values)
  // Positive = forward/right, Negative = backward/left
  int forward_pwm = (a5 > 0.1) ? (int)(a5 * 255) : ((a4 > 0.1) ? -(int)(a4 * 255) : 0);
  int strafe_pwm = (a2 > 0.1) ? (int)(a2 * 255) : ((a2 < -0.1) ? -(int)(abs(a2) * 255) : 0);
  int turn_pwm = (a0 > 0.1) ? (int)(a0 * 255) : ((a0 < -0.1) ? -(int)(abs(a0) * 255) : 0);
  
  // Calculate PWM for each wheel using mecanum/4WD mixing formula
  // FL = Forward - Strafe + Turn
  // FR = Forward + Strafe - Turn  
  // RL = Forward + Strafe + Turn
  // RR = Forward - Strafe - Turn
  int fl_pwm = forward_pwm - strafe_pwm + turn_pwm;
  int fr_pwm = forward_pwm + strafe_pwm - turn_pwm;
  int rl_pwm = forward_pwm + strafe_pwm + turn_pwm;
  int rr_pwm = forward_pwm - strafe_pwm - turn_pwm;
  
  // Clamp PWMs to valid range (-255 to 255)
  fl_pwm = constrain(fl_pwm, -255, 255);
  fr_pwm = constrain(fr_pwm, -255, 255);
  rl_pwm = constrain(rl_pwm, -255, 255);
  rr_pwm = constrain(rr_pwm, -255, 255);
  
  // Apply calculated PWMs to each wheel
  setWheel(FL_IN1, FL_IN2, FL_PWM, fl_pwm);
  setWheel(FR_IN1, FR_IN2, FR_PWM, fr_pwm);
  setWheel(RL_IN1, RL_IN2, RL_PWM, rl_pwm);
  setWheel(RR_IN1, RR_IN2, RR_PWM, rr_pwm);
}

void setWheel(int in1, int in2, int pwm_pin, int pwm) {
  // Set direction and speed for a single wheel
  // pwm > 0: forward, pwm < 0: reverse, pwm = 0: stop
  if (pwm > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwm_pin, pwm);
  } else if (pwm < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwm_pin, abs(pwm));
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwm_pin, 0);
  }
}

void stopAll() {
  // Emergency stop all drive motors with active braking
  anidur();
}

void controlElevator() {
  // Control elevator movement with position tracking
  // Total travel time: 3 seconds (3000ms) up
  
  if (b13 == 1 && elevator_position < 3000) {
    // Button 13 pressed and elevator not at top
    if (!elevator_moving) {
      // Start new movement
      elevator_start_time = millis();
      elevator_moving = true;
    }
    // Move up
    eyukari();
    // Update position based on elapsed time
    unsigned long elapsed = millis() - elevator_start_time;
    elevator_position = min(3000, elevator_position + (int)elapsed);
    elevator_start_time = millis();  // Reset timer
  } else if (b12 == 1 && elevator_position > 0) {
    // Button 12 pressed and elevator not at bottom
    if (!elevator_moving) {
      // Start new movement
      elevator_start_time = millis();
      elevator_moving = true;
    }
    // Move down
    easagi();
    // Update position based on elapsed time
    unsigned long elapsed = millis() - elevator_start_time;
    elevator_position = max(0, elevator_position - (int)elapsed);
    elevator_start_time = millis();  // Reset timer
  } else {
    // No valid button pressed - stop elevator
    analogWrite(EL_PWM, 0);
    analogWrite(ER_PWM, 0);
    elevator_moving = false;
  }
}

void controlServo() {
  // Control elevator lid servo - toggle on button 6 press
  if (b6 == 1 && previous_b6 == 0) {
    // Button 6 just pressed (edge detection)
    e_lid_open = !e_lid_open;  // Toggle state
    e_lid_servo.write(e_lid_open ? 90 : 0);  // Move to position
  }
  previous_b6 = b6;  // Update previous state
}
