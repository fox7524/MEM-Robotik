
// 192.168.4.1

// MECHATAK - SIMPLE MOTOR TEST (NO ProBot complications)
// MECHATAK - SIMPLE MOTOR TEST (NO ProBot complications)

#define PROBOT_WIFI_AP_PASSWORD "kayra123"
#define PROBOT_WIFI_AP_SSID "MECHATAK"
#define PROBOT_WIFI_AP_CHANNEL 3
#include <probot.h>

// http://192.168.4.1
#include <probot/devices/servo/servo.hpp>


// --- Servo pins (choose actual GPIOs for your hardware) ---
const int SERVO_BOTTOM_PIN = 21;  // example pin for bottom lid
const int SERVO_TOP_PIN    = 22;  // example pin for top lid

// --- Servo objects ---
probot::devices::Servo bottomLidServo(SERVO_BOTTOM_PIN);
probot::devices::Servo topLidServo(SERVO_TOP_PIN);

// --- Servo states ---
bool bottomLidOpen = false;
bool topLidOpen = false;

// --- Track button press edges ---
bool lastBtn0State = false;
bool lastBtn3State = false;

void robotInit() {
  Serial.begin(115200);
  delay(100);

  // Initialize all motors
  drvFL.begin(); drvFR.begin(); drvRL.begin(); drvRR.begin();
  drvEL.begin();

  drvFL.setBrakeMode(true); drvFR.setBrakeMode(true);
  drvRL.setBrakeMode(true); drvRR.setBrakeMode(true);
  drvEL.setBrakeMode(true);

  // Initialize servos
  bottomLidServo.begin();
  topLidServo.begin();
  bottomLidServo.write(0);   // start closed
  topLidServo.write(0);      // start closed

  mecanum.setInverted(false, true, false, true);
  mecanum.setWheelBase(30.0f);
  mecanum.setTrackWidth(28.0f);

  Scheduler::instance().registerSubsystem(&mecanum);
  Serial.println("[MecanumDriveDemo] robotInit: Mecanum + Elevator + Servos hazir");
}

void teleopLoop() {
  auto js = probot::io::joystick_api::makeDefault();

  // --- Mecanum drive mapping ---
  float omega = js.getAxis(0);   // Left stick X = rotation
  float vy    = js.getAxis(2);   // Axis 2 = strafe
  float rt    = js.getTriggerRight(); // RT forward
  float lt    = js.getTriggerLeft();  // LT backward
  float vx    = rt - lt;              // net forward/backward

  mecanum.drivePower(vx, vy, omega);

  // --- Elevator control (same as before) ---
  unsigned long currentMillis = millis();
  float elevatorMotorCommand = 0.0f;

  if (js.getRawButton(6)) {
    if (elevatorDoorPulseStart == 0) {
      elevatorDoorPulseStart = currentMillis;
      Serial.println("[ELEVATOR] Door pulse started.");
    }
    elevatorMotorCommand = ELEVATOR_DOOR_PULSE_POWER;
  } else {
    if (elevatorDoorPulseStart != 0 &&
        (currentMillis - elevatorDoorPulseStart) < ELEVATOR_DOOR_PULSE_DURATION_MS) {
      elevatorMotorCommand = ELEVATOR_DOOR_PULSE_POWER;
    } else {
      elevatorDoorPulseStart = 0;
    }
  }

  if (js.getRawButton(13) && elevatorDoorPulseStart == 0) {
    if (elevatorUpRemainingBudget > 0) {
      elevatorMotorCommand = ELEVATOR_SPEED;
      elevatorUpRemainingBudget -= 20;
    } else {
      Serial.println("[ELEVATOR] Up budget exhausted.");
    }
  } else if (js.getRawButton(14) && elevatorDoorPulseStart == 0) {
    if (elevatorDownRemainingBudget > 0) {
      elevatorMotorCommand = -ELEVATOR_SPEED;
      elevatorDownRemainingBudget -= 20;
    } else {
      Serial.println("[ELEVATOR] Down budget exhausted.");
    }
  }

  drvEL.setPower(elevatorMotorCommand);

  // --- Servo toggle logic ---
  bool btn0 = js.getRawButton(0);
  bool btn3 = js.getRawButton(3);

  // Bottom lid toggle
  if (btn0 && !lastBtn0State) { // detect rising edge
    bottomLidOpen = !bottomLidOpen;
    bottomLidServo.write(bottomLidOpen ? 180 : 0); // open fully or close
    Serial.printf("[SERVO] Bottom lid %s\n", bottomLidOpen ? "OPEN" : "CLOSED");
  }
  lastBtn0State = btn0;

  // Top lid toggle
  if (btn3 && !lastBtn3State) { // detect rising edge
    topLidOpen = !topLidOpen;
    topLidServo.write(topLidOpen ? 180 : 0);
    Serial.printf("[SERVO] Top lid %s\n", topLidOpen ? "OPEN" : "CLOSED");
  }
  lastBtn3State = btn3;

  // --- Telemetry ---
  Serial.printf("[Mecanum] vx=%.2f vy=%.2f w=%.2f | "
                "[Elevator] pwr=%.2f | UpB:%.0f DownB:%.0f | "
                "[Servos] Bottom:%s Top:%s\n",
                vx, vy, omega, drvEL.getPower(),
                elevatorUpRemainingBudget, elevatorDownRemainingBudget,
                bottomLidOpen ? "OPEN" : "CLOSED",
                topLidOpen ? "OPEN" : "CLOSED");

  delay(20);
=======

#include <probot.h>

// YOUR PINS
const int FL_IN1 = 16, FL_IN2 = 17, FL_ENA = 15;
const int FR_IN1 = 5,  FR_IN2 = 6,  FR_ENA = 4;  
const int RL_IN1 = 8,  RL_IN2 = 9,  RL_ENA = 7;
const int RR_IN1 = 13, RR_IN2 = 11, RR_ENA = 12;
const int EL_IN1 = 38, EL_IN2 = 39, EL_ENA = 18;

void robotInit() {
  Serial.begin(115200);
  delay(500);
  
  // Setup all pins
  pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT); pinMode(FL_ENA, OUTPUT);
  pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT); pinMode(FR_ENA, OUTPUT);
  pinMode(RL_IN1, OUTPUT); pinMode(RL_IN2, OUTPUT); pinMode(RL_ENA, OUTPUT);
  pinMode(RR_IN1, OUTPUT); pinMode(RR_IN2, OUTPUT); pinMode(RR_ENA, OUTPUT);
  pinMode(EL_IN1, OUTPUT); pinMode(EL_IN2, OUTPUT); pinMode(EL_ENA, OUTPUT);
  
  Serial.println("\n🚀 MECHATAK SIMPLE MOTOR TEST");
  Serial.println("📡 WiFi: MECHATAK (kayra123)");
  Serial.println("✅ ALL PINS READY - 3s per test");
}

void motorForward(int in1, int in2, int ena, float speed = 0.6) {
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  analogWrite(ena, speed * 255);
}

void motorBackward(int in1, int in2, int ena, float speed = 0.6) {
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  analogWrite(ena, speed * 255);
}

void motorStop(int in1, int in2, int ena) {
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  analogWrite(ena, 0);
}

void teleopInit() {
  Serial.println("❌ TELEOP DISABLED");
}

void teleopLoop() {
  // Empty - no joystick
>>>>>>> Stashed changes
}
=======

#include <probot.h>

// YOUR PINS
const int FL_IN1 = 16, FL_IN2 = 17, FL_ENA = 15;
const int FR_IN1 = 5,  FR_IN2 = 6,  FR_ENA = 4;  
const int RL_IN1 = 8,  RL_IN2 = 9,  RL_ENA = 7;
const int RR_IN1 = 13, RR_IN2 = 11, RR_ENA = 12;
const int EL_IN1 = 38, EL_IN2 = 39, EL_ENA = 18;

void robotInit() {
  Serial.begin(115200);
  delay(500);
  
  // Setup all pins
  pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT); pinMode(FL_ENA, OUTPUT);
  pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT); pinMode(FR_ENA, OUTPUT);
  pinMode(RL_IN1, OUTPUT); pinMode(RL_IN2, OUTPUT); pinMode(RL_ENA, OUTPUT);
  pinMode(RR_IN1, OUTPUT); pinMode(RR_IN2, OUTPUT); pinMode(RR_ENA, OUTPUT);
  pinMode(EL_IN1, OUTPUT); pinMode(EL_IN2, OUTPUT); pinMode(EL_ENA, OUTPUT);
  
  Serial.println("\n🚀 MECHATAK SIMPLE MOTOR TEST");
  Serial.println("📡 WiFi: MECHATAK (kayra123)");
  Serial.println("✅ ALL PINS READY - 3s per test");
}

void motorForward(int in1, int in2, int ena, float speed = 0.6) {
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  analogWrite(ena, speed * 255);
}

void motorBackward(int in1, int in2, int ena, float speed = 0.6) {
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  analogWrite(ena, speed * 255);
}

void motorStop(int in1, int in2, int ena) {
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  analogWrite(ena, 0);
}

void teleopInit() {
  Serial.println("❌ TELEOP DISABLED");
}

void teleopLoop() {
  // Empty - no joystick
}

void autonomousInit() {
  Serial.println("🧪 SIMPLE MOTOR TEST START");
}

void autonomousLoop() {
  static unsigned long start = millis();
  static int phase = 0;
  
  if (millis() - start > 3000) {
    phase = (phase + 1) % 11;
    start = millis();
    Serial.printf("PHASE %d/10\n", phase);
  }
  
  // STOP ALL FIRST
  motorStop(FL_IN1, FL_IN2, FL_ENA);
  motorStop(FR_IN1, FR_IN2, FR_ENA);
  motorStop(RL_IN1, RL_IN2, RL_ENA);
  motorStop(RR_IN1, RR_IN2, RR_ENA);
  motorStop(EL_IN1, EL_IN2, EL_ENA);
  
  switch(phase) {
    case 0: Serial.println("🛑 ALL STOP"); break;
    
    case 1: motorForward(FL_IN1, FL_IN2, FL_ENA); Serial.println("🔄 FL FORWARD"); break;
    case 2: motorBackward(FL_IN1, FL_IN2, FL_ENA); Serial.println("🔄 FL BACKWARD"); break;
    
    case 3: motorForward(FR_IN1, FR_IN2, FR_ENA); Serial.println("🔄 FR FORWARD"); break;
    case 4: motorBackward(FR_IN1, FR_IN2, FR_ENA); Serial.println("🔄 FR BACKWARD"); break;
    
    case 5: motorForward(RL_IN1, RL_IN2, RL_ENA); Serial.println("🔄 RL FORWARD"); break;
    case 6: motorBackward(RL_IN1, RL_IN2, RL_ENA); Serial.println("🔄 RL BACKWARD"); break;
    
    case 7: motorForward(RR_IN1, RR_IN2, RR_ENA); Serial.println("🔄 RR FORWARD"); break;
    case 10: motorBackward(RR_IN1, RR_IN2, RR_ENA); Serial.println("🔄 RR BACKWARD"); break;
    
    case 8: motorForward(EL_IN1, EL_IN2, EL_ENA, 0.7); Serial.println("⬆️ ELEVATOR UP"); break;
    case 9: motorBackward(EL_IN1, EL_IN2, EL_ENA, 0.7); Serial.println("⬇️ ELEVATOR DOWN"); break;
  }
  
  delay(50);
}

void robotEnd() {
  motorStop(FL_IN1, FL_IN2, FL_ENA);
  motorStop(FR_IN1, FR_IN2, FR_ENA);
  motorStop(RL_IN1, RL_IN2, RL_ENA);
  motorStop(RR_IN1, RR_IN2, RR_ENA);
  motorStop(EL_IN1, EL_IN2, EL_ENA);
  Serial.println("🏁 EMERGENCY STOP");
}
>>>>>>> Stashed changes
