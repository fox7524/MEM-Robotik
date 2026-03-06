/*
 * ESP32-S3-N8R2 SLAVE MOTOR CONTROLLER
 * 
 * This board ONLY controls:
 * - 4 main chassis motors (mecanum wheels)
 * - 2 elevator motors (mirrored)
 * - 3 servos (elevator door)
 * 
 * Receives movement commands from Master (N16R8) via UART
 * Implements non-mixing control: only ONE primary movement at a time
 * Elevator and servo work independently
 * 
 * CORE 1: Motor control loop (simple execution)
 * No sensor reading on this board - only actuators
 */

#include <Arduino.h>
#include <ESP32Servo.h>

// ============================================================================
// CONSTANTS
// ============================================================================

// Elevator pulse threshold calculation:
// 20 pulses/rotation × (200 RPM / 60 sec) × 2 sec = ~133 pulses total
const int ELEVATOR_PULSE_LIMIT = 133;

// Dead zone for joystick input (ignore values smaller than this)
const float DEADZONE = 0.1;

// ============================================================================
// PIN DEFINITIONS - MAIN CHASSIS MOTORS
// ============================================================================

// Rear Left (RL) Motor
const int RL_PWM = 7;    // PWM speed control
const int RL_IN1 = 8;    // Direction bit 1
const int RL_IN2 = 9;    // Direction bit 2

// Rear Right (RR) Motor
const int RR_PWM = 12;   // PWM speed control
const int RR_IN1 = 11;   // Direction bit 1
const int RR_IN2 = 13;   // Direction bit 2

// Front Left (FL) Motor
const int FL_PWM = 15;   // PWM speed control
const int FL_IN1 = 16;   // Direction bit 1
const int FL_IN2 = 17;   // Direction bit 2

// Front Right (FR) Motor
const int FR_PWM = 4;    // PWM speed control
const int FR_IN1 = 5;    // Direction bit 1
const int FR_IN2 = 6;    // Direction bit 2

// ============================================================================
// PIN DEFINITIONS - ELEVATOR MOTORS
// ============================================================================

// Elevator Left Motor (EL)
const int EL_PWM = 18;   // PWM speed control
const int EL_IN1 = 38;   // Direction bit 1
const int EL_IN2 = 39;   // Direction bit 2

// Elevator Right Motor (ER) - Mirrored from EL
const int ER_PWM = 1;    // PWM speed control
const int ER_IN1 = 2;    // Direction bit 1
const int ER_IN2 = 42;   // Direction bit 2

// ============================================================================
// PIN DEFINITIONS - ELEVATOR ENCODERS (for position tracking)
// ============================================================================

#define EL_ENCODER_A 10  // Elevator left encoder phase A
#define EL_ENCODER_B 11  // Elevator left encoder phase B
#define ER_ENCODER_A 12  // Elevator right encoder phase A
#define ER_ENCODER_B 13  // Elevator right encoder phase B

// ============================================================================
// PIN DEFINITIONS - SERVOS
// ============================================================================

const int SERVO_ELEVATOR_DOOR = 41;   // Servo for elevator door/lid
const int SERVO_UNUSED_1 = 14;        // Unused servo pin
const int SERVO_UNUSED_2 = 40;        // Unused servo pin

// ============================================================================
// PIN DEFINITIONS - UART FROM MASTER
// ============================================================================

const int UART_RX = 21;   // RX pin (from Master)
const int UART_TX = 10;   // TX pin (to Master, optional)

// ============================================================================
// VOLATILE VARIABLES - ENCODER COUNTERS (for ISRs)
// ============================================================================

// Elevator encoder pulse counters
// These track total pulses from 0 (down) to ~133 (up)
volatile long elevator_left_pulses = 0;
volatile long elevator_right_pulses = 0;

// ============================================================================
// STATE VARIABLES - CURRENT COMMANDS
// ============================================================================

// Latest movement commands from Master (normalized -1 to 1)
float cmd_forward = 0;    // Forward/backward (combine axis 4 and 5)
float cmd_strafe = 0;     // Left/right slide (axis 2)
float cmd_turn = 0;       // Rotation (axis 0)

// Servo and elevator commands
float cmd_servo_door = 0; // Servo door position (0-1)
float cmd_elevator_up = 0;    // Elevator up button (0 or 1)
float cmd_elevator_down = 0;  // Elevator down button (0 or 1)

// ============================================================================
// STATE VARIABLES - PREVIOUS COMMANDS (for edge detection)
// ============================================================================

float prev_forward = 0;
float prev_strafe = 0;
float prev_turn = 0;

// ============================================================================
// STATE VARIABLES - ELEVATOR
// ============================================================================

// Elevator position state
// 0 = fully down, 133 = fully up
int elevator_position = 0;
bool elevator_at_top = false;     // Flag: is elevator at max height?
bool elevator_at_bottom = true;   // Flag: is elevator at min height?

// Servo state
Servo servo_door;
bool servo_door_open = false;
int prev_servo_button = 0;

// ============================================================================
// INTERRUPT SERVICE ROUTINES - ELEVATOR ENCODERS
// ============================================================================

/*
 * ISR for Elevator Left encoder
 * Counts pulses as elevator moves up/down
 */
void IRAM_ATTR isr_elevator_left() {
  if (digitalRead(EL_ENCODER_A) == digitalRead(EL_ENCODER_B)) {
    elevator_left_pulses++;  // Up movement
  } else {
    elevator_left_pulses--;  // Down movement
  }
  
  // Clamp to valid range [0, ELEVATOR_PULSE_LIMIT]
  elevator_left_pulses = constrain(elevator_left_pulses, 0, ELEVATOR_PULSE_LIMIT);
  
  // Update position and limit flags
  elevator_position = elevator_left_pulses;
  elevator_at_top = (elevator_left_pulses >= ELEVATOR_PULSE_LIMIT);
  elevator_at_bottom = (elevator_left_pulses <= 0);
}

/*
 * ISR for Elevator Right encoder
 * Should mirror the left encoder (same pulses, opposite direction)
 */
void IRAM_ATTR isr_elevator_right() {
  if (digitalRead(ER_ENCODER_A) == digitalRead(ER_ENCODER_B)) {
    elevator_right_pulses++;  // Up movement
  } else {
    elevator_right_pulses--;  // Down movement
  }
  
  // Clamp to valid range [0, ELEVATOR_PULSE_LIMIT]
  elevator_right_pulses = constrain(elevator_right_pulses, 0, ELEVATOR_PULSE_LIMIT);
}

// ============================================================================
// MOTOR CONTROL HELPER FUNCTIONS
// ============================================================================

/*
 * Set a single motor speed and direction
 * 
 * Parameters:
 *   in1, in2: Direction control pins (H/L combinations determine direction)
 *   pwm_pin:  PWM pin for speed (0-255)
 *   power:    -1.0 to 1.0, where:
 *             - Negative: reverse
 *             - 0: stop (coast)
 *             - Positive: forward
 * 
 * Motor direction truth table:
 *   IN1  IN2  Direction
 *   H    L    Forward
 *   L    H    Reverse
 *   L    L    Stop (coast)
 *   H    H    Stop (brake) - we won't use this
 */
void setMotorSpeed(int in1, int in2, int pwm_pin, float power) {
  // Apply deadzone
  if (abs(power) < 0.01) {
    power = 0;
  }
  
  // Clamp to valid range
  power = constrain(power, -1.0, 1.0);
  
  // Convert normalized value to PWM (0-255)
  int pwm_value = abs(power) * 255;
  
  if (power > 0) {
    // Forward: IN1=HIGH, IN2=LOW
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwm_pin, pwm_value);
  } 
  else if (power < 0) {
    // Reverse: IN1=LOW, IN2=HIGH
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwm_pin, pwm_value);
  } 
  else {
    // Stop (coast): IN1=LOW, IN2=LOW, PWM=0
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwm_pin, 0);
  }
}

/*
 * Apply motor command to all 4 main chassis motors
 * using mecanum wheel mixing formula
 * 
 * Non-Mixing Control Mode:
 * - If forward/backward is non-zero, ONLY apply forward/backward
 * - If strafe is non-zero (and forward is zero), ONLY apply strafe
 * - If turn is non-zero (and above two are zero), ONLY apply turn
 * 
 * This ensures clean, predictable movement with no simultaneous commands
 */
void driveMainChassis() {
  // ===== NON-MIXING LOGIC =====
  // Priority: Forward/Backward > Strafe > Turn
  
  int fl_power = 0;
  int fr_power = 0;
  int rl_power = 0;
  int rr_power = 0;

  // Check forward/backward command (highest priority)
  if (abs(cmd_forward) > DEADZONE) {
    // FORWARD/BACKWARD ONLY
    // All wheels move in same direction
    fl_power = cmd_forward * 255;
    fr_power = cmd_forward * 255;
    rl_power = cmd_forward * 255;
    rr_power = cmd_forward * 255;
  }
  // Check strafe command (if forward is zero)
  else if (abs(cmd_strafe) > DEADZONE) {
    // STRAFE (LEFT/RIGHT SLIDE) ONLY
    // Mecanum configuration: wheels rotate for pure strafe
    // FL and RR go one direction, FR and RL go opposite
    fl_power = cmd_strafe * 255;
    fr_power = -cmd_strafe * 255;
    rl_power = -cmd_strafe * 255;
    rr_power = cmd_strafe * 255;
  }
  // Check turn command (if both above are zero)
  else if (abs(cmd_turn) > DEADZONE) {
    // ROTATION (360° TURN) ONLY
    // Left side wheels opposite to right side wheels
    fl_power = cmd_turn * 255;
    fr_power = -cmd_turn * 255;
    rl_power = cmd_turn * 255;
    rr_power = -cmd_turn * 255;
  }
  // No command: STOP (coast)
  else {
    fl_power = 0;
    fr_power = 0;
    rl_power = 0;
    rr_power = 0;
  }

  // Apply to motors
  setMotorSpeed(FL_IN1, FL_IN2, FL_PWM, fl_power / 255.0);
  setMotorSpeed(FR_IN1, FR_IN2, FR_PWM, fr_power / 255.0);
  setMotorSpeed(RL_IN1, RL_IN2, RL_PWM, rl_power / 255.0);
  setMotorSpeed(RR_IN1, RR_IN2, RR_PWM, rr_power / 255.0);
}

/*
 * Control elevator motors with encoder-based limits
 * 
 * Elevator state machine:
 * 1. If elevator_at_bottom (pulses=0): Only UP is allowed
 * 2. If elevator_at_top (pulses≥133): Only DOWN is allowed
 * 3. Otherwise: Both UP and DOWN allowed
 * 
 * Both elevator motors must move together (mirrored)
 */
void driveElevator() {
  // Read current button states
  bool should_go_up = (cmd_elevator_up > 0.5) && !elevator_at_top;
  bool should_go_down = (cmd_elevator_down > 0.5) && !elevator_at_bottom;

  // Only ONE direction at a time (safety)
  if (should_go_up && should_go_down) {
    should_go_down = false;  // Prioritize UP
  }

  if (should_go_up) {
    // Move UP: Both motors forward (mirrored)
    // EL: IN1=HIGH, IN2=LOW -> UP
    // ER: IN1=HIGH, IN2=LOW -> UP (mirrored means same direction relative to motor)
    digitalWrite(EL_IN1, HIGH);
    digitalWrite(EL_IN2, LOW);
    analogWrite(EL_PWM, 255);

    digitalWrite(ER_IN1, HIGH);
    digitalWrite(ER_IN2, LOW);
    analogWrite(ER_PWM, 255);
  } 
  else if (should_go_down) {
    // Move DOWN: Both motors reverse
    // EL: IN1=LOW, IN2=HIGH -> DOWN
    // ER: IN1=LOW, IN2=HIGH -> DOWN (mirrored)
    digitalWrite(EL_IN1, LOW);
    digitalWrite(EL_IN2, HIGH);
    analogWrite(EL_PWM, 255);

    digitalWrite(ER_IN1, LOW);
    digitalWrite(ER_IN2, HIGH);
    analogWrite(ER_PWM, 255);
  } 
  else {
    // STOP: Coast all motors
    digitalWrite(EL_IN1, LOW);
    digitalWrite(EL_IN2, LOW);
    analogWrite(EL_PWM, 0);

    digitalWrite(ER_IN1, LOW);
    digitalWrite(ER_IN2, LOW);
    analogWrite(ER_PWM, 0);
  }
}

/*
 * Control servo door (toggle on button press)
 * 
 * Simple edge detection:
 * When button transitions from 0→1, toggle servo position
 */
void driveServo() {
  // Edge detection on servo button (cmd_servo_door)
  if ((cmd_servo_door > 0.5) && (prev_servo_button == 0)) {
    // Button just pressed (edge 0→1)
    servo_door_open = !servo_door_open;
    
    if (servo_door_open) {
      servo_door.write(90);   // Open position
      Serial.println("[SERVO] Door opened");
    } else {
      servo_door.write(0);    // Closed position
      Serial.println("[SERVO] Door closed");
    }
  }
  
  prev_servo_button = (cmd_servo_door > 0.5) ? 1 : 0;
}

// ============================================================================
// UART PARSING FUNCTION
// ============================================================================

/*
 * Parse motor command from Master via UART
 * 
 * Command format: "MOV f,s,t,d,u,d,spare\n"
 * Where:
 *   f = forward (-1 to 1)
 *   s = strafe (-1 to 1)
 *   t = turn (-1 to 1)
 *   d = door (0-1, only edge triggers action)
 *   u = elevator_up (0-1)
 *   d = elevator_down (0-1)
 *   spare = unused (7th field)
 * 
 * Example: "MOV 0.50,0.00,-0.30,0,0,0,0\n"
 */
void parseUARTCommand(String msg) {
  msg.trim();
  
  // Check command type
  if (!msg.startsWith("MOV ")) {
    return;  // Invalid command
  }
  
  // Remove "MOV " prefix
  msg = msg.substring(4);
  
  // Split by comma
  int index = 0;
  float values[7] = {0, 0, 0, 0, 0, 0, 0};
  
  int lastComma = -1;
  for (int i = 0; i < msg.length(); i++) {
    if (msg[i] == ',' || i == msg.length() - 1) {
      int endPos = (i == msg.length() - 1) ? (msg.length()) : i;
      String valStr = msg.substring(lastComma + 1, endPos);
      
      if (index < 7) {
        values[index] = valStr.toFloat();
        index++;
      }
      
      lastComma = i;
    }
  }
  
  // Assign parsed values to command variables
  cmd_forward = values[0];
  cmd_strafe = values[1];
  cmd_turn = values[2];
  cmd_servo_door = values[3];
  cmd_elevator_up = values[4];
  cmd_elevator_down = values[5];
  
  // Debug output
  Serial.printf("[UART] CMD: F:%.2f S:%.2f T:%.2f Door:%d EU:%d ED:%d | Elevator:%d/%d At_Top:%d\n",
                cmd_forward, cmd_strafe, cmd_turn,
                (int)cmd_servo_door, (int)cmd_elevator_up, (int)cmd_elevator_down,
                elevator_position, ELEVATOR_PULSE_LIMIT, elevator_at_top);
}

// ============================================================================
// SETUP FUNCTION
// ============================================================================

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\n=== ESP32-S3-N8R2 SLAVE STARTING ===\n");

  // ========== PIN SETUP - MOTOR PINS ==========
  
  // Main chassis motor pins
  pinMode(FL_IN1, OUTPUT);
  pinMode(FL_IN2, OUTPUT);
  pinMode(FL_PWM, OUTPUT);

  pinMode(FR_IN1, OUTPUT);
  pinMode(FR_IN2, OUTPUT);
  pinMode(FR_PWM, OUTPUT);

  pinMode(RL_IN1, OUTPUT);
  pinMode(RL_IN2, OUTPUT);
  pinMode(RL_PWM, OUTPUT);

  pinMode(RR_IN1, OUTPUT);
  pinMode(RR_IN2, OUTPUT);
  pinMode(RR_PWM, OUTPUT);

  // Elevator motor pins
  pinMode(EL_IN1, OUTPUT);
  pinMode(EL_IN2, OUTPUT);
  pinMode(EL_PWM, OUTPUT);

  pinMode(ER_IN1, OUTPUT);
  pinMode(ER_IN2, OUTPUT);
  pinMode(ER_PWM, OUTPUT);

  Serial.println("[INIT] Motor pins configured");

  // ========== PIN SETUP - ENCODER PINS ==========
  
  pinMode(EL_ENCODER_A, INPUT);
  pinMode(EL_ENCODER_B, INPUT);
  pinMode(ER_ENCODER_A, INPUT);
  pinMode(ER_ENCODER_B, INPUT);

  // Attach elevator encoder ISRs
  attachInterrupt(digitalPinToInterrupt(EL_ENCODER_A), isr_elevator_left, RISING);
  attachInterrupt(digitalPinToInterrupt(ER_ENCODER_A), isr_elevator_right, RISING);

  Serial.println("[INIT] Elevator encoders configured");

  // ========== SERVO SETUP ==========
  
  servo_door.attach(SERVO_ELEVATOR_DOOR);
  servo_door.write(0);  // Closed position
  
  Serial.println("[INIT] Servo initialized");

  // ========== UART SETUP ==========
  
  Serial.begin(115200, SERIAL_8N1, UART_RX, UART_TX);
  delay(100);
  
  Serial.println("[INIT] UART from Master configured");
  Serial.printf("[INFO] Listening on RX=%d, TX=%d at 115200 baud\n", UART_RX, UART_TX);

  // ========== INITIAL STATE ==========
  
  Serial.println("\n[READY] Waiting for commands from Master...");
  Serial.printf("[INFO] Elevator pulse limit: %d\n", ELEVATOR_PULSE_LIMIT);
  Serial.println("[INFO] Non-mixing control enabled (one movement type at a time)\n");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // ========== READ UART COMMANDS FROM MASTER ==========
  
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    msg.trim();
    
    if (msg.length() > 0) {
      parseUARTCommand(msg);
    }
  }

  // ========== EXECUTE MOTOR COMMANDS ==========
  
  // Drive main chassis (with non-mixing logic)
  driveMainChassis();
  
  // Drive elevator (with encoder-based limits)
  driveElevator();
  
  // Drive servo (with edge detection)
  driveServo();

  // Small delay to prevent CPU hogging
  delay(3);
}

// ============================================================================

void sagonileri() {
    setMotorSpeed(FL_IN1, FL_IN2, FL_PWM, 1.0);   // Front Left stop
    setMotorSpeed(FR_IN1, FR_IN2, FR_PWM, 0.0);   // Front Right Stop
    setMotorSpeed(RL_IN1, RL_IN2, RL_PWM, 0.0);   // Rear Left Stop
    setMotorSpeed(RR_IN1, RR_IN2, RR_PWM, 1.0);   // Rear Right Forward
    }

// ============================================================================
// END OF CODE
// ============================================================================
