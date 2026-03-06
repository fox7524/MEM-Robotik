/**
 * ============================================================================
 * ROBOT SLAVE CODE (ESP32-S3 N8R2)
 * ============================================================================
 * RESPONSIBILITIES:
 * 1. Receive Motor Commands via UART
 * 2. Drive 4x Mecanum Wheels
 * 3. Drive 2x Elevator Motors (Mirrored)
 * 4. Control Servo
 * ============================================================================
 */

#include <Arduino.h>
#include <ESP32Servo.h>

// ================= PIN DEFINITIONS =================
// Rear Left
const int RL_PWM = 7; const int RL_IN1 = 8; const int RL_IN2 = 9;
// Rear Right
const int RR_PWM = 12; const int RR_IN1 = 11; const int RR_IN2 = 13;
// Front Left
const int FL_PWM = 15; const int FL_IN1 = 16; const int FL_IN2 = 17;
// Front Right
const int FR_PWM = 4; const int FR_IN1 = 5; const int FR_IN2 = 6;

// Elevator (Left & Right)
const int EL_PWM = 18; const int EL_IN1 = 38; const int EL_IN2 = 39;
const int ER_PWM = 1;  const int ER_IN1 = 2;  const int ER_IN2 = 42;

// Servo
const int E_LID = 41;

// UART
const int RX_PIN = 21; // Listen to Master TX
const int TX_PIN = 10;

// ================= OBJECTS =================
Servo lidServo;

void setup() {
  Serial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  // Motor Pins
  pinMode(RL_PWM, OUTPUT); pinMode(RL_IN1, OUTPUT); pinMode(RL_IN2, OUTPUT);
  pinMode(RR_PWM, OUTPUT); pinMode(RR_IN1, OUTPUT); pinMode(RR_IN2, OUTPUT);
  pinMode(FL_PWM, OUTPUT); pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT);
  pinMode(FR_PWM, OUTPUT); pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT);

  pinMode(EL_PWM, OUTPUT); pinMode(EL_IN1, OUTPUT); pinMode(EL_IN2, OUTPUT);
  pinMode(ER_PWM, OUTPUT); pinMode(ER_IN1, OUTPUT); pinMode(ER_IN2, OUTPUT);

  // Servo
  lidServo.attach(E_LID);
  lidServo.write(0); // Initial closed

  Serial.println("SLAVE N8R2 READY");
}

// Function to set motor driver
void setMotor(int pwmPin, int in1, int in2, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
    analogWrite(pwmPin, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
    analogWrite(pwmPin, abs(speed));
  } else {
    // BRAKE
    digitalWrite(in1, HIGH); digitalWrite(in2, HIGH);
    analogWrite(pwmPin, 255);
  }
}

// Data Parsing Buffer
String inputString = "";
bool stringComplete = false;

void loop() {
  // 1. Read UART
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '<') {
      inputString = ""; // Start packet
    } else if (inChar == '>') {
      stringComplete = true; // End packet
    } else {
      inputString += inChar;
    }
  }

  // 2. Parse & Execute
  if (stringComplete) {
    // Format: fl,fr,rl,rr,el,sv
    // Example: 100,100,100,100,1,0
    
    int firstComma = inputString.indexOf(',');
    int secondComma = inputString.indexOf(',', firstComma + 1);
    int thirdComma = inputString.indexOf(',', secondComma + 1);
    int fourthComma = inputString.indexOf(',', thirdComma + 1);
    int fifthComma = inputString.indexOf(',', fourthComma + 1);

    if (fifthComma > 0) {
      int fl = inputString.substring(0, firstComma).toInt();
      int fr = inputString.substring(firstComma + 1, secondComma).toInt();
      int rl = inputString.substring(secondComma + 1, thirdComma).toInt();
      int rr = inputString.substring(thirdComma + 1, fourthComma).toInt();
      int el_cmd = inputString.substring(fourthComma + 1, fifthComma).toInt();
      int sv_cmd = inputString.substring(fifthComma + 1).toInt();

      // Drive Wheels
      setMotor(FL_PWM, FL_IN1, FL_IN2, fl);
      setMotor(FR_PWM, FR_IN1, FR_IN2, fr);
      setMotor(RL_PWM, RL_IN1, RL_IN2, rl);
      setMotor(RR_PWM, RR_IN1, RR_IN2, rr);

      // Drive Elevator (Mirrored)
      // el_cmd: 0=Stop, 1=Up, 2=Down
      if (el_cmd == 1) { // UP
        // Left CW, Right CCW (Mirrored)
        setMotor(EL_PWM, EL_IN1, EL_IN2, 255); 
        setMotor(ER_PWM, ER_IN1, ER_IN2, -255);
      } else if (el_cmd == 2) { // DOWN
        // Left CCW, Right CW
        setMotor(EL_PWM, EL_IN1, EL_IN2, -255);
        setMotor(ER_PWM, ER_IN1, ER_IN2, 255);
      } else {
        setMotor(EL_PWM, EL_IN1, EL_IN2, 0);
        setMotor(ER_PWM, ER_IN1, ER_IN2, 0);
      }

      // Servo
      if (sv_cmd == 1) lidServo.write(90); // Open
      else lidServo.write(0); // Close
    }

    inputString = "";
    stringComplete = false;
  }
}