#include <Arduino.h>
#include <ESP32Servo.h>

// Runs on CORE 1 only - Pure motor control

// === MOTOR PINS ===
const int RL_PWM=7, RL_IN1=8, RL_IN2=9;
const int RR_PWM=12, RR_IN1=11, RR_IN2=13;
const int FL_PWM=15, FL_IN1=16, FL_IN2=17;
const int FR_PWM=4, FR_IN1=5, FR_IN2=6;
const int EL_PWM=18, EL_IN1=38, EL_IN2=39;
const int ER_PWM=1, ER_IN1=2, ER_IN2=42;

// === SERVO PINS ===
const int E_LID=41, B_LID=14, H_LID=40;

// === UART ===
const int RX_PIN=21, TX_PIN=10;

// === CONTROL STATE ===
float axis0=0, axis2=0, axis4=0, axis5=0;  // Manual axes
int b6=0, b12=0, b13=0;                     // Buttons
bool servoState=false;
bool elevatorTop=false, elevatorBottom=true;
long elevatorPulseTarget=0;

// Elevator limits from master: 133 pulses total travel
const long ELEVATOR_MAX_PULSES=133;
Servo eLidServo;

void setup() {
  Serial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  
  // Motor pins
  pinMode(RL_IN1,OUTPUT); pinMode(RL_IN2,OUTPUT); pinMode(RL_PWM,OUTPUT);
  pinMode(RR_IN1,OUTPUT); pinMode(RR_IN2,OUTPUT); pinMode(RR_PWM,OUTPUT);
  pinMode(FL_IN1,OUTPUT); pinMode(FL_IN2,OUTPUT); pinMode(FL_PWM,OUTPUT);
  pinMode(FR_IN1,OUTPUT); pinMode(FR_IN2,OUTPUT); pinMode(FR_PWM,OUTPUT);
  pinMode(EL_IN1,OUTPUT); pinMode(EL_IN2,OUTPUT); pinMode(EL_PWM,OUTPUT);
  pinMode(ER_IN1,OUTPUT); pinMode(ER_IN2,OUTPUT); pinMode(ER_PWM,OUTPUT);
  
  // Servos
  pinMode(E_LID,OUTPUT); eLidServo.attach(E_LID);
  
  Serial.println("N8R2 Slave Started - Motor Control Ready");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.startsWith("AXIS ")) parseAxis(cmd);
    else if (cmd.startsWith("BUTTON ")) parseButton(cmd);
    else if (cmd.startsWith("AUTONOMOUS ")) parseAutonomous(cmd);
  }
  
  // Execute controls
  driveChassis();
  controlElevator();
  updateServo();
  
  delay(10);
}

void parseAxis(String msg) {
  int id = msg.substring(5, msg.indexOf(' ', 5)).toInt();
  float val = msg.substring(msg.lastIndexOf(' ')+1).toFloat();
  
  switch(id) {
    case 0: axis0 = val; break;  // Left-right slide
    case 2: axis2 = val; break;  // Left-right turn
    case 4: axis4 = val; break;  // Forward
    case 5: axis5 = val; break;  // Backward
  }
}

void parseButton(String msg) {
  int id = msg.substring(7, msg.indexOf(' ', 7)).toInt();
  int val = msg.substring(msg.lastIndexOf(' ')+1).toInt();
  
  switch(id) {
    case 6: b6 = val; break;     // Elevator door
    case 12: b12 = val; break;   // Elevator down
    case 13: b13 = val; break;   // Elevator up
  }
}

void parseAutonomous(String msg) {
  // Parse "AUTONOMOUS: TURN0.50 FWD0.30"
  int turnIdx = msg.indexOf("TURN");
  int fwdIdx = msg.indexOf("FWD");
  axis2 = msg.substring(turnIdx+4, fwdIdx-1).toFloat();
  axis4 = msg.substring(fwdIdx+3).toFloat();
}

void driveChassis() {
  // NO MIXING - Pure individual controls
  // Axis 4: Forward, Axis 5: Backward, Axis 0: Slide, Axis 2: Turn
  
  int fwd = constrain(abs(axis4)*255, 0, 255);
  int bwd = constrain(abs(axis5)*255, 0, 255);
  int slide = constrain(abs(axis0)*255, 0, 255);
  int turn = constrain(abs(axis2)*255, 0, 255);
  
  // Forward/Backward - All wheels same direction
  if (fwd > 20) {
    setAllWheels(true, fwd);  // Forward
  } else if (bwd > 20) {
    setAllWheels(false, bwd); // Backward
  }
  // Left/Right Slide - Opposite sides
  else if (slide > 20) {
    if (axis0 > 0) {  // Right slide
      setWheelsRightSlide(slide);
    } else {          // Left slide
      setWheelsLeftSlide(slide);
    }
  }
  // Left/Right Turn - Tank turn
  else if (turn > 20) {
    if (axis2 > 0) {  // Right turn
      setWheelsRightTurn(turn);
    } else {          // Left turn
      setWheelsLeftTurn(turn);
    }
  } else {
    stopAllWheels();  // IMMEDIATE STOP
  }
}

void setAllWheels(bool forward, int pwm) {
  setWheel(FL_IN1,FL_IN2,FL_PWM, forward ? pwm : -pwm);
  setWheel(FR_IN1,FR_IN2,FR_PWM, forward ? pwm : -pwm);
  setWheel(RL_IN1,RL_IN2,RL_PWM, forward ? pwm : -pwm);
  setWheel(RR_IN1,RR_IN2,RR_PWM, forward ? pwm : -pwm);
}

void setWheelsRightSlide(int pwm) {
  setWheel(FL_IN1,FL_IN2,FL_PWM, -pwm);
  setWheel(FR_IN1,FR_IN2,FR_PWM, pwm);
  setWheel(RL_IN1,RL_IN2,RL_PWM, pwm);
  setWheel(RR_IN1,RR_IN2,RR_PWM, -pwm);
}

void setWheelsLeftSlide(int pwm) {
  setWheel(FL_IN1,FL_IN2,FL_PWM, pwm);
  setWheel(FR_IN1,FR_IN2,FR_PWM, -pwm);
  setWheel(RL_IN1,RL_IN2,RL_PWM, -pwm);
  setWheel(RR_IN1,RR_IN2,RR_PWM, pwm);
}

void setWheelsRightTurn(int pwm) {
  setWheel(FL_IN1,FL_IN2,FL_PWM, pwm);
  setWheel(FR_IN1,FR_IN2,FR_PWM, -pwm);
  setWheel(RL_IN1,RL_IN2,RL_PWM, pwm);
  setWheel(RR_IN1,RR_IN2,RR_PWM, -pwm);
}

void setWheelsLeftTurn(int pwm) {
  setWheel(FL_IN1,FL_IN2,FL_PWM, -pwm);
  setWheel(FR_IN1,FR_IN2,FR_PWM, pwm);
  setWheel(RL_IN1,RL_IN2,RL_PWM, -pwm);
  setWheel(RR_IN1,RR_IN2,RR_PWM, pwm);
}

void stopAllWheels() {
  setWheel(FL_IN1,FL_IN2,FL_PWM, 0);
  setWheel(FR_IN1,FR_IN2,FR_PWM, 0);
  setWheel(RL_IN1,RL_IN2,RL_PWM, 0);
  setWheel(RR_IN1,RR_IN2,RR_PWM, 0);
}

void setWheel(int in1, int in2, int pwmPin, int pwm) {
  if (pwm > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW); analogWrite(pwmPin, pwm);
  } else if (pwm < 0) {
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH); analogWrite(pwmPin, abs(pwm));
  } else {
    digitalWrite(in1, LOW); digitalWrite(in2, LOW); analogWrite(pwmPin, 0);
  }
}

void controlElevator() {
  // Read encoder pulses from master via UART or calculate locally
  // For now using button logic with limits
  
  static unsigned long lastElevCheck = 0;
  if (millis() - lastElevCheck > 50) {
    
    if (b13 == 1 && !elevatorTop) {  // Up but not at top
      elevatorUp();
    } else if (b12 == 1 && !elevatorBottom) {  // Down but not at bottom
      elevatorDown();
    } else {
      elevatorStop();
    }
    
    // Update limits (simplified - master sends actual pulses)
    if (/*EL_count >= ELEVATOR_MAX_PULSES && ER_count >= ELEVATOR_MAX_PULSES*/) {
      elevatorTop = true; elevatorBottom = false;
    } else if (/*EL_count <= ELEVATOR_MIN_PULSES && ER_count <= ELEVATOR_MIN_PULSES*/) {
      elevatorTop = false; elevatorBottom = true;
    }
    
    lastElevCheck = millis();
  }
}

void elevatorUp() {
  digitalWrite(EL_IN1, HIGH); digitalWrite(EL_IN2, LOW); analogWrite(EL_PWM, 255);
  digitalWrite(ER_IN1, LOW); digitalWrite(ER_IN2, HIGH); analogWrite(ER_PWM, 255);  // MIRRORED
}

void elevatorDown() {
  digitalWrite(EL_IN1, LOW); digitalWrite(EL_IN2, HIGH); analogWrite(EL_PWM, 255);
  digitalWrite(ER_IN1, HIGH); digitalWrite(ER_IN2, LOW); analogWrite(ER_PWM, 255);  // MIRRORED
}

void elevatorStop() {
  analogWrite(EL_PWM, 0);
  analogWrite(ER_PWM, 0);
}

void updateServo() {
  static int prevB6 = 0;
  if (b6 == 1 && prevB6 == 0) {
    servoState = !servoState;
    eLidServo.write(servoState ? 90 : 0);
  }
  prevB6 = b6;
}