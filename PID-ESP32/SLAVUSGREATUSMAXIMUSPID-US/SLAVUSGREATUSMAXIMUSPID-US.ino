#include <Arduino.h>
#include <ESP32Servo.h>

// === Motor Pins (from your pinout) ===
struct MotorPins { int pwm, in1, in2, channel; };
MotorPins motors[6] = {
  {7, 8, 9, 0},    // M1 Rear Left
  {12, 11, 13, 1}, // M2 Rear Right
  {15, 16, 17, 2}, // M3 Front Left
  {4, 5, 6, 3},    // M4 Front Right
  {18, 38, 39, 4}, // M5 Elevator Left
  {1, 2, 42, 5}    // M6 Elevator Right (kept as you defined)
};

// === Servos ===
Servo servoElevator, servoBottom, servoHuni;
#define SERVO_ELEVATOR 41
#define SERVO_BOTTOM   14
#define SERVO_HUNI     40

// === Setup ===
void setup() {
  // Use Serial2 on RX=21, TX=10
  Serial2.begin(115200, SERIAL_8N1, 21, 10);

  for(int i=0;i<6;i++){
  pinMode(motors[i].in1,OUTPUT);
  pinMode(motors[i].in2,OUTPUT);
  ledcAttach(motors[i].pwm, 20000, 8); // 20kHz, 8-bit resolution
}

  // Servos
  servoElevator.attach(SERVO_ELEVATOR, 500, 2400);
  servoBottom.attach(SERVO_BOTTOM, 500, 2400);
  servoHuni.attach(SERVO_HUNI, 500, 2400);

  Serial2.println("SLAVE READY");
}

// === Motor drive ===
void driveMotor(MotorPins m, float power){
  power = constrain(power,-1.0,1.0);
  int pwmVal = abs(power)*255;
  if(power>=0){ digitalWrite(m.in1,HIGH); digitalWrite(m.in2,LOW); }
  else { digitalWrite(m.in1,LOW); digitalWrite(m.in2,HIGH); }
  ledcWrite(m.pwm, pwmVal); // use pin instead of channel
}

// === Loop ===
void loop() {
  if(Serial2.available()){
    String cmd=Serial2.readStringUntil('\n');
    cmd.trim();

    if(cmd.startsWith("MOV")){
      cmd.remove(0,4);
      float f=cmd.substring(0,cmd.indexOf(',')).toFloat();
      cmd=cmd.substring(cmd.indexOf(',')+1);
      float s=cmd.substring(0,cmd.indexOf(',')).toFloat();
      cmd=cmd.substring(cmd.indexOf(',')+1);
      float t=cmd.substring(0,cmd.indexOf(',')).toFloat();
      cmd=cmd.substring(cmd.indexOf(',')+1);
      int servo=cmd.substring(0,cmd.indexOf(',')).toInt();
      cmd=cmd.substring(cmd.indexOf(',')+1);
      int el_up=cmd.substring(0,cmd.indexOf(',')).toInt();
      cmd=cmd.substring(cmd.indexOf(',')+1);
      int el_down=cmd.toInt();

      // Map to wheel powers (mecanum example)
      float RL = f - s - t;
      float RR = f + s + t;
      float FL = f + s - t;
      float FR = f - s + t;

      // Drive chassis motors
      driveMotor(motors[0], RL);
      driveMotor(motors[1], RR);
      driveMotor(motors[2], FL);
      driveMotor(motors[3], FR);

      // Elevator motors
      if(el_up){ driveMotor(motors[4], 1.0); driveMotor(motors[5], 1.0); }
      else if(el_down){ driveMotor(motors[4], -1.0); driveMotor(motors[5], -1.0); }
      else { driveMotor(motors[4], 0); driveMotor(motors[5], 0); }

      // Servo control (expand as needed)
      if(servo==1) servoElevator.write(90);
      else servoElevator.write(0);
    }
  }
}