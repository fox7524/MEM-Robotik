#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

WiFiUDP udp;
unsigned int localPort = 4210;
char packetBuffer[255];
unsigned long lastAxisPacket = 0;
const unsigned long TIMEOUT = 5000;

#define WIFI_SSID "Fox-17"
#define WIFI_PSWD "Kyra2bin9"

// Global Axis States
float axis0 = 0.0; // SPIN 360
float axis2 = -1.0; // REVERSE
float axis5 = -1.0; // FORWARD
float axis3 = 0.0; // MOVE RIGHT-LEFT
int button13 = 0; // ELEVATOR UP
int button14 = 0; // ELEVATOR DOWN

float fr_sens = 0.5;
float turnSpin_sens = 0.5;
int minSpeed = 128;
int maxSpeed = 200;

int speed = 255;

const int RL_PWM = 7, RL_IN1 = 8, RL_IN2 = 9;
const int RR_PWM = 12, RR_IN1 = 13, RR_IN2 = 11;
const int FL_PWM = 15, FL_IN1 = 16, FL_IN2 = 17;
const int FR_PWM = 4, FR_IN1 = 5, FR_IN2 = 6;
const int EL_PWM = 18, EL_IN1 = 38, EL_IN2 = 39;
const int ER_PWM = 1, ER_IN1 = 2, ER_IN2 = 42;

const int E_LID = 41;
const int B_LID = 14;

void setMotor(int in1, int in2, int pwm, int dir, int speed) {
  if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwm, abs(speed));

  } else if (dir == 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(pwm, abs(speed));

  } else if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwm, abs(speed));
  } else if (dir == 2) {
    digitalWrite(in1, HIGH);  // FOR SUDDENLY BRAKING
    digitalWrite(in2, HIGH);
    digitalWrite(pwm, LOW);  // pwm low for sudden brake
  }
}


int getSpeedTrig(float axis_val) {
  float scaled = axis_val * fr_sens;
  int speed = constrain(scaled * 64 + 191, 0, maxSpeed);
  return (speed >= minSpeed) ? speed : 0;  // Cutoff below minSpeed
}

int getSpeedJoy(float axis_val) {
  float abs_val = abs(axis_val);
  float scaled = abs_val * turnSpin_sens;
  int speed = constrain((int)(scaled * 127) + 128, 128, maxSpeed);
  return (speed >= minSpeed) ? speed : 0;  // Minimum speed check only
}

void suddenStop() {
  setMotor(RL_IN1, RL_IN2, RL_PWM, 2, 0);
  setMotor(RR_IN1, RR_IN2, RR_PWM, 2, 0);
  setMotor(FL_IN1, FL_IN2, FL_PWM, 2, 0);
  setMotor(FR_IN1, FR_IN2, FR_PWM, 2, 0);
}

void stopAll() {
  setMotor(RL_IN1, RL_IN2, RL_PWM, 0, 0);
  setMotor(RR_IN1, RR_IN2, RR_PWM, 0, 0);
  setMotor(FL_IN1, FL_IN2, FL_PWM, 0, 0);
  setMotor(FR_IN1, FR_IN2, FR_PWM, 0, 0);
}

void setup() {
  Serial.begin(115200);
  pinMode(RL_IN1, OUTPUT);
  pinMode(RL_IN2, OUTPUT);
  pinMode(RL_PWM, OUTPUT);

  pinMode(RR_IN1, OUTPUT);
  pinMode(RR_IN2, OUTPUT);
  pinMode(RR_PWM, OUTPUT);

  pinMode(FL_IN1, OUTPUT);
  pinMode(FL_IN2, OUTPUT);
  pinMode(FL_PWM, OUTPUT);

  pinMode(FR_IN1, OUTPUT);
  pinMode(FR_IN2, OUTPUT);
  pinMode(FR_PWM, OUTPUT);

  pinMode(EL_IN1, OUTPUT);
  pinMode(EL_IN2, OUTPUT);
  pinMode(EL_PWM, OUTPUT);

  pinMode(ER_IN1, OUTPUT);
  pinMode(ER_IN2, OUTPUT);
  pinMode(ER_PWM, OUTPUT);

  stopAll();

  WiFi.begin(WIFI_SSID, WIFI_PSWD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi OK! IP: " + WiFi.localIP().toString());
  udp.begin(localPort);
}

void loop() {
  unsigned long now = millis();
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;
    String message = String(packetBuffer);

    if (message.startsWith("AXIS 5 ")) {
      axis5 = message.substring(7).toFloat();  
      lastAxisPacket = now;
    } else if (message.startsWith("AXIS 2 ")) {
      axis2 = message.substring(7).toFloat();
      lastAxisPacket = now;
    } else if (message.startsWith("AXIS 3 ")) {
      axis3 = message.substring(7).toFloat();
      lastAxisPacket = now;
    } else if (message.startsWith("AXIS 0 ")) {
      axis0 = message.substring(7).toFloat();
      lastAxisPacket = now;
    } else if (message.startsWith("BUTTON 12")){
      button13 = message.substring(10).toInt();
      lastAxisPacket = now;
      Serial.println(button13);

    } else if (message.startsWith("BUTTON 13")){
      button14 = message.substring(10).toInt();
      lastAxisPacket = now;
      Serial.println(button14);

    }
  }

  // --- FINAL LOGIC ENGINE ---

  // 1. Safety Timeout (If no signal for 500ms, stop)
  if (now - lastAxisPacket > TIMEOUT) {
    stopAll();
    Serial.println("TIMEOUT - CONNECTION LOST");
  }



  // 2. Drive Forward (Trigger 5)
  else if (axis5 > -0.995) {  // or inside these conditions
    //Serial.println("FORWARD");
    speed = getSpeedTrig(axis5);
    setMotor(RL_IN1, RL_IN2, RL_PWM, 1, speed);
    setMotor(RR_IN1, RR_IN2, RR_PWM, 1, speed);
    setMotor(FL_IN1, FL_IN2, FL_PWM, 1, speed);
    setMotor(FR_IN1, FR_IN2, FR_PWM, 1, speed);
    Serial.println("FORWARD:" + String(speed));
  }

  // 3. Drive Reverse (Trigger 2)
  else if (axis2 > -0.995) {
    //Serial.println("REVERSE");
    speed = getSpeedTrig(axis2);
    setMotor(RL_IN1, RL_IN2, RL_PWM, -1, speed);
    setMotor(RR_IN1, RR_IN2, RR_PWM, -1, speed);
    setMotor(FL_IN1, FL_IN2, FL_PWM, -1, speed);
    setMotor(FR_IN1, FR_IN2, FR_PWM, -1, speed);
    Serial.println("REVERSE:" + String(speed));
  }


  else if (axis0 > 0.040) {
    //Serial.println("MOVE RIGHT");
    speed = getSpeedJoy(axis0);
    setMotor(RL_IN1, RL_IN2, RL_PWM, -1, speed);
    setMotor(FL_IN1, FL_IN2, FL_PWM, 1, speed);
    setMotor(RR_IN1, RR_IN2, RR_PWM, 1, speed);
    setMotor(FR_IN1, FR_IN2, FR_PWM, -1, speed);
    Serial.println("MOVE RIGHT:" + String(speed));
  }

  else if (axis0 < -0.040) {
    //Serial.println("MOVE LEFT");
    speed = getSpeedJoy(axis0);
    setMotor(RL_IN1, RL_IN2, RL_PWM, 1, speed);
    setMotor(FL_IN1, FL_IN2, FL_PWM, -1, speed);
    setMotor(RR_IN1, RR_IN2, RR_PWM, -1, speed);
    setMotor(FR_IN1, FR_IN2, FR_PWM, 1, speed);
    Serial.println("MOVE LEFT:" + String(speed));
  }

  // 4. Spin Left (Stick pushed far left)
  else if (axis3 < -0.045) {
    //Serial.println("360 LEFT");
    speed = getSpeedJoy(axis3);
    setMotor(RL_IN1, RL_IN2, RL_PWM, -1, speed);  // Left side backwards
    setMotor(FL_IN1, FL_IN2, FL_PWM, -1, speed);
    setMotor(RR_IN1, RR_IN2, RR_PWM, 1, speed);  // Right side forwards
    setMotor(FR_IN1, FR_IN2, FR_PWM, 1, speed);
    Serial.println("360 LEFT:" + String(speed));
  }

  // 5. Spin Right (Stick pushed far right)
  else if (axis3 > 0.045) {
    //Serial.println("360 RIGHT");
    speed = getSpeedJoy(axis3);
    setMotor(RL_IN1, RL_IN2, RL_PWM, 1, speed);  // Left side forwards
    setMotor(FL_IN1, FL_IN2, FL_PWM, 1, speed);
    setMotor(RR_IN1, RR_IN2, RR_PWM, -1, speed);  // Right side backwards
    setMotor(FR_IN1, FR_IN2, FR_PWM, -1, speed);
    Serial.println("360 RIGHT:" + String(speed));
  }
/*
  else if (button13 == 1) {
    setMotor(ER_IN1, ER_IN2, ER_PWM, 1, 255);
    setMotor(EL_IN1, EL_IN2, EL_PWM, -1, 255);
    Serial.println("ELEVATOR UP");
  }
  
  else if (button14 == 1) {
    setMotor(ER_IN1, ER_IN2, ER_PWM, -1, 255);
    setMotor(EL_IN1, EL_IN2, EL_PWM, +1, 255);
    Serial.println("ELEVATOR DOWN");
  }
*/

  else {
    Serial.println("STOPPED");
    stopAll();
  }
  delay(5);
}
