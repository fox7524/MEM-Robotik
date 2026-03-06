#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

// --- MOTOR PIN DEFINITIONS ---
const int RL_PWM = 7;   
const int RL_IN1 = 8;   
const int RL_IN2 = 9;   

const int RR_PWM = 12;
const int RR_IN1 = 11;
const int RR_IN2 = 13;

const int FL_PWM = 15;
const int FL_IN1 = 16;
const int FL_IN2 = 17;

const int FR_PWM = 4;
const int FR_IN1 = 5;
const int FR_IN2 = 6;

// --- ELEVATOR LEFT MOTOR ---
const int EL_PWM = 18;
const int EL_IN1 = 38;
const int EL_IN2 = 39;

// --- ELEVATOR RIGHT MOTOR ---
const int ER_PWM = 1;
const int ER_IN1 = 2;
const int ER_IN2 = 42;

// --- SERVO LID ---
const int SERVO_PIN = 14;
Servo lidServo;

// --- WIFI SETTINGS ---
WiFiUDP udp;
unsigned int localPort = 4210;
char packetBuffer[255];

// --- CONTROL INPUTS ---
float a0 = 0;   // Rotation axis (left/right spin)
float a2 = 0;   // Slide axis (left/right strafe)
float a4 = 0;   // Backward trigger
float a5 = 0;   // Forward trigger
bool b0 = false;   // Servo toggle
bool b11 = false;  // Elevator up
bool b12 = false;  // Elevator down

// --- ELEVATOR VARIABLES ---
unsigned long elevUpStart = 0;
unsigned long elevDownStart = 0;
unsigned long elevUpTotal = 0;
unsigned long elevDownTotal = 0;
const unsigned long ELEV_LIMIT = 3000;  // 3 seconds
bool elevUpActive = false;
bool elevDownActive = false;
bool elevUpLocked = false;
bool elevDownLocked = false;

// --- SERVO VARIABLES ---
bool servoOpen = false;
bool lastB0 = false;

unsigned long lastUpdate = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\nBASLATILDI!");
  
  // Setup motor pins
  pinMode(RL_PWM, OUTPUT); pinMode(RL_IN1, OUTPUT); pinMode(RL_IN2, OUTPUT);
  pinMode(RR_PWM, OUTPUT); pinMode(RR_IN1, OUTPUT); pinMode(RR_IN2, OUTPUT);
  pinMode(FL_PWM, OUTPUT); pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT);
  pinMode(FR_PWM, OUTPUT); pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT);
  
  // Elevator motor pins
  pinMode(EL_PWM, OUTPUT); pinMode(EL_IN1, OUTPUT); pinMode(EL_IN2, OUTPUT);
  pinMode(ER_PWM, OUTPUT); pinMode(ER_IN1, OUTPUT); pinMode(ER_IN2, OUTPUT);
  
  // Initialize all motors to LOW/OFF
  digitalWrite(RL_IN1, LOW); digitalWrite(RL_IN2, LOW); analogWrite(RL_PWM, 0);
  digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, LOW); analogWrite(RR_PWM, 0);
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, LOW); analogWrite(FL_PWM, 0);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, LOW); analogWrite(FR_PWM, 0);
  
  // Initialize elevator motors to LOW/OFF
  digitalWrite(EL_IN1, LOW); digitalWrite(EL_IN2, LOW); analogWrite(EL_PWM, 0);
  digitalWrite(ER_IN1, LOW); digitalWrite(ER_IN2, LOW); analogWrite(ER_PWM, 0);
  
  // Servo
  lidServo.attach(SERVO_PIN);
  lidServo.write(0);
  servoOpen = false;
  
  // Stop all motors
  stopAll();
  elevatorStop();
  
  // WiFi connection
  WiFi.begin("Ben", "sitrayburgbiriki3");
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
  // WiFi watchdog
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi Koptu, Restart...");
    ESP.restart();
  }
  
  // Parse UDP packets
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, 255);
    if (len > 0 && len < 255) {
      packetBuffer[len] = 0;
      String msg = String(packetBuffer);
      
      parseInput(msg);
      
      // Send to serial for debugging
      Serial.printf("A0:%.2f A2:%.2f A4:%.2f A5:%.2f B0:%d B11:%d B12:%d\n", a0, a2, a4, a5, b0, b11, b12);
      
      // Process movement
      processMovement();
    }
  } else {
    // No packet received - ensure all motors are OFF
    stopAll();
    controlElevator();  // Still handle elevator timers
    controlServo();     // Still handle servo
  }
}

void parseInput(String msg) {
  if (msg.startsWith("AXIS 0 ")) a0 = msg.substring(7).toFloat();
  if (msg.startsWith("AXIS 2 ")) a2 = msg.substring(7).toFloat();
  if (msg.startsWith("AXIS 4 ")) a4 = msg.substring(7).toFloat();
  if (msg.startsWith("AXIS 5 ")) a5 = msg.substring(7).toFloat();
  if (msg.startsWith("BUTTON 0 ")) b0 = (msg.substring(9).toInt() > 0);
  if (msg.startsWith("BUTTON 11 ")) b11 = (msg.substring(10).toInt() > 0);
  if (msg.startsWith("BUTTON 12 ")) b12 = (msg.substring(10).toInt() > 0);
}

void processMovement() {
  // DEADZONE CHECK - Her şeyi durumlu başlat
  bool deadzone = (abs(a2) <= 0.1 && abs(a0) <= 0.1 && a5 <= -0.8 && a4 <= -0.8);
  if (deadzone) {
    stopAll();
    return;
  }
  
  // PRIORITY: Slide > Rotation > Forward/Backward
  
  // Handle elevator
  controlElevator();
  
  // Handle servo
  controlServo();
  
  // 1. SLIDE HAREKETI (SAG-SOL)
  if (abs(a2) > 0.15) {
    int pwm = map(abs(a2) * 100, 15, 100, 100, 255);
    pwm = constrain(pwm, 100, 255);
    
    if (a2 < 0) {
      Serial.printf("SLIDE LEFT: a2=%.3f pwm=%d\n", a2, pwm);
      slideLeft(pwm);
    } else {
      Serial.printf("SLIDE RIGHT: a2=%.3f pwm=%d\n", a2, pwm);
      slideRight(pwm);
    }
  }
  // 2. 360 DONME HAREKETI
  else if (abs(a0) > 0.15) {
    int pwm = map(abs(a0) * 100, 15, 100, 100, 255);
    pwm = constrain(pwm, 100, 255);
    
    if (a0 > 0) {
      Serial.printf("ROTATE RIGHT: a0=%.3f pwm=%d\n", a0, pwm);
      rotateRight(pwm);
    } else {
      Serial.printf("ROTATE LEFT: a0=%.3f pwm=%d\n", a0, pwm);
      rotateLeft(pwm);
    }
  }
  // 3. ILERI HAREKETI
  else if (a5 > -0.5) {
    int pwm = map((a5 + 1) * 50, 0, 100, 80, 255);
    pwm = constrain(pwm, 80, 255);
    Serial.printf("FORWARD: a5=%.3f pwm=%d\n", a5, pwm);
    moveForward(pwm);
  }
  // 4. GERI HAREKETI
  else if (a4 > -0.5) {
    int pwm = map((a4 + 1) * 50, 0, 100, 80, 255);
    pwm = constrain(pwm, 80, 255);
    Serial.printf("BACKWARD: a4=%.3f pwm=%d\n", a4, pwm);
    moveBackward(pwm);
  }
  // 5. DURMA
  else {
    Serial.println("STOP ALL");
    stopAll();
  }
}

// --- HAREKET FONKSIYONLARI ---

void moveForward(int pwm) {
  // Tum motorlar ileri - AYNI YÖN
  digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, LOW);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
  digitalWrite(RL_IN1, HIGH); digitalWrite(RL_IN2, LOW);
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
  
  delay(5);
  
  analogWrite(RR_PWM, pwm);
  analogWrite(FR_PWM, pwm);
  analogWrite(RL_PWM, pwm);
  analogWrite(FL_PWM, pwm);
  
  Serial.printf("FWD: RR=%d FR=%d RL=%d FL=%d\n", pwm, pwm, pwm, pwm);
}

void moveBackward(int pwm) {
  // Tum motorlar geri - AYNI YÖN
  digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, HIGH);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH);
  digitalWrite(RL_IN1, LOW); digitalWrite(RL_IN2, HIGH);
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH);
  
  delay(5);
  
  analogWrite(RR_PWM, pwm);
  analogWrite(FR_PWM, pwm);
  analogWrite(RL_PWM, pwm);
  analogWrite(FL_PWM, pwm);
  
  Serial.printf("BWD: RR=%d FR=%d RL=%d FL=%d\n", pwm, pwm, pwm, pwm);
}

void slideLeft(int pwm) {
  // SAG TARAF MOTORLAR ILERIYE, SOL TARAF MOTORLAR GERIYE
  digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, LOW);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
  digitalWrite(RL_IN1, LOW); digitalWrite(RL_IN2, HIGH);
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH);
  
  analogWrite(RR_PWM, pwm);
  analogWrite(FR_PWM, pwm);
  analogWrite(RL_PWM, pwm);
  analogWrite(FL_PWM, pwm);
}

void slideRight(int pwm) {
  // SOL TARAF MOTORLAR ILERIYE, SAG TARAF MOTORLAR GERIYE
  digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, HIGH);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH);
  digitalWrite(RL_IN1, HIGH); digitalWrite(RL_IN2, LOW);
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
  
  analogWrite(RR_PWM, pwm);
  analogWrite(FR_PWM, pwm);
  analogWrite(RL_PWM, pwm);
  analogWrite(FL_PWM, pwm);
}

void rotateLeft(int pwm) {
  // SOLA DONME: SAGI SAG, SOLU SOL
  digitalWrite(RR_IN1, HIGH); digitalWrite(RR_IN2, LOW);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
  digitalWrite(RL_IN1, LOW); digitalWrite(RL_IN2, HIGH);
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH);
  
  analogWrite(RR_PWM, pwm);
  analogWrite(FR_PWM, pwm);
  analogWrite(RL_PWM, pwm);
  analogWrite(FL_PWM, pwm);
}

void rotateRight(int pwm) {
  // SAGA DONME: SOLU SAG, SAGI SOL
  digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, HIGH);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH);
  digitalWrite(RL_IN1, HIGH); digitalWrite(RL_IN2, LOW);
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
  
  analogWrite(RR_PWM, pwm);
  analogWrite(FR_PWM, pwm);
  analogWrite(RL_PWM, pwm);
  analogWrite(FL_PWM, pwm);
}

void stopAll() {
  // Tum motorlari immediate OFF
  digitalWrite(RR_IN1, LOW); digitalWrite(RR_IN2, LOW);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, LOW);
  digitalWrite(RL_IN1, LOW); digitalWrite(RL_IN2, LOW);
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, LOW);
  
  analogWrite(RR_PWM, 0);
  analogWrite(FR_PWM, 0);
  analogWrite(RL_PWM, 0);
  analogWrite(FL_PWM, 0);
  
  Serial.println("STOP");
}

// --- ELEVATOR CONTROL FUNCTIONS ---

void controlElevator() {
  // YUKARIYA ÇIKMA KONTROLÜ (Button 11)
  if (b11 && !elevUpLocked) {
    if (!elevUpActive) {
      elevUpStart = millis();
      elevUpActive = true;
      Serial.println("ASC: UP START");
    }
    
    elevUpTotal = millis() - elevUpStart;
    if (elevUpTotal < ELEV_LIMIT) {
      elevatorUp();
    } else {
      elevatorStop();
      elevUpLocked = true;
      elevUpActive = false;
      Serial.println("ASC: UP LIMIT REACHED");
    }
  } else if (!b11 && elevUpActive) {
    elevatorStop();
    elevUpActive = false;
  }
  
  // AŞAĞIYA İNME KONTROLÜ (Button 12)
  if (b12 && !elevDownLocked) {
    if (!elevDownActive) {
      elevDownStart = millis();
      elevDownActive = true;
      Serial.println("ASC: DOWN START");
    }
    
    elevDownTotal = millis() - elevDownStart;
    if (elevDownTotal < ELEV_LIMIT) {
      elevatorDown();
    } else {
      elevatorStop();
      elevDownLocked = true;
      elevDownActive = false;
      Serial.println("ASC: DOWN LIMIT REACHED");
    }
  } else if (!b12 && elevDownActive) {
    elevatorStop();
    elevDownActive = false;
  }
}

void elevatorUp() {
  // Left motor: in1=HIGH, in2=LOW (forward)
  // Right motor: in1=LOW, in2=HIGH (backward - opposite direction)
  digitalWrite(EL_IN1, HIGH);
  digitalWrite(EL_IN2, LOW);
  digitalWrite(ER_IN1, LOW);
  digitalWrite(ER_IN2, HIGH);
  
  analogWrite(EL_PWM, 200);
  analogWrite(ER_PWM, 200);
  
}

void elevatorDown() {
  // Left motor: in1=LOW, in2=HIGH (backward)
  // Right motor: in1=HIGH, in2=LOW (forward - opposite direction)
  digitalWrite(EL_IN1, LOW);
  digitalWrite(EL_IN2, HIGH);
  digitalWrite(ER_IN1, HIGH);
  digitalWrite(ER_IN2, LOW);
  
  analogWrite(EL_PWM, 200);
  analogWrite(ER_PWM, 200);
}

void elevatorStop() {
  // Brake
  digitalWrite(EL_IN1, HIGH);
  digitalWrite(EL_IN2, HIGH);
  digitalWrite(ER_IN1, HIGH);
  digitalWrite(ER_IN2, HIGH);
  
  analogWrite(EL_PWM, 255);
  analogWrite(ER_PWM, 255);
  
  delay(30);
  
  // Coast
  digitalWrite(EL_IN1, LOW);
  digitalWrite(EL_IN2, LOW);
  digitalWrite(ER_IN1, LOW);
  digitalWrite(ER_IN2, LOW);
  
  analogWrite(EL_PWM, 0);
  analogWrite(ER_PWM, 0);
}

// --- SERVO CONTROL FUNCTIONS ---

void controlServo() {
  if (b0 && !lastB0) {
    // Toggle on button press
    servoOpen = !servoOpen;
    
    if (servoOpen) {
      lidServo.write(90);
      Serial.println("SERVO: OPEN (90)");
    } else {
      lidServo.write(0);
      Serial.println("SERVO: CLOSE (0)");
    }
  }
  lastB0 = b0;
}

