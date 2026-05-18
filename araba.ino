// 192.168.4.1
// MECHATAK - Final versiyon kodu
 
 
 #define PROBOT_WIFI_AP_PASSWORD "ATAKFL"
 #define PROBOT_WIFI_AP_SSID "ATAKFL"
 #define PROBOT_WIFI_AP_CHANNEL 9
 
 #include <Arduino.h>
 #include <WiFi.h>
 #include <WiFiUdp.h>
 
 const int L_PWM = 4;
 const int L_IN1 = 5;
 const int L_IN2 = 6;
 
 const int R_PWM = 7;
 const int R_IN1 = 8;
 const int R_IN2 = 9;
 
 static WiFiUDP udp;
 static constexpr uint16_t UDP_LOCAL_PORT = 4210;
 static char packetBuffer[256];
 
 struct InputState {
   float axis[8];
   int button[32];
   int hatX = 0;
   int hatY = 0;
   uint32_t seq = 0;
 };
 
 static InputState in;
 static IPAddress lastRemoteIP;
 static uint16_t lastRemotePort = 0;
 static unsigned long lastRxMs = 0;
 static unsigned long lastTxMs = 0;
 
 static float trigger01(float v) {
   if (v < -0.1f) return constrain((v + 1.0f) * 0.5f, 0.0f, 1.0f);
   return constrain(v, 0.0f, 1.0f);
 }
 
 static void setMotor(int in1, int in2, int pwmPin, int pwm) {
   pwm = constrain(pwm, -255, 255);
   if (pwm > 0) {
     digitalWrite(in1, HIGH);
     digitalWrite(in2, LOW);
     analogWrite(pwmPin, pwm);
   } else if (pwm < 0) {
     digitalWrite(in1, LOW);
     digitalWrite(in2, HIGH);
     analogWrite(pwmPin, -pwm);
   } else {
     digitalWrite(in1, LOW);
     digitalWrite(in2, LOW);
     analogWrite(pwmPin, 0);
   }
 }
 
 void dur() {
   setMotor(L_IN1, L_IN2, L_PWM, 0);
   setMotor(R_IN1, R_IN2, R_PWM, 0);
 }
 
 void anidur() {
   digitalWrite(L_IN1, HIGH);
   digitalWrite(L_IN2, HIGH);
   digitalWrite(R_IN1, HIGH);
   digitalWrite(R_IN2, HIGH);
   analogWrite(L_PWM, 255);
   analogWrite(R_PWM, 255);
   delay(30);
   dur();
 }
 
 void fanidur() {
   digitalWrite(L_IN1, HIGH);
   digitalWrite(L_IN2, HIGH);
   digitalWrite(R_IN1, HIGH);
   digitalWrite(R_IN2, HIGH);
   analogWrite(L_PWM, 255);
   analogWrite(R_PWM, 255);
   delay(10);
   dur();
 }
 
 void ileri(int pwm) {
   setMotor(L_IN1, L_IN2, L_PWM, pwm);
   setMotor(R_IN1, R_IN2, R_PWM, pwm);
 }
 
 void geri(int pwm) {
   setMotor(L_IN1, L_IN2, L_PWM, -pwm);
   setMotor(R_IN1, R_IN2, R_PWM, -pwm);
 }
 
 void sagslide(int pwm) {
   setMotor(L_IN1, L_IN2, L_PWM, pwm);
   setMotor(R_IN1, R_IN2, R_PWM, 0);
 }
 
 void solslide(int pwm) {
   setMotor(L_IN1, L_IN2, L_PWM, 0);
   setMotor(R_IN1, R_IN2, R_PWM, pwm);
 }
 
 void sol360(int pwm) {
   setMotor(L_IN1, L_IN2, L_PWM, -pwm);
   setMotor(R_IN1, R_IN2, R_PWM, pwm);
 }
 
 void sag360(int pwm) {
   setMotor(L_IN1, L_IN2, L_PWM, pwm);
   setMotor(R_IN1, R_IN2, R_PWM, -pwm);
 }
 
 static void parseInput(const String& msg) {
   int s1 = msg.indexOf(' ');
   if (s1 < 0) return;
   int s2 = msg.indexOf(' ', s1 + 1);
   if (s2 < 0) return;
 
   String type = msg.substring(0, s1);
   int id = msg.substring(s1 + 1, s2).toInt();
   String v = msg.substring(s2 + 1);
 
   if (type == "AXIS") {
     if (id >= 0 && id < (int)(sizeof(in.axis) / sizeof(in.axis[0]))) {
       in.axis[id] = v.toFloat();
     }
   } else if (type == "BUTTON") {
     if (id >= 0 && id < (int)(sizeof(in.button) / sizeof(in.button[0]))) {
       in.button[id] = v.toInt();
     }
   } else if (type == "HAT") {
     int s3 = v.indexOf(' ');
     if (s3 < 0) return;
     int x = v.substring(0, s3).toInt();
     int y = v.substring(s3 + 1).toInt();
     in.hatX = x;
     in.hatY = y;
   }
 }
 
 static void sendTelemetryLine(const String& payload) {
  if (lastRemotePort == 0) return;
  if (lastRemoteIP == IPAddress(0, 0, 0, 0)) return;
   udp.beginPacket(lastRemoteIP, lastRemotePort);
   udp.print(payload);
   udp.endPacket();
 }
 
 void robotInit() {
   pinMode(L_PWM, OUTPUT);
   pinMode(L_IN1, OUTPUT);
   pinMode(L_IN2, OUTPUT);
 
   pinMode(R_PWM, OUTPUT);
   pinMode(R_IN1, OUTPUT);
   pinMode(R_IN2, OUTPUT);
 
   dur();
 }
 
 void autonomousInit() {}
 void autonomousLoop() {}
 void teleopInit() {}
 
 void teleopLoop() {
   int speed = 0;
 
   float rt = trigger01(in.axis[5]);
   float lt = trigger01(in.axis[2]);
   float lt_alt = trigger01(in.axis[4]);
   if (lt_alt > lt) lt = lt_alt;
 
   float leftX = in.axis[0];
   float rightX = in.axis[3];
 
   String tlm;
   tlm.reserve(256);
   tlm += "=== JOYSTICK ===\n";
   tlm += "Axis0=" + String(leftX, 2) + " Axis2=" + String(in.axis[2], 2) + " Axis3=" + String(rightX, 2) + " Axis4=" + String(in.axis[4], 2) + " Axis5=" + String(in.axis[5], 2) + "\n";
   tlm += "LT=" + String(lt, 2) + " RT=" + String(rt, 2) + "\n";
   tlm += "HAT=" + String(in.hatX) + "," + String(in.hatY) + "\n";
   tlm += "Seq=" + String(in.seq) + "\n";
 
   if (rt > 0.05f) {
     speed = map((int)lroundf(rt * 100.0f), 0, 100, 0, 255);
     ileri(speed);
     tlm += "YON: ILERI | HIZ: " + String(speed) + "\n";
   } else if (lt > 0.05f) {
     speed = map((int)lroundf(lt * 100.0f), 0, 100, 0, 255);
     geri(speed);
     tlm += "YON: GERI | HIZ: " + String(speed) + "\n";
   } else if (leftX > 0.1f) {
     speed = map((int)lroundf(leftX * 100.0f), 0, 100, 0, 255);
     sagslide(speed);
     tlm += "YON: SAG | HIZ: " + String(speed) + "\n";
   } else if (leftX < -0.1f) {
     speed = map((int)lroundf(fabsf(leftX) * 100.0f), 0, 100, 0, 255);
     solslide(speed);
     tlm += "YON: SOL | HIZ: " + String(speed) + "\n";
   } else if (rightX > 0.1f) {
     speed = map((int)lroundf(rightX * 100.0f), 0, 100, 0, 255);
     sag360(speed);
     tlm += "YON: SAG DONUS | HIZ: " + String(speed) + "\n";
   } else if (rightX < -0.1f) {
     speed = map((int)lroundf(fabsf(rightX) * 100.0f), 0, 100, 0, 255);
     sol360(speed);
     tlm += "YON: SOL DONUS | HIZ: " + String(speed) + "\n";
   } else {
     anidur();
     tlm += "DURUM: BEKLEMEDE\n";
   }
 
   unsigned long now = millis();
   if (now - lastTxMs >= 100) {
     lastTxMs = now;
     sendTelemetryLine(tlm);
   }
 
   delay(20);
 }
 
 void robotEnd() {
   delay(34);
   fanidur();
 }
 
 static void setupWifiAp() {
   WiFi.mode(WIFI_AP);
   IPAddress local_ip(192, 168, 4, 1);
   IPAddress gateway(192, 168, 4, 1);
   IPAddress subnet(255, 255, 255, 0);
   WiFi.softAPConfig(local_ip, gateway, subnet);
   WiFi.softAP(PROBOT_WIFI_AP_SSID, PROBOT_WIFI_AP_PASSWORD, PROBOT_WIFI_AP_CHANNEL);
 }
 
 void setup() {
   Serial.begin(115200);
   delay(200);
 
   robotInit();
   setupWifiAp();
 
   udp.begin(UDP_LOCAL_PORT);
   lastRxMs = millis();
   teleopInit();
 }
 
 void loop() {
   unsigned long now = millis();
 
   int packetSize = udp.parsePacket();
   while (packetSize > 0) {
     int len = udp.read(packetBuffer, (int)sizeof(packetBuffer) - 1);
     if (len > 0) {
       packetBuffer[len] = 0;
       lastRemoteIP = udp.remoteIP();
       lastRemotePort = udp.remotePort();
       lastRxMs = now;
       in.seq++;
       parseInput(String(packetBuffer));
     }
     packetSize = udp.parsePacket();
   }
 
   if (now - lastRxMs > 250) {
     dur();
     delay(10);
     return;
   }
 
   teleopLoop();
 }
