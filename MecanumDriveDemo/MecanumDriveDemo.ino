#include <probot.h>
// [Global Ayarlar Bölgesi]
#define PROBOT_WIFI_AP_PASSWORD "kayra123"
// [Global Ayarlar Bölgesi]
// Pin eşlemeleri (örnek)
#define LEFT_MOTOR_INA 8   /* DOLDUR */
#define LEFT_MOTOR_INB 9  /* DOLDUR */
#define LEFT_MOTOR_PWM 7  /* DOLDUR */
#define LEFT_MOTOR_ENA   -1   // ENA pini 3V3'e bağlıysa -1 bırakabilirsiniz
#define LEFT_MOTOR_ENB   -1   // ENB pini 3V3'e bağlıysa -1 bırakabilirsiniz
#define RIGHT_MOTOR_INA 11 /* DOLDUR */
#define RIGHT_MOTOR_INB 13 /* DOLDUR */
#define RIGHT_MOTOR_PWM 12 /* DOLDUR */
#define RIGHT_MOTOR_ENA  -1
#define RIGHT_MOTOR_ENB  -1

// [Global Ayarlar Bölgesi]
// Motor tanımları (Boardoza VNH5019 motor kontrolcusu)
static probot::motor::BoardozaVNH5019MotorController leftMotor(
  LEFT_MOTOR_INA, LEFT_MOTOR_INB, LEFT_MOTOR_PWM, LEFT_MOTOR_ENA, LEFT_MOTOR_ENB);
static probot::motor::BoardozaVNH5019MotorController rightMotor(
  RIGHT_MOTOR_INA, RIGHT_MOTOR_INB, RIGHT_MOTOR_PWM, RIGHT_MOTOR_ENA, RIGHT_MOTOR_ENB);

// [Global Ayarlar Bölgesi]
// Zamanlama (örnek başlangıç değeri)
const unsigned loopPeriodMs = 20; // her 20 ms'de bir güncelle
// Gerekirse sensör pinleri de burada tanımlanır.

// [Global Ayarlar Bölgesi]

// Probot yaşam döngüsü iskeleti
void robotInit() {
  // Robot init tuşuna basıldığında çalışacak temel kodlar
}

void robotEnd() {
  // Robot kapatıldığında temizlik için çalışacak kodlar
}


void autonomousInit() {
  // Otonom başlangıcında çalışacak kodlar
}

void autonomousLoop() {
  // Otonom döngüsünde devamlı çalışacak kodlar
}


void teleopInit() {
  // Teleop başlangıcında çalışacak kodlar
}

void teleopLoop() {
  // Teleop döngüsünde devamlı çalışacak kodlar
}