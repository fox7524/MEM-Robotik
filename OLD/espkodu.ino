#include <Arduino.h>

constexpr int NUM_CHANNELS = 14;
int ch[NUM_CHANNELS];

unsigned long lastPrint = 0;
constexpr unsigned long PRINT_INTERVAL = 1000; 

int toPercent(int val) {
  if (val < 1000) val = 1000;
  if (val > 2000) val = 2000;
  return (val - 1000) * 100 / 1000;
}

void printBar(const char* name, int val) {
  int percent = toPercent(val);
  int barLength = percent / 5; 
  Serial.print(name);
  Serial.print(": [");
  for (int i = 0; i < 20; i++) {
    if (i < barLength) Serial.print("#");
    else Serial.print(" ");
  }
  Serial.print("] ");
  Serial.print(percent);
  Serial.println("%");
}

void setup() {
  Serial.begin(115200);                           
  Serial1.begin(115200, SERIAL_8N1, 18, 17);        // RX=GPIO18, TX=GPIO17
}

void loop() {
  if (Serial1.available()) {
    String line = Serial1.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      int idx = 0;
      int start = 0;
      while (idx < NUM_CHANNELS) {
        int sep = line.indexOf(',', start);
        String token = (sep == -1) ? line.substring(start) : line.substring(start, sep);
        ch[idx++] = token.toInt();
        if (sep == -1) break;
        start = sep + 1;
      }
    }
  }

  if (millis() - lastPrint >= PRINT_INTERVAL) {
    lastPrint = millis();
    Serial.println("— Channel bars —");
    printBar("Roll (CH1)", ch[0]);
    printBar("Pitch (CH2)", ch[1]);
    printBar("Throttle (CH3)", ch[2]);
    printBar("Yaw (CH4)", ch[3]);
    printBar("SWA (CH5)", ch[4]);
    printBar("SWB (CH6)", ch[5]);
    printBar("SWC (CH7)", ch[6]);
    printBar("SWD (CH8)", ch[7]);
    printBar("VRA (CH9)", ch[8]);
    printBar("VRB (CH10)", ch[9]);
    Serial.println();
  }
}

// USB İLE AÇILIŞ - ÖNCE STM SONRA ESP32, KOD ATILIRKEN ÖNCE İKİ USB'Yİ ÇIKAR ÖNCE STM'YE KOD AT SONRA ESP32'YE KOD AT STM TAKILI
// 5V İLE AÇILIŞ - ÖNCE STM ENERJİ SONRA ESP32 ENERJİ


