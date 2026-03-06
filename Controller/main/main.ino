#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "LAGARIMEDYA";
const char* password = "lagari5253";
WiFiUDP udp;
unsigned int localPort = 4210;
char packetBuffer[255];  

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  Serial.print("Connecting...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected!");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(localPort);
  Serial.println("UDP slave ready");
}

void loop() {
  char packet[255];
  int len = udp.parsePacket();

  if (len) {
    udp.read(packet, 255);
    packet[len] = '\0';

    String msg = String(packet);

    if (msg.startsWith("AXIS")) {
      int firstSpace = msg.indexOf(' ');
      int secondSpace = msg.indexOf(' ', firstSpace + 1);
      int axisId = msg.substring(firstSpace + 1, secondSpace).toInt();
      float val = msg.substring(secondSpace + 1).toFloat();

      Serial.printf("Axis %d value: %.3f\n", axisId, val);
    }

    else if (msg.startsWith("BUTTON")) {
      int firstSpace = msg.indexOf(' ');
      int secondSpace = msg.indexOf(' ', firstSpace + 1);
      int btnId = msg.substring(firstSpace + 1, secondSpace).toInt();
      int state = msg.substring(secondSpace + 1).toInt();

      Serial.printf("Button %d: %s\n", btnId, state ? "PRESSED" : "Released");
    }

    else if (msg.startsWith("HAT")) {
      // Format: HAT <id> <x> <y>
      int firstSpace = msg.indexOf(' ');
      int secondSpace = msg.indexOf(' ', firstSpace + 1);
      int thirdSpace = msg.indexOf(' ', secondSpace + 1);

      int hatId = msg.substring(firstSpace + 1, secondSpace).toInt();
      int x = msg.substring(secondSpace + 1, thirdSpace).toInt();
      int y = msg.substring(thirdSpace + 1).toInt();

      Serial.printf("D-pad %d: X=%d, Y=%d\n", hatId, x, y);
    }
  }
}