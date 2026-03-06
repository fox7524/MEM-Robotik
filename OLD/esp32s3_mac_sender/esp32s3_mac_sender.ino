#include <WiFi.h>
#include <WebServer.h>

// WiFi AP config
const char* ap_ssid = "ESP32-S3-N16R8";
const char* ap_password = "12345678";
const IPAddress local_ip(192, 168, 4, 1);
const IPAddress gateway(192, 168, 4, 1);
const IPAddress subnet(255, 255, 255, 0);

WebServer server(80);

// Timing & data
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 100;
String lastSensorData = "";
String activeKey = "";
unsigned long lastKeyTime = 0;
const unsigned long keyResetTime = 200;
String keyPressLog = "";
const int maxLogLines = 50;

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n\nESP32-S3 WiFi Access Point\n=============================");
  
  // Initialize WiFi as access point
  startAccessPoint();
  
  // Setup HTTP routes and web server
  setupWebServer();
}

void loop() {
  // Process incoming HTTP requests
  server.handleClient();
  
  // Update sensor data every 100ms
  if (millis() - lastSendTime >= sendInterval) {
    lastSendTime = millis();
    lastSensorData = prepareSensorData();
    Serial.println("Data: " + lastSensorData);
  }
  delay(10);
}

void startAccessPoint() {
  Serial.println("Starting WiFi AP...");
  
  // Set ESP32 as WiFi access point (not a client)
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  
  if (WiFi.softAP(ap_ssid, ap_password)) {
    Serial.println("AP Started!");
    Serial.print("SSID: "); Serial.println(ap_ssid);
    Serial.print("Password: "); Serial.println(ap_password);
    Serial.print("IP: "); Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("AP failed");
  }
}

void setupWebServer() {
  // Route: GET /data - retrieve current sensor/key status
  server.on("/data", HTTP_GET, []() {
    server.send(200, "application/json", lastSensorData);
  });
  
  // Route: POST /keypress - receive key input from Mac
  server.on("/keypress", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      String body = server.arg("plain");
      if (body.indexOf("\"key\"") != -1) {
        int keyStart = body.indexOf(":") + 2;
        int keyEnd = body.indexOf("\"", keyStart);
        if (keyStart > 0 && keyEnd > keyStart) {
          activeKey = body.substring(keyStart, keyEnd);
          lastKeyTime = millis();
          String logEntry = "[" + String(millis() / 1000) + "s] KEY: " + activeKey + "\n";
          keyPressLog += logEntry;
          
          int lineCount = 0;
          for (int i = 0; i < keyPressLog.length(); i++) {
            if (keyPressLog[i] == '\n') lineCount++;
          }
          if (lineCount > maxLogLines) {
            keyPressLog = keyPressLog.substring(keyPressLog.indexOf('\n') + 1);
          }
          
          server.send(200, "application/json", "{\"status\":\"" + activeKey + " OK\"}");
          Serial.println(logEntry);
          return;
        }
      }
    }
    server.send(400, "application/json", "{\"error\":\"Invalid\"}");
  });
  
  // Route: GET /logs - retrieve key press log
  server.on("/logs", HTTP_GET, []() {
    server.send(200, "application/json", "{\"logs\":\"" + keyPressLog + "\"}");
  });
  
  // Route: GET /status - ESP32 system information
  server.on("/status", HTTP_GET, []() {
    String status = "{\"ssid\":\"" + String(ap_ssid) + "\",\"clients\":" + String(WiFi.softAPgetStationNum()) + ",\"uptime\":" + String(millis() / 1000) + "}";
    server.send(200, "application/json", status);
  });
  
  server.on("/", HTTP_GET, []() {
    String html = "<html><head><title>ESP32-S3 Keyboard Monitor</title>";
    html += "<style>";
    html += "* { margin: 0; padding: 0; box-sizing: border-box; }";
    html += "body { font-family: 'Courier New', monospace; background: #0a0a0a; color: #00ff00; height: 100vh; display: flex; justify-content: center; align-items: center; }";
    html += ".container { width: 100%; height: 100%; display: flex; flex-direction: column; }";
    html += ".header { background: linear-gradient(135deg, #1a1a1a 0%, #0a0a0a 100%); padding: 20px; border-bottom: 3px solid #00ff00; text-align: center; }";
    html += ".header h1 { font-size: 24px; letter-spacing: 2px; }";
    html += ".content { flex: 1; display: flex; flex-direction: column; justify-content: center; align-items: center; padding: 30px; overflow-y: auto; }";
    html += ".key-display { text-align: center; }";
    html += ".key-label { font-size: 14px; color: #00aaff; margin-bottom: 10px; letter-spacing: 1px; }";
    html += ".key-box { font-size: 100px; font-weight: bold; color: #ffff00; text-shadow: 0 0 20px #ffff00; line-height: 1; min-height: 120px; display: flex; align-items: center; justify-content: center; border: 3px solid #00ff00; padding: 15px; border-radius: 10px; background: rgba(0, 255, 0, 0.05); width: 180px; }";
    html += ".key-box.idle { color: #00ff00; text-shadow: 0 0 10px #00ff00; font-size: 35px; }";
    html += ".buttons-section { margin-top: 30px; text-align: center; }";
    html += ".buttons-label { font-size: 12px; color: #00aaff; margin-bottom: 12px; }";
    html += ".button-row { display: flex; gap: 10px; justify-content: center; margin-bottom: 10px; }";
    html += ".btn { padding: 12px 20px; font-size: 16px; font-weight: bold; background: #0a0a0a; color: #00ff00; border: 2px solid #00ff00; cursor: pointer; border-radius: 5px; font-family: 'Courier New', monospace; }";
    html += ".btn:hover { background: #00ff00; color: #0a0a0a; box-shadow: 0 0 20px #00ff00; }";
    html += ".verification-box { margin-top: 20px; padding: 12px; border: 2px solid #ff6600; background: rgba(255, 102, 0, 0.1); border-radius: 5px; }";
    html += ".verification-text { font-size: 14px; color: #ff6600; font-weight: bold; }";
    html += ".verification-text.success { color: #00ff00; }";
    html += ".logs-section { margin-top: 25px; width: 100%; max-width: 700px; }";
    html += ".logs-title { font-size: 12px; color: #00aaff; margin-bottom: 8px; }";
    html += ".monitor { background: #0a0a0a; border: 2px solid #00ff00; padding: 12px; height: 140px; overflow-y: auto; font-size: 10px; }";
    html += ".log-line { color: #00ff00; padding: 2px; border-left: 2px solid #00ff00; }";
    html += ".log-line .time { color: #00aaff; }";
    html += ".log-line .key { color: #ffff00; font-weight: bold; }";
    html += ".footer { background: #0a0a0a; padding: 10px; border-top: 2px solid #00ff00; text-align: center; font-size: 11px; color: #00aaff; }";
    html += "</style></head><body>";
    html += "<div class='container'><div class='header'><h1>⌨️ MAC ↔ ESP32</h1></div>";
    html += "<div class='content'><div class='key-display'>";
    html += "<div class='key-label'>CURRENT KEY</div>";
    html += "<div class='key-box idle' id='keyBox'>IDLE</div></div>";
    html += "<div class='buttons-section'><div class='buttons-label'>SEND KEY</div>";
    html += "<div class='button-row'>";
    html += "<button class='btn' onclick='sendKey(\"w\")'>W</button>";
    html += "<button class='btn' onclick='sendKey(\"a\")'>A</button>";
    html += "<button class='btn' onclick='sendKey(\"s\")'>S</button>";
    html += "<button class='btn' onclick='sendKey(\"d\")'>D</button></div>";
    html += "<div class='verification-box'><div class='verification-text' id='verificationText'>Waiting...</div></div></div>";
    html += "<div class='logs-section'><div class='logs-title'>KEY LOG</div>";
    html += "<div class='monitor' id='monitor'></div></div></div>";
    html += "<div class='footer'>WiFi Bridge: MAC ↔ ESP32-S3</div></div>";
    html += "<script>";
    html += "let lastKey='', lastLogLength=0;";
    html += "function sendKey(key) {";
    html += "  document.getElementById('verificationText').innerText='⏳ Sending...';";
    html += "  fetch('/keypress', {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify({key})})";
    html += "  .then(r => r.json()).then(d => {";
    html += "    document.getElementById('verificationText').innerText='✓ '+d.status;";
    html += "    document.getElementById('verificationText').className='verification-text success';";
    html += "  }).catch(e => {";
    html += "    document.getElementById('verificationText').innerText='✗ Error';";
    html += "  });";
    html += "}";
    html += "function updateStatus() {";
    html += "  fetch('/data').then(r => r.json()).then(d => {";
    html += "    let kb=document.getElementById('keyBox'), ck=d.activeKey||'';";
    html += "    if (ck && ck !== lastKey) { kb.innerText=ck.toUpperCase(); kb.className='key-box'; lastKey=ck; }";
    html += "    else if (!ck && lastKey) { kb.innerText='IDLE'; kb.className='key-box idle'; lastKey=''; }";
    html += "  });";
    html += "}";
    html += "function updateLogs() {";
    html += "  fetch('/logs').then(r => r.json()).then(d => {";
    html += "    let m=document.getElementById('monitor'), logs=d.logs.trim();";
    html += "    if (logs.length > lastLogLength) {";
    html += "      m.innerHTML='';";
    html += "      logs.split('\\n').filter(l=>l).slice(-10).forEach(l => {";
    html += "        let match=l.match(/\\[(.*?)\\] KEY: (.*?) /);";
    html += "        if (match) {";
    html += "          let div=document.createElement('div');";
    html += "          div.className='log-line';";
    html += "          div.innerHTML='<span class=\"time\">['+match[1]+']</span> <span class=\"key\">'+match[2].toUpperCase()+'</span> ✓';";
    html += "          m.appendChild(div);";
    html += "        }";
    html += "      });";
    html += "      m.scrollTop=m.scrollHeight; lastLogLength=logs.length;";
    html += "    }";
    html += "  });";
    html += "}";
    html += "setInterval(updateStatus,50); setInterval(updateLogs,100);";
    html += "</script></body></html>";
    server.send(200, "text/html", html);
  });
  
  // Start HTTP server listening for requests
  server.begin();
  Serial.println("Web server on http://192.168.4.1");
}

// Build JSON with current key state and system info
String prepareSensorData() {
  // Reset key if no input received within timeout period
  if (millis() - lastKeyTime > keyResetTime && activeKey.length() > 0) {
    activeKey = "";
  }
  
  // Build JSON response with all sensor data
  String json = "{";
  json += "\"KAYRA\":\"ESP32-S3\",";
  json += "\"timestamp\":" + String(millis()) + ",";
  
  if (activeKey.length() > 0) {
    json += "\"activeKey\":\"" + activeKey + "\",\"keyStatus\":\"" + activeKey + " active\",";
  } else {
    json += "\"activeKey\":\"\",\"keyStatus\":\"idle\",";
  }
  
  json += "\"freeHeap\":" + String(ESP.getFreeHeap()) + ",";
  json += "\"totalHeap\":" + String(ESP.getHeapSize()) + ",";
  json += "\"heapUsage\":" + String((100 * (ESP.getHeapSize() - ESP.getFreeHeap())) / ESP.getHeapSize()) + ",";
  json += "\"temperature\":" + String(temperatureRead(), 2) + ",";
  json += "\"clients\":" + String(WiFi.softAPgetStationNum()) + ",";
  json += "\"chip\":\"ESP32-S3-N16R8\"";
  json += "}";
  
  return json;
}
