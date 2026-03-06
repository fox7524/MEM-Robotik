#include <WiFi.h>
#include <WebServer.h>

// ESP32 as Access Point
const char* ap_ssid = "ESP32-S3-N16R8";      // WiFi network name
const char* ap_password = "12345678";        // WiFi password (min 8 chars)
const IPAddress local_ip(192, 168, 4, 1);
const IPAddress gateway(192, 168, 4, 1);
const IPAddress subnet(255, 255, 255, 0);

// Web server on port 80
WebServer server(80);

// Timing
unsigned long lastSendTime = 0;
const unsigned long sendInterval = 10;  // 10msde bir package göndermesi

// data storage başlığı
String lastSensorData = "";

// Keyboard state tracking
String activeKey = "";
unsigned long lastKeyTime = 0;
const unsigned long keyResetTime = 200;  // Reset key after 200ms if no new input

// Key press log
String keyPressLog = "";
const int maxLogLines = 50;  // Keep last 50 log entries

void setup() {
  Serial.begin(115200);
  delay(2000);  // Give serial a moment to initialize
  
  Serial.println("\n\nESP32-S3 WiFi Access Point");
  Serial.println("===========================");
  
  // Start ESP32 as WiFi Access Point
  startAccessPoint();
  
  // Setup web server routes
  setupWebServer();
}

void loop() {
  // Handle incoming web requests
  server.handleClient();
  
  // Check if it's time to update sensor data
  if (millis() - lastSendTime >= sendInterval) {
    lastSendTime = millis();
    
    // Update sensor data
    lastSensorData = prepareSensorData();
    
    // Print to serial for debugging
    Serial.println("Data updated: " + lastSensorData);
  }
  
  delay(10);  // Small delay to prevent watchdog issues
}

void startAccessPoint() {
  Serial.println("Starting WiFi Access Point...");
  
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  
  boolean apStatus = WiFi.softAP(ap_ssid, ap_password);
  
  if (apStatus) {
    Serial.println("Access Point Started!");
    Serial.print("SSID: ");
    Serial.println(ap_ssid);
    Serial.print("Password: ");
    Serial.println(ap_password);
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("Failed to start Access Point");
  }
}

void setupWebServer() {
  // Route to get sensor data
  server.on("/data", HTTP_GET, []() {
    server.send(200, "application/json", lastSensorData);
  });
  
  // Route to receive keyboard input from Mac
  server.on("/keypress", HTTP_POST, []() {
    if (server.hasArg("plain")) {
      String body = server.arg("plain");
      
      // Parse JSON: {"key":"w"}
      if (body.indexOf("\"key\"") != -1) {
        int keyStart = body.indexOf(":") + 2;
        int keyEnd = body.indexOf("\"", keyStart);
        
        if (keyStart > 0 && keyEnd > keyStart) {
          activeKey = body.substring(keyStart, keyEnd);
          lastKeyTime = millis();
          
          // Add to log
          String logEntry = "[" + String(millis() / 1000) + "s] KEY PRESSED: " + activeKey + " activated\n";
          keyPressLog += logEntry;
          
          // Limit log size
          int lineCount = 0;
          for (int i = 0; i < keyPressLog.length(); i++) {
            if (keyPressLog[i] == '\n') lineCount++;
          }
          if (lineCount > maxLogLines) {
            int firstNewline = keyPressLog.indexOf('\n') + 1;
            keyPressLog = keyPressLog.substring(firstNewline);
          }
          
          // Send response back
          String response = "{\"status\":\"" + activeKey + " activated\",\"received\":true}";
          server.send(200, "application/json", response);
          
          Serial.println(logEntry);
          return;
        }
      }
    }
    
    server.send(400, "application/json", "{\"error\":\"Invalid key data\"}");
  });
  
  // Route for logs endpoint
  server.on("/logs", HTTP_GET, []() {
    server.send(200, "application/json", "{\"logs\":\"" + keyPressLog + "\"}");
  });
  
  // Route for status
  server.on("/status", HTTP_GET, []() {
    String status = "{";
    status += "\"ssid\":\"" + String(ap_ssid) + "\",";
    status += "\"connected_clients\":" + String(WiFi.softAPgetStationNum()) + ",";
    status += "\"uptime\":" + String(millis() / 1000) + "";
    status += "}";
    server.send(200, "application/json", status);
  });
  
  // Route for simple HTML page
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
    html += ".buttons-label { font-size: 12px; color: #00aaff; margin-bottom: 12px; letter-spacing: 1px; }";
    html += ".button-row { display: flex; gap: 10px; justify-content: center; margin-bottom: 10px; }";
    html += ".btn { padding: 12px 20px; font-size: 16px; font-weight: bold; background: #0a0a0a; color: #00ff00; border: 2px solid #00ff00; cursor: pointer; border-radius: 5px; font-family: 'Courier New', monospace; transition: all 0.1s; letter-spacing: 1px; }";
    html += ".btn:hover { background: #00ff00; color: #0a0a0a; box-shadow: 0 0 20px #00ff00; }";
    html += ".btn:active { transform: scale(0.95); }";
    html += ".verification-box { margin-top: 20px; padding: 12px; border: 2px solid #ff6600; background: rgba(255, 102, 0, 0.1); border-radius: 5px; min-height: 25px; }";
    html += ".verification-label { font-size: 11px; color: #ff6600; margin-bottom: 5px; }";
    html += ".verification-text { font-size: 14px; color: #ff6600; font-weight: bold; }";
    html += ".verification-text.success { color: #00ff00; border-color: #00ff00; }";
    html += ".logs-section { margin-top: 25px; width: 100%; max-width: 700px; }";
    html += ".logs-title { font-size: 12px; color: #00aaff; margin-bottom: 8px; letter-spacing: 1px; }";
    html += ".monitor { background: #0a0a0a; border: 2px solid #00ff00; padding: 12px; height: 140px; overflow-y: auto; font-size: 10px; line-height: 1.7; }";
    html += ".log-line { margin: 2px 0; color: #00ff00; padding: 2px 5px; border-left: 2px solid transparent; }";
    html += ".log-line.verified { border-left-color: #00ff00; }";
    html += ".log-line .time { color: #00aaff; }";
    html += ".log-line .key { color: #ffff00; font-weight: bold; }";
    html += ".log-line .verify-icon { color: #00ff00; }";
    html += ".footer { background: #0a0a0a; padding: 10px; border-top: 2px solid #00ff00; text-align: center; font-size: 11px; color: #00aaff; }";
    html += "</style></head><body>";
    html += "<div class='container'>";
    html += "<div class='header'><h1>⌨️ MAC → ESP32 → MAC</h1></div>";
    
    html += "<div class='content'>";
    html += "<div class='key-display'>";
    html += "<div class='key-label'>CURRENT KEY</div>";
    html += "<div class='key-box idle' id='keyBox'>IDLE</div>";
    html += "</div>";
    
    html += "<div class='buttons-section'>";
    html += "<div class='buttons-label'>SEND KEY TO ESP32</div>";
    html += "<div class='button-row'>";
    html += "<button class='btn' onclick='sendKey(\"w\")'>W</button>";
    html += "<button class='btn' onclick='sendKey(\"a\")'>A</button>";
    html += "<button class='btn' onclick='sendKey(\"s\")'>S</button>";
    html += "<button class='btn' onclick='sendKey(\"d\")'>D</button>";
    html += "</div>";
    
    html += "<div class='verification-box'>";
    html += "<div class='verification-label'>ESP32 VERIFICATION:</div>";
    html += "<div class='verification-text' id='verificationText'>Waiting for key press...</div>";
    html += "</div>";
    html += "</div>";
    
    html += "<div class='logs-section'>";
    html += "<div class='logs-title'>KEY PRESS LOG (MAC ↔ ESP32 ↔ MAC)</div>";
    html += "<div class='monitor' id='monitor'></div>";
    html += "</div>";
    html += "</div>";
    
    html += "<div class='footer'>WiFi Communication: MAC sends key → ESP32 receives & verifies → MAC receives confirmation</div>";
    html += "</div>";
    
    html += "<script>";
    html += "let lastKey = '';";
    html += "let lastLogLength = 0;";
    html += "let verificationPending = false;";
    
    html += "function sendKey(key) {";
    html += "  verificationPending = true;";
    html += "  document.getElementById('verificationText').innerText = '⏳ Sending ' + key.toUpperCase() + ' to ESP32...';";
    html += "  document.getElementById('verificationText').className = 'verification-text';";
    html += "  ";
    html += "  fetch('/keypress', {";
    html += "    method: 'POST',";
    html += "    headers: {'Content-Type': 'application/json'},";
    html += "    body: JSON.stringify({key: key})";
    html += "  })";
    html += "  .then(r => r.json())";
    html += "  .then(data => {";
    html += "    document.getElementById('verificationText').innerText = '✓ ESP32: ' + data.status;";
    html += "    document.getElementById('verificationText').className = 'verification-text success';";
    html += "    console.log('Verification received from ESP32:', data);";
    html += "    verificationPending = false;";
    html += "  })";
    html += "  .catch(e => {";
    html += "    document.getElementById('verificationText').innerText = '✗ Error: ' + e;";
    html += "    document.getElementById('verificationText').className = 'verification-text';";
    html += "    console.error('Error:', e);";
    html += "    verificationPending = false;";
    html += "  });";
    html += "}";
    
    html += "function updateStatus() {";
    html += "  fetch('/data')";
    html += "    .then(r => r.json())";
    html += "    .then(data => {";
    html += "      let keyBox = document.getElementById('keyBox');";
    html += "      let currentKey = data.activeKey || '';";
    html += "      if (currentKey && currentKey !== lastKey) {";
    html += "        keyBox.innerText = currentKey.toUpperCase();";
    html += "        keyBox.className = 'key-box';";
    html += "        lastKey = currentKey;";
    html += "      } else if (!currentKey && lastKey) {";
    html += "        keyBox.innerText = 'IDLE';";
    html += "        keyBox.className = 'key-box idle';";
    html += "        lastKey = '';";
    html += "      }";
    html += "    })";
    html += "    .catch(e => console.error('Status update error:', e));";
    html += "}";
    
    html += "function updateLogs() {";
    html += "  fetch('/logs')";
    html += "    .then(r => r.json())";
    html += "    .then(data => {";
    html += "      let monitor = document.getElementById('monitor');";
    html += "      let logs = data.logs.trim();";
    html += "      if (logs.length > lastLogLength) {";
    html += "        let lines = logs.split('\\n').filter(l => l.length > 0);";
    html += "        let recentLines = lines.slice(-10);";
    html += "        monitor.innerHTML = '';";
    html += "        recentLines.forEach(line => {";
    html += "          let div = document.createElement('div');";
    html += "          div.className = 'log-line verified';";
    html += "          let match = line.match(/\\[(.*?)\\] KEY PRESSED: (.*?) activated/);";
    html += "          if (match) {";
    html += "            let time = match[1];";
    html += "            let key = match[2].toUpperCase();";
    html += "            div.innerHTML = '<span class=\"time\">[' + time + ']</span> <span class=\"key\">' + key + '</span> activated <span class=\"verify-icon\">✓</span>';";
    html += "          }";
    html += "          monitor.appendChild(div);";
    html += "        });";
    html += "        monitor.scrollTop = monitor.scrollHeight;";
    html += "        lastLogLength = logs.length;";
    html += "      }";
    html += "    })";
    html += "    .catch(e => console.error('Log update error:', e));";
    html += "}";
    
    html += "setInterval(updateStatus, 50);";
    html += "setInterval(updateLogs, 100);";
    html += "</script>";
    html += "</body></html>";
    
    server.send(200, "text/html", html);
  });
  
  server.begin();
  Serial.println("Web server started on http://192.168.4.1");
}

String prepareSensorData() {
  // Check if key should be reset (if no new input for 200ms)
  if (millis() - lastKeyTime > keyResetTime && activeKey.length() > 0) {
    activeKey = "";
  }
  
  // Create JSON payload with sensor/system data
  String json = "{";
  
  // Custom identifier
  json += "\"KAYRA\":\"ESP32-S3-Data\",";
  
  // Timestamp
  json += "\"timestamp\":" + String(millis()) + ",";
  
  // Keyboard input status
  if (activeKey.length() > 0) {
    json += "\"activeKey\":\"" + activeKey + "\",";
    json += "\"keyStatus\":\"" + activeKey + " activated\",";
  } else {
    json += "\"activeKey\":\"\",";
    json += "\"keyStatus\":\"idle\",";
  }
  
  // ESP32 System Information
  json += "\"freeHeap\":" + String(ESP.getFreeHeap()) + ",";
  json += "\"totalHeap\":" + String(ESP.getHeapSize()) + ",";
  json += "\"heapUsagePercent\":" + String((100 * (ESP.getHeapSize() - ESP.getFreeHeap())) / ESP.getHeapSize()) + ",";
  
  // Temperature (built-in sensor)
  json += "\"temperature\":" + String(temperatureRead(), 2) + ",";
  
  // WiFi info
  json += "\"connectedClients\":" + String(WiFi.softAPgetStationNum()) + ",";
  
  // You can add more sensors here
  json += "\"chip\":\"ESP32-S3-N16R8\"";
  
  json += "}";
  
  return json;
}
