#include <WiFi.h>

// === KẾT NỐI RA NGOÀI (iPhone hotspot) ===
const char* sta_ssid = "Duc Long";  // ĐỔI THÀNH TÊN HOTSPOT CỦA BẠN
const char* sta_pass = "18092004";        // MẬT KHẨU

// === AP NỘI BỘ ===
const char* ap_ssid = "ESP32_AP";
const char* ap_pass = "12345678";

IPAddress local_IP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

// === MQTT BROKER TRÊN MÁY TÍNH ===
const char* host_broker = "172.20.10.3";  // IP MÁY TÍNH (xem bằng ipconfig)
const uint16_t host_port = 1883;

// === PROXY ===
WiFiServer proxyServer(1883);
WiFiClient serverClients[4];
WiFiClient brokerClient;

void setup() {
  Serial.begin(115200);
  delay(1000);

  // 1. KẾT NỐI STA
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(sta_ssid, sta_pass);
  Serial.print("Connecting to hotspot");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nSTA OK: " + WiFi.localIP().toString());

  // 2. KHỞI ĐỘNG AP
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(ap_ssid, ap_pass);
  Serial.println("AP OK: " + WiFi.softAPIP().toString());
  // 3. PROXY
  proxyServer.begin();
  proxyServer.setNoDelay(true);
  Serial.println("Proxy OK: 192.168.4.1:1883 → 172.20.10.3:1883");
}

void loop() {
  // Chấp nhận client
  if (proxyServer.hasClient()) {
    WiFiClient c = proxyServer.available();
    for (int i = 0; i < 4; i++) {
      if (!serverClients[i]) {
        serverClients[i] = c;
        Serial.println("Client in: " + c.remoteIP().toString());
        break;
      }
    }
  }

  // Kết nối broker
  if (!brokerClient.connected()) {
    Serial.print("Connecting to Mosquitto...");
    if (brokerClient.connect(host_broker, host_port)) {
      Serial.println(" OK");
    } else {
      Serial.println(" failed");
      delay(3000);
      return;
    }
  }

  // Forward
  for (int i = 0; i < 4; i++) {
    if (serverClients[i] && serverClients[i].connected() && brokerClient.connected()) {
      while (serverClients[i].available()) brokerClient.write(serverClients[i].read());
      while (brokerClient.available()) serverClients[i].write(brokerClient.read());
    } else if (serverClients[i]) {
      serverClients[i].stop();
    }
  }
  yield();
}