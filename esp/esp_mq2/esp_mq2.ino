#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <MD5Builder.h>

#define MQ2_PIN 34
#define LED_PIN 2   // GPIO2 onboard LED

const char* ssid = "ESP32_AP";
const char* password = "12345678";
const char* mqtt_server = "192.168.1.2";  // IP máy tính chạy Mosquitto
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

const char* dev_id = "esp32_mq2";
const char* topic_data = "iot/sensor/esp32_mq2";
const char* topic_response = "iot/response";

unsigned long lastMsg = 0;
uint16_t seq_num = 0;

// void callback(char* topic, byte* payload, unsigned int length) {
//   payload[length] = '\0';
//   String msg = String((char*)payload);
//   Serial.printf("[%s] Received: %s\n", dev_id, msg.c_str());

//   StaticJsonDocument<128> doc;
//   deserializeJson(doc, msg);
//   String status = doc["status"];

//   if (status == "attack") {
//     Serial.println("⚠️  Attack detected! Activating LED/Buzzer");
//     // digitalWrite(ledPin, HIGH);
//   }
// }
void callback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';
  String msg = String((char*)payload);
  Serial.printf("[%s] Received: %s\n", dev_id, msg.c_str());

  StaticJsonDocument<128> doc;
  deserializeJson(doc, msg);
  const char* cmd = doc["cmd"];
  if (cmd) {
    if (strcmp(cmd, "led_on") == 0)  { digitalWrite(LED_PIN, HIGH);  Serial.println("LED ON"); }
    if (strcmp(cmd, "led_off")== 0) { digitalWrite(LED_PIN, LOW);   Serial.println("LED OFF"); }
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect(dev_id)) {
      Serial.println("connected!");
      client.subscribe(topic_response);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" retry in 3s");
      delay(3000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(MQ2_PIN, INPUT);

  WiFi.begin(ssid, password);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.printf("\nWiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  unsigned long now = millis();
  if (now - lastMsg > 3000) {
    lastMsg = now;
    int gasValue = analogRead(MQ2_PIN);
    int rssi = WiFi.RSSI();

    StaticJsonDocument<256> doc;
    doc["dev_id"] = dev_id;
    doc["timestamp"] = millis() / 1000;  // ← DÙNG millis() thay time(NULL)
    doc["dev_ip"] = WiFi.localIP().toString();
    doc["packet_count"] = seq_num;
    doc["packet_interval"] = 3;
    doc["dev_status"] = "normal";
    doc["seq_num"] = seq_num;
    doc["gas_level"] = gasValue;
    doc["rssi"] = rssi;
    doc["wifi_status"] = "active";
    doc["ssid"] = WiFi.SSID();

    String jsonStr;
    serializeJson(doc, jsonStr);

    MD5Builder md5;
    md5.begin();
    md5.add(jsonStr);
    md5.calculate();
    String hash = md5.toString();
    uint8_t checksum = strtol(hash.substring(0, 2).c_str(), NULL, 16);
    doc["checksum"] = checksum;

    String payload;
    serializeJson(doc, payload);

    client.publish(topic_data, payload.c_str());
    Serial.printf("[%s] Sent: %s\n", dev_id, payload.c_str());

    seq_num = (seq_num + 1) % 65536;
  }
}
