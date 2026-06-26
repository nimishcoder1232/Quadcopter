#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

typedef struct {
  float throttle;
  float x;
  float y;
  float z;
  bool arm;
} ControlPacket;

ControlPacket packet;

uint8_t receiverMac[] = {0x30, 0xED, 0xA0, 0xA4, 0xD8, 0x38};

esp_now_peer_info_t peer;

float lx = 0;
float ly = 0;
float rx = 0;
float ry = 0;

float throttle = 0.0f;
bool armed = true;

unsigned long lastTime;

void setup() {
  Serial.begin(9600);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1);
  }

  memcpy(peer.peer_addr, receiverMac, 6);
  peer.channel = 0;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add peer");
    while (1);
  }

  Serial.println("Transmitter Ready");
  lastTime = millis();
}

void loop() {
  float dt = (millis() - lastTime) / 1000.0f;
  lastTime = millis();

  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    // Abort command from Python
    if (line == "ABORT") {
      Serial.println("ABORT RECEIVED");

      armed = false;
      throttle = 0.0f;

      packet.throttle = 0.0f;
      packet.x = 0.0f;
      packet.y = 0.0f;
      packet.z = 0.0f;
      packet.arm = false;

      esp_now_send(receiverMac, (uint8_t *)&packet, sizeof(packet));
      return;
    }

    // Parse joystick values: lx,ly,rx,ry
    if (sscanf(line.c_str(), "%f,%f,%f,%f", &lx, &ly, &rx, &ry) == 4) {

      // Throttle controlled by right stick Y
      if (armed && fabs(ly) > 0.1f) {
        throttle += ly * dt;
      }

      throttle = constrain(throttle, 0.0f, 0.8f);

      packet.throttle = throttle;
      packet.x = rx * 20.0f;      // X angle target (deg)
      packet.y = ry * 20.0f;      // Y angle target (deg)
      packet.z = lx * 60.0f;     // Z rate target (deg/s)
      packet.arm = armed;

      esp_now_send(receiverMac, (uint8_t *)&packet, sizeof(packet));
    }
  }
  delay(20);
}