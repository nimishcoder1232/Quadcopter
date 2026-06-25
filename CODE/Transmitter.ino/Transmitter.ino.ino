#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

typedef struct {
  float x;
  float y; 
  float z;
  float t;
  bool arm;
} ControlPacket;

ControlPacket packet;

uint8_t receiverMac[] = {0xAA,0xBB,0xCC,0xDD,0xEE,0xFF};

esp_now_peer_info_t peer; 

float lx = 0;
float ly = 0;
float rx = 0;
float ry = 0;

unsigned long lastTime;

float throttle = 0.0f;
bool armed = true;

void setup() {
  Serial.begin(9600);
  WiFi.mode(WIFI_STA);

  memcpy(peer.peer_addr, receiverMac, 6);
  peer.channel = 0;
  peer.encrypt = false;

  esp_now_add_peer(&peer);

  lastTime = millis();
}

void loop() {
  float dt = (millis() - lastTime) / 1000.0f;
  lastTime = millis();

  // 🔴 HANDLE SERIAL INPUT
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    // 🛑 ABORT COMMAND
    if (line == "ABORT") {
      Serial.println("ABORT RECEIVED");

      armed = false;
      throttle = 0.0f;

      packet.x = 0;
      packet.y = 0;
      packet.z = 0;
      packet.t = 0;
      packet.arm = false;

      esp_now_send(receiverMac, (uint8_t*)&packet, sizeof(packet));
      return;
    }

    // 🎮 NORMAL JOYSTICK DATA
    sscanf(line.c_str(), "%f,%f,%f,%f", &lx, &ly, &rx, &ry);

    if (armed) {
      packet.x = lx * 20;     // roll angle target
      packet.y = ly * 20;     // pitch angle target
      packet.z = rx * 120;    // yaw rate
      packet.t = throttle;    // throttle 0–1
      packet.arm = true;
    } else {
      packet.x = 0;
      packet.y = 0;
      packet.z = 0;
      packet.t = 0;
      packet.arm = false;
    }

    esp_now_send(receiverMac, (uint8_t*)&packet, sizeof(packet));
  }

  // 🔼 throttle update (only if still armed)
  if (armed) {
    throttle = constrain(throttle, 0.0f, 0.8f);

    if (fabs(ry) > 0.1f) {
      throttle += ry * dt;
    }
  }
}