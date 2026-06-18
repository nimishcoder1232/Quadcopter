#include <WiFi.h>
#include <esp_now.h>

uint8_t receiverMac[6] = { 0x11, 0x22, 0x33, 0x44, 0x55, 0x66 };

typedef struct {
  float throttle;
  float x;
  float y;
  float z;
  bool arm;
} ControlPacket;

ControlPacket packet;

void onSent(const uint8_t *mac, esp_now_send_status_t status) {

}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1) delay(10);
  }

  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, receiverMac, 6);
  peer.channel = 0;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add peer");
    while (1) delay(10);
  }

  Serial.print("ESP32 MAC: ");
  Serial.println(WiFi.macAddress());

  Serial.println("Waiting for controller data...");
}

void loop() {
  if (Serial.available()) {

    String line = Serial.readStringUntil('\n');
    line.trim();

    float throttle, x, y, z;
    int arm;

    int parsed = sscanf(
      line.c_str(),
      "%f,%f,%f,%f,%d",
      &throttle,
      &x,
      &y,
      &z,
      &arm
    );

    if (parsed == 5) {

      packet.throttle = constrain(throttle, 0.0f, 1.0f);
      packet.x = x;
      packet.y = y;
      packet.z = z;
      packet.arm = (arm != 0);

      esp_now_send(
        receiverMac,
        (uint8_t *)&packet,
        sizeof(packet)
      );

      Serial.printf(
        "TX: T=%.2f X=%.2f Y=%.2f Z=%.2f ARM=%d\n",
        packet.throttle,
        packet.x,
        packet.y,
        packet.z,
        packet.arm
      );
    }
  }
}