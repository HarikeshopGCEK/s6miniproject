#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define WIFI_CHANNEL 1

typedef struct {
  char tagID[4];
  float x;
  float y;
  uint32_t ts;
} PositionPacket;

PositionPacket pkt;
unsigned long count = 0;

void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  memcpy(&pkt, data, sizeof(pkt));
  count++;

  // Machine-readable CSV for Python
  Serial.printf("DATA,%s,%.3f,%.3f\n",
                pkt.tagID,
                pkt.x,
                pkt.y);

  // Optional human-readable debug
  Serial.printf("PKT %lu | %s -> X:%.3f Y:%.3f\n",
                count,
                pkt.tagID,
                pkt.x,
                pkt.y);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);

  Serial.printf("Gateway MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2],
                mac[3], mac[4], mac[5]);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(onRecv);
  Serial.println("Gateway ready");
}

void loop() {
  delay(1000);
}
