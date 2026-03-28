#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// WiFi channel (must match sender nodes)
#define WIFI_CHANNEL 1

// Structure of received packet from moving node
typedef struct {
  char tagID[4];   // Unique tag identifier (e.g., T1, T2)
  float x;         // X coordinate
  float y;         // Y coordinate
  uint32_t ts;     // Timestamp
} PositionPacket;

PositionPacket pkt;

// Count number of packets received
unsigned long count = 0;

// Callback function triggered when data is received via ESP-NOW
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {

  // Copy received raw data into structure
  memcpy(&pkt, data, sizeof(pkt));

  count++;

  // ── MACHINE-READABLE OUTPUT (USED BY PYTHON VISUALIZER) ──
  // Format: DATA,TagID,X,Y
  Serial.printf("DATA,%s,%.3f,%.3f\n",
                pkt.tagID,
                pkt.x,
                pkt.y);

  // ── DEBUG OUTPUT (FOR HUMAN MONITORING) ──
  Serial.printf("PKT %lu | %s -> X:%.3f Y:%.3f\n",
                count,
                pkt.tagID,
                pkt.x,
                pkt.y);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Set ESP32 in station mode (required for ESP-NOW)
  WiFi.mode(WIFI_STA);

  // Disconnect from any previous WiFi
  WiFi.disconnect();

  // Set fixed channel (must match sender)
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  // Get and print gateway MAC address
  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_STA, mac);

  Serial.printf("Gateway MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2],
                mac[3], mac[4], mac[5]);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  // Register receive callback
  esp_now_register_recv_cb(onRecv);

  Serial.println("Gateway ready");
}

void loop() {
  // No processing needed — everything handled via callback
  delay(1000);
}
