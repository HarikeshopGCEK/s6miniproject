#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <math.h>

#define WIFI_CHANNEL 1
#define RSSI_SAMPLES 10
#define RSSI_TIMEOUT 8000   // Increased timeout for stability

uint8_t gatewayMAC[] = {0x3C, 0x8A, 0x1F, 0x9D, 0x3F, 0x04};
const char* TAG_ID = "T2";

typedef struct {
  char tagID[4];
  float x;
  float y;
  uint32_t ts;
} PositionPacket;

PositionPacket packet;
BLEScan *pBLEScan;

float txPower = -63.0;   // CALIBRATE THIS
float n = 2.2;           // CALIBRATE THIS
float D = 2.0;           // Distance between anchors

struct BeaconData {
  int rssi_buf[RSSI_SAMPLES];
  unsigned long last_seen;
  bool detected;
};

BeaconData beaconA, beaconB, beaconC;
int buf_idx = 0;

// ----------------------
// Distance Calculation
// ----------------------
float rssiToDistance(int rssi) {
  if (rssi >= 0 || rssi < -100) return 999.0;
  return pow(10.0, (txPower - rssi) / (10.0 * n));
}

// ----------------------
// RSSI Filtering
// ----------------------
int filterRSSI(BeaconData* b) {
  if (millis() - b->last_seen > RSSI_TIMEOUT) {
    b->detected = false;
    return -100;
  }

  long sum = 0;
  for (int i = 0; i < RSSI_SAMPLES; i++)
    sum += b->rssi_buf[i];

  return sum / RSSI_SAMPLES;
}

// ----------------------
// Beacon Update
// ----------------------
void updateBeacon(BeaconData* b, int rssi) {
  b->rssi_buf[buf_idx % RSSI_SAMPLES] = rssi;
  b->last_seen = millis();
  b->detected = true;
}

// ----------------------
// BLE Callback
// ----------------------
class ScanCB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice d) {
    String name = d.getName().c_str();
    int rssi = d.getRSSI();

    if (name == "BCN_A") updateBeacon(&beaconA, rssi);
    else if (name == "BCN_B") updateBeacon(&beaconB, rssi);
    else if (name == "BCN_C") updateBeacon(&beaconC, rssi);
  }
};

// ----------------------
// ESP-NOW Callback
// ----------------------
void onSend(const wifi_tx_info_t *, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "ESP-NOW SENT" : "ESP-NOW FAILED");
}

// ----------------------
// Setup
// ----------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  strcpy(packet.tagID, TAG_ID);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1);
  }

  esp_now_register_send_cb(onSend);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, gatewayMAC, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new ScanCB());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);

  for (int i = 0; i < RSSI_SAMPLES; i++)
    beaconA.rssi_buf[i] = beaconB.rssi_buf[i] = beaconC.rssi_buf[i] = -100;

  Serial.println("MOVING NODE READY");
}

// ----------------------
// Loop
// ----------------------
void loop() {

  // BLE SCAN
  pBLEScan->start(3, false);
  buf_idx++;
  pBLEScan->clearResults();
  pBLEScan->stop();

  int rA = filterRSSI(&beaconA);
  int rB = filterRSSI(&beaconB);
  int rC = filterRSSI(&beaconC);

  Serial.println("----- RSSI VALUES -----");
  Serial.print("A: "); Serial.print(rA);
  Serial.print("  B: "); Serial.print(rB);
  Serial.print("  C: "); Serial.println(rC);

  bool valid = beaconA.detected && beaconB.detected && beaconC.detected;

  if (!valid) {
    Serial.println("Invalid position - Missing:");

    if (!beaconA.detected)
      Serial.println("  BCN_A missing");

    if (!beaconB.detected)
      Serial.println("  BCN_B missing");

    if (!beaconC.detected)
      Serial.println("  BCN_C missing");

    delay(500);
    return;
  }

  float dA = rssiToDistance(rA);
  float dB = rssiToDistance(rB);
  float dC = rssiToDistance(rC);

  Serial.println("----- DISTANCES -----");
  Serial.print("dA: "); Serial.print(dA);
  Serial.print("  dB: "); Serial.print(dB);
  Serial.print("  dC: "); Serial.println(dC);

  float x = (sq(dA) - sq(dB) + sq(D)) / (2 * D);
  float y = (sq(dA) - sq(dC) + sq(D)) / (2 * D);

  // Simple smoothing
  static float prevX = 0, prevY = 0;
  x = 0.7 * prevX + 0.3 * x;
  y = 0.7 * prevY + 0.3 * y;
  prevX = x;
  prevY = y;

  Serial.println("----- POSITION -----");
  Serial.print("X: "); Serial.print(x);
  Serial.print("  Y: "); Serial.println(y);

  packet.x = x;
  packet.y = y;
  packet.ts = millis();

  esp_now_send(gatewayMAC, (uint8_t*)&packet, sizeof(packet));

  delay(500);
}