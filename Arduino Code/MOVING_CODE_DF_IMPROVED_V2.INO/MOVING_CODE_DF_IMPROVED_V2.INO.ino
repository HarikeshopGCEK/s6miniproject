#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <math.h>

#define WIFI_CHANNEL 1
#define RSSI_SAMPLES 8
#define RSSI_TIMEOUT 5000

uint8_t gatewayMAC[] = {0x3C,0x8A,0x1F,0x9D,0x3F,0x04};
const char* TAG_ID = "T3";

float txPower = -66;
float n = 3;  // tune this for your environment

// ── BEACON PHYSICAL POSITIONS ──────────────────────────────────────────────
// Measure these yourself in metres from your chosen origin point.
// Example: right-angle layout where A is origin
const float Ax = 0.0, Ay = 0.0;
const float Bx = 3.0, By = 0.0;   // ← set your real measured distance
const float Cx = 0.0, Cy = 3.0;   // ← set your real measured distance
// ───────────────────────────────────────────────────────────────────────────

typedef struct {
  char tagID[4];
  float x;
  float y;
  uint8_t beaconMask;
  uint32_t ts;
} PositionPacket;

PositionPacket packet;
BLEScan *pBLEScan;

struct BeaconData {
  int rssi_buf[RSSI_SAMPLES];
  int buf_idx;          // ← FIX 3: each beacon has its own index
  unsigned long last_seen;
  bool detected;
  bool initialized;     // ← FIX 2: track first reading
};

BeaconData beaconA, beaconB, beaconC;

float prevX = 0, prevY = 0;
bool emaInitialized = false;   // ← FIX 2

float rssiToDistance(int rssi) {
  if (rssi >= 0 || rssi < -100) return 999;
  return pow(10.0, (txPower - rssi) / (10.0 * n));
}

int filterRSSI(BeaconData* b) {
  if (millis() - b->last_seen > RSSI_TIMEOUT) {
    b->detected = false;
    return -100;
  }
  int temp[RSSI_SAMPLES];
  memcpy(temp, b->rssi_buf, sizeof(temp));
  for (int i = 0; i < RSSI_SAMPLES - 1; i++)
    for (int j = i + 1; j < RSSI_SAMPLES; j++)
      if (temp[i] > temp[j]) { int t = temp[i]; temp[i] = temp[j]; temp[j] = t; }
  return temp[RSSI_SAMPLES / 2];
}

void updateBeacon(BeaconData* b, int rssi) {
  b->rssi_buf[b->buf_idx % RSSI_SAMPLES] = rssi;  // FIX 3: own index
  b->buf_idx++;
  b->last_seen = millis();
  b->detected = true;
}

// ── FIX 1: General trilateration (works for any beacon layout) ─────────────
bool trilaterate(float dA, float dB, float dC, float &x, float &y) {
  // Translate so A is origin
  float ex = Bx - Ax, ey = By - Ay;
  float d  = sqrt(ex*ex + ey*ey);
  if (d < 0.01) return false;
  ex /= d; ey /= d;  // unit vector A→B

  float fx = Cx - Ax, fy = Cy - Ay;
  float i  = ex*fx + ey*fy;
  float jx = fx - i*ex, jy = fy - i*ey;
  float j  = sqrt(jx*jx + jy*jy);
  if (j < 0.01) return false;
  jx /= j; jy /= j;  // unit vector perpendicular

  float xp = (dA*dA - dB*dB + d*d) / (2*d);
  float yp = (dA*dA - dC*dC + i*i + j*j - 2*i*xp) / (2*j);

  // Rotate back to world coordinates
  x = Ax + xp*ex + yp*jx;
  y = Ay + xp*ey + yp*jy;
  return true;
}
// ───────────────────────────────────────────────────────────────────────────

class ScanCB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice d) {
    String name = d.getName().c_str();
    int r = d.getRSSI();
    if      (name == "BCN_A") updateBeacon(&beaconA, r);
    else if (name == "BCN_B") updateBeacon(&beaconB, r);
    else if (name == "BCN_C") updateBeacon(&beaconC, r);
  }
};

void onSend(const wifi_tx_info_t*, esp_now_send_status_t s) {
  Serial.println(s == ESP_NOW_SEND_SUCCESS ? "SEND OK" : "SEND FAIL");
}

void initBeacon(BeaconData* b) {
  for (int i = 0; i < RSSI_SAMPLES; i++) b->rssi_buf[i] = -100;
  b->buf_idx = 0;
  b->detected = false;
  b->last_seen = 0;
}

void setup() {
  Serial.begin(115200);
  strcpy(packet.tagID, TAG_ID);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_now_init();
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

  initBeacon(&beaconA);
  initBeacon(&beaconB);
  initBeacon(&beaconC);

  Serial.println("TAG T1 READY");
}

void loop() {
  pBLEScan->start(4, false);
  pBLEScan->clearResults();

  int rA = filterRSSI(&beaconA);
  int rB = filterRSSI(&beaconB);
  int rC = filterRSSI(&beaconC);

  uint8_t mask = 0;
  if (beaconA.detected) mask |= 1;
  if (beaconB.detected) mask |= 2;
  if (beaconC.detected) mask |= 4;

  packet.beaconMask = mask;

  if (mask != 7) {
    packet.x = -1; packet.y = -1;
    packet.ts = millis();
    esp_now_send(gatewayMAC, (uint8_t*)&packet, sizeof(packet));
    Serial.printf("Beacon missing mask=%d\n", mask);
    delay(600);
    return;
  }

  float dA = constrain(rssiToDistance(rA), 0.1, 15.0);
  float dB = constrain(rssiToDistance(rB), 0.1, 15.0);
  float dC = constrain(rssiToDistance(rC), 0.1, 15.0);

  float x_raw, y_raw;
  if (!trilaterate(dA, dB, dC, x_raw, y_raw)) {
    Serial.println("Trilaterate failed");
    delay(600);
    return;
  }

  float x, y;
  // FIX 2: seed EMA with first real measurement
  if (!emaInitialized) {
    x = x_raw; y = y_raw;
    emaInitialized = true;
  } else {
    x = 0.6*prevX + 0.4*x_raw;
    y = 0.6*prevY + 0.4*y_raw;
  }

  prevX = x; prevY = y;

  packet.x = x; packet.y = y;
  packet.ts = millis();
  esp_now_send(gatewayMAC, (uint8_t*)&packet, sizeof(packet));

  Serial.printf("T1 X %.2f Y %.2f | dA=%.2f dB=%.2f dC=%.2f\n",
                x, y, dA, dB, dC);
  delay(600);
}