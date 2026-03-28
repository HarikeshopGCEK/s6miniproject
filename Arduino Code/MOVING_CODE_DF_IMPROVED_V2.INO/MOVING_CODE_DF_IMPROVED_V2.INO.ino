#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <math.h>

// WiFi channel for ESP-NOW communication
#define WIFI_CHANNEL 1

// Number of RSSI samples for filtering
#define RSSI_SAMPLES 8

// Time after which beacon is considered lost (ms)
#define RSSI_TIMEOUT 5000

// MAC address of gateway (receiver)
uint8_t gatewayMAC[] = {0x3C,0x8A,0x1F,0x9D,0x3F,0x04};

// Unique ID of this tag
const char* TAG_ID = "T3";

// RSSI at 1 meter (calibrated value)
float txPower = -66;

// Path loss exponent (environment dependent)
float n = 3;

// ── BEACON POSITIONS (in meters) ─────────────────────────────
// These must match real-world placement
const float Ax = 0.0, Ay = 0.0;
const float Bx = 3.0, By = 0.0;
const float Cx = 0.0, Cy = 3.0;

// Packet structure sent to gateway
typedef struct {
  char tagID[4];       // Tag identifier
  float x;             // Calculated X coordinate
  float y;             // Calculated Y coordinate
  uint8_t beaconMask;  // Which beacons are active
  uint32_t ts;         // Timestamp
} PositionPacket;

PositionPacket packet;
BLEScan *pBLEScan;

// Structure to store RSSI data for each beacon
struct BeaconData {
  int rssi_buf[RSSI_SAMPLES];  // Circular buffer
  int buf_idx;                 // Buffer index
  unsigned long last_seen;     // Last detection time
  bool detected;               // Beacon detected or not
  bool initialized;
};

// Three beacon instances
BeaconData beaconA, beaconB, beaconC;

// Previous position (for smoothing)
float prevX = 0, prevY = 0;
bool emaInitialized = false;

// Convert RSSI → distance using path loss model
float rssiToDistance(int rssi) {
  if (rssi >= 0 || rssi < -100) return 999; // invalid
  return pow(10.0, (txPower - rssi) / (10.0 * n));
}

// Median filter to remove noise from RSSI
int filterRSSI(BeaconData* b) {

  // If beacon not seen recently → mark as lost
  if (millis() - b->last_seen > RSSI_TIMEOUT) {
    b->detected = false;
    return -100;
  }

  int temp[RSSI_SAMPLES];
  memcpy(temp, b->rssi_buf, sizeof(temp));

  // Sort values (simple bubble sort)
  for (int i = 0; i < RSSI_SAMPLES - 1; i++)
    for (int j = i + 1; j < RSSI_SAMPLES; j++)
      if (temp[i] > temp[j]) {
        int t = temp[i];
        temp[i] = temp[j];
        temp[j] = t;
      }

  // Return median value
  return temp[RSSI_SAMPLES / 2];
}

// Update RSSI buffer for a beacon
void updateBeacon(BeaconData* b, int rssi) {
  b->rssi_buf[b->buf_idx % RSSI_SAMPLES] = rssi;
  b->buf_idx++;
  b->last_seen = millis();
  b->detected = true;
}

// General trilateration (works for any layout)
bool trilaterate(float dA, float dB, float dC, float &x, float &y) {

  // Vector from A → B
  float ex = Bx - Ax, ey = By - Ay;
  float d  = sqrt(ex*ex + ey*ey);
  if (d < 0.01) return false;

  // Normalize
  ex /= d; 
  ey /= d;

  // Vector from A → C
  float fx = Cx - Ax, fy = Cy - Ay;

  // Projection of C onto AB
  float i = ex*fx + ey*fy;

  // Perpendicular component
  float jx = fx - i*ex;
  float jy = fy - i*ey;
  float j  = sqrt(jx*jx + jy*jy);
  if (j < 0.01) return false;

  // Normalize perpendicular
  jx /= j; 
  jy /= j;

  // Solve for position
  float xp = (dA*dA - dB*dB + d*d) / (2*d);
  float yp = (dA*dA - dC*dC + i*i + j*j - 2*i*xp) / (2*j);

  // Convert back to global coordinates
  x = Ax + xp*ex + yp*jx;
  y = Ay + xp*ey + yp*jy;

  return true;
}

// BLE scan callback → receives RSSI
class ScanCB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice d) {
    String name = d.getName().c_str();
    int r = d.getRSSI();

    // Identify beacon by name
    if      (name == "BCN_A") updateBeacon(&beaconA, r);
    else if (name == "BCN_B") updateBeacon(&beaconB, r);
    else if (name == "BCN_C") updateBeacon(&beaconC, r);
  }
};

// ESP-NOW send status
void onSend(const wifi_tx_info_t*, esp_now_send_status_t s) {
  Serial.println(s == ESP_NOW_SEND_SUCCESS ? "SEND OK" : "SEND FAIL");
}

// Initialize beacon buffers
void initBeacon(BeaconData* b) {
  for (int i = 0; i < RSSI_SAMPLES; i++) b->rssi_buf[i] = -100;
  b->buf_idx = 0;
  b->detected = false;
  b->last_seen = 0;
}

void setup() {
  Serial.begin(115200);

  strcpy(packet.tagID, TAG_ID);

  // Setup WiFi for ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  esp_now_init();
  esp_now_register_send_cb(onSend);

  // Register gateway as peer
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, gatewayMAC, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  // Initialize BLE scanning
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new ScanCB());
  pBLEScan->setActiveScan(true);

  // Initialize beacon data
  initBeacon(&beaconA);
  initBeacon(&beaconB);
  initBeacon(&beaconC);

  Serial.println("TAG READY");
}

void loop() {

  // Scan BLE signals
  pBLEScan->start(4, false);
  pBLEScan->clearResults();

  // Filter RSSI
  int rA = filterRSSI(&beaconA);
  int rB = filterRSSI(&beaconB);
  int rC = filterRSSI(&beaconC);

  // Create beacon mask
  uint8_t mask = 0;
  if (beaconA.detected) mask |= 1;
  if (beaconB.detected) mask |= 2;
  if (beaconC.detected) mask |= 4;

  packet.beaconMask = mask;

  // If any beacon missing → send invalid data
  if (mask != 7) {
    packet.x = -1; 
    packet.y = -1;
    packet.ts = millis();
    esp_now_send(gatewayMAC, (uint8_t*)&packet, sizeof(packet));
    return;
  }

  // Convert RSSI → distance
  float dA = constrain(rssiToDistance(rA), 0.1, 15.0);
  float dB = constrain(rssiToDistance(rB), 0.1, 15.0);
  float dC = constrain(rssiToDistance(rC), 0.1, 15.0);

  float x_raw, y_raw;

  // Trilateration
  if (!trilaterate(dA, dB, dC, x_raw, y_raw)) return;

  float x, y;

  // EMA smoothing
  if (!emaInitialized) {
    x = x_raw;
    y = y_raw;
    emaInitialized = true;
  } else {
    x = 0.6*prevX + 0.4*x_raw;
    y = 0.6*prevY + 0.4*y_raw;
  }

  prevX = x;
  prevY = y;

  // Send data
  packet.x = x;
  packet.y = y;
  packet.ts = millis();

  esp_now_send(gatewayMAC, (uint8_t*)&packet, sizeof(packet));

  delay(600);
}
