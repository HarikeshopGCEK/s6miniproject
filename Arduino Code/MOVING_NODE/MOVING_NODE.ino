#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <math.h>

BLEScan *pBLEScan;

// Calibration values
float txPower = -70.0;  // RSSI at 1 meter - Your calibrated value ✓
float n = 2.5;          // Path loss exponent
float D = 2.0;          // Distance between beacons (meters)

// RSSI filtering with timeout
#define RSSI_SAMPLES 5
#define RSSI_TIMEOUT 5000  // 5 seconds - if no update, consider beacon lost

struct BeaconData {
  int rssi_buf[RSSI_SAMPLES];
  int current_rssi;
  unsigned long last_seen;
  bool detected;
};

BeaconData beaconA, beaconB, beaconC, beaconD;
int buf_idx = 0;

float rssiToDistance(int rssi) {
  if (rssi >= 0 || rssi < -100) return 999.9;
  return pow(10.0, (txPower - rssi) / (10.0 * n));
}
c
int filterRSSI(BeaconData* beacon) {
  // If beacon not detected recently, return invalid
  if (millis() - beacon->last_seen > RSSI_TIMEOUT) {
    beacon->detected = false;
    return -100;
  }
  
  // Calculate average
  long sum = 0;
  for(int i = 0; i < RSSI_SAMPLES; i++) {
    sum += beacon->rssi_buf[i];
  }
  return sum / RSSI_SAMPLES;
}

void updateBeacon(BeaconData* beacon, int newRSSI) {
  beacon->rssi_buf[buf_idx % RSSI_SAMPLES] = newRSSI;
  beacon->current_rssi = newRSSI;
  beacon->last_seen = millis();
  beacon->detected = true;
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice device) {
    String name = device.getName().c_str();
    int rssi = device.getRSSI();

    if (name == "BCN_A") updateBeacon(&beaconA, rssi);
    else if (name == "BCN_B") updateBeacon(&beaconB, rssi);
    else if (name == "BCN_C") updateBeacon(&beaconC, rssi);
    else if (name == "BCN_D") updateBeacon(&beaconD, rssi);
  }
};

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n================================");
  Serial.println("  BLE POSITIONING SCANNER");
  Serial.println("================================");
  Serial.printf("Calibration: txPower=%.1f dBm\n", txPower);
  Serial.printf("Path Loss: n=%.1f\n", n);
  Serial.printf("Beacon Distance: D=%.1f m\n", D);
  Serial.println("================================\n");
  
  // Initialize all beacon buffers with -100
  for(int i = 0; i < RSSI_SAMPLES; i++) {
    beaconA.rssi_buf[i] = -100;
    beaconB.rssi_buf[i] = -100;
    beaconC.rssi_buf[i] = -100;
    beaconD.rssi_buf[i] = -100;
  }
  
  // Initialize timestamps to force immediate "not detected" state
  beaconA.last_seen = beaconB.last_seen = 0;
  beaconC.last_seen = beaconD.last_seen = 0;
  beaconA.detected = beaconB.detected = false;
  beaconC.detected = beaconD.detected = false;

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
  
  Serial.println("✓ Scanner initialized");
  Serial.println("✓ Scanning for beacons...\n");
  
  // Pre-fill buffers with initial scan (solves slow startup)
  Serial.println("Initial calibration scan (6 seconds)...");
  for(int i = 0; i < 3; i++) {
    pBLEScan->start(2, false);
    buf_idx++;
    pBLEScan->clearResults();
  }
  Serial.println("✓ Calibration complete!\n");
}

void loop() {
  // Scan for beacons
  pBLEScan->start(2, false);
  
  // Get filtered RSSI values
  int fA = filterRSSI(&beaconA);
  int fB = filterRSSI(&beaconB);
  int fC = filterRSSI(&beaconC);
  int fD = filterRSSI(&beaconD);
  
  buf_idx++;
  pBLEScan->clearResults();

  // Calculate distances
  float dA = rssiToDistance(fA);
  float dB = rssiToDistance(fB);
  float dC = rssiToDistance(fC);
  float dD = rssiToDistance(fD);

  // Count detected beacons
  int beacons_found = beaconA.detected + beaconB.detected + beaconC.detected + beaconD.detected;
  
  // Display
  Serial.println("\n========================================");
  Serial.println("         BEACON STATUS");
  Serial.println("========================================");
  
  // Show each beacon with status indicator
  Serial.printf("BCN_A: %4d dBm | %.2fm | %s\n", 
                fA, dA, beaconA.detected ? "✓ ACTIVE" : "✗ LOST");
  Serial.printf("BCN_B: %4d dBm | %.2fm | %s\n", 
                fB, dB, beaconB.detected ? "✓ ACTIVE" : "✗ LOST");
  Serial.printf("BCN_C: %4d dBm | %.2fm | %s\n", 
                fC, dC, beaconC.detected ? "✓ ACTIVE" : "✗ LOST");
  Serial.printf("BCN_D: %4d dBm | %.2fm | %s\n", 
                fD, dD, beaconD.detected ? "✓ ACTIVE" : "✗ LOST");
  
  Serial.println("----------------------------------------");
  Serial.printf("Beacons detected: %d/4\n", beacons_found);
  
  // Calculate position only if we have enough beacons
  if (beaconA.detected && beaconB.detected && beaconC.detected) {
    float x = (sq(dA) - sq(dB) + sq(D)) / (2.0 * D);
    float y = (sq(dA) - sq(dC) + sq(D)) / (2.0 * D);
    
    x = constrain(x, -D*0.5, D*1.5);
    y = constrain(y, -D*0.5, D*1.5);
    
    Serial.println("\n          POSITION");
    Serial.println("========================================");
    Serial.printf("X: %.2f m\n", x);
    Serial.printf("Y: %.2f m\n", y);
    Serial.println("Status: VALID ✓");
  } else {
    Serial.println("\n          POSITION");
    Serial.println("========================================");
    Serial.println("Status: INVALID ✗");
    Serial.printf("Need A, B, C active. Currently: %d/4 beacons\n", beacons_found);
  }
  
  Serial.println("========================================\n");
  
  delay(500);
}