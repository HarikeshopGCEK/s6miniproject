#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <math.h>

BLEScan *pBLEScan;

// Calibration values
float txPower = -59.0;  // RSSI at 1 meter - CALIBRATE THIS!
float n = 2.5;          // 2.0-2.5 for indoor, 3.0-4.0 for obstacles
float D = 2.0;          // distance between beacons (meters)

// RSSI filtering
#define RSSI_SAMPLES 3
int rssiA_buf[RSSI_SAMPLES], rssiB_buf[RSSI_SAMPLES];
int rssiC_buf[RSSI_SAMPLES], rssiD_buf[RSSI_SAMPLES];
int rssiA = -100, rssiB = -100, rssiC = -100, rssiD = -100;
int buf_idx = 0;

float rssiToDistance(int rssi) {
  if (rssi >= 0 || rssi < -100) return 999.9; // Invalid
  return pow(10.0, (txPower - rssi) / (10.0 * n));
}

int filterRSSI(int newVal, int* buf) {
  buf[buf_idx % RSSI_SAMPLES] = newVal;
  long sum = 0;
  for(int i = 0; i < RSSI_SAMPLES; i++) sum += buf[i];
  return sum / RSSI_SAMPLES;
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice device) {
    String name = device.getName().c_str();
    int rssi = device.getRSSI();

    if (name == "BCN_A") rssiA = rssi;
    else if (name == "BCN_B") rssiB = rssi;
    else if (name == "BCN_B") rssiC = rssi;
    else if (name == "BCN_D") rssiD = rssi;
  }
};

void setup() {
  Serial.begin(115200);
  
  // Initialize buffers
  for(int i = 0; i < RSSI_SAMPLES; i++) {
    rssiA_buf[i] = rssiB_buf[i] = rssiC_buf[i] = rssiD_buf[i] = -100;
  }

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
}

void loop() {
  // Scan
  pBLEScan->start(2, false);
  
  // Filter RSSI
  int fA = filterRSSI(rssiA, rssiA_buf);
  int fB = filterRSSI(rssiB, rssiB_buf);
  int fC = filterRSSI(rssiC, rssiC_buf);
  int fD = filterRSSI(rssiD, rssiD_buf);
  buf_idx++;
  
  pBLEScan->clearResults();

  // Calculate distances
  float dA = rssiToDistance(fA);
  float dB = rssiToDistance(fB);
  float dC = rssiToDistance(fC);
  float dD = rssiToDistance(fD);

  // Trilateration (assumes A at origin, B on X-axis, C on Y-axis)
  float x = (sq(dA) - sq(dB) + sq(D)) / (2.0 * D);
  float y = (sq(dA) - sq(dC) + sq(D)) / (2.0 * D);
  
  // Bounds checking
  x = constrain(x, -D*0.5, D*1.5);
  y = constrain(y, -D*0.5, D*1.5);

  // Display
  Serial.println("\n==== Position ====");
  Serial.printf("RSSI (filtered): A=%d B=%d C=%d D=%d\n", fA, fB, fC, fD);
  Serial.printf("Distance: A=%.2fm B=%.2fm C=%.2fm D=%.2fm\n", dA, dB, dC, dD);
  Serial.printf("Position: X=%.2fm, Y=%.2fm\n", x, y);
  
  // Confidence check
  int beacons_found = (fA > -95) + (fB > -95) + (fC > -95);
  if (beacons_found < 3) {
    Serial.printf("WARNING: Only %d/3 beacons detected!\n", beacons_found);
  }

  delay(500);
}