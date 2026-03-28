#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// Unique identifier for each beacon
// IMPORTANT: Each beacon must have a different name (BCN_A, BCN_B, BCN_C)
const char* BEACON_ID = "BCN_B";

void setup() {
  Serial.begin(115200);

  // Initialize BLE with beacon name
  // This name is what the moving node scans and identifies
  BLEDevice::init(BEACON_ID);

  // Create BLE server (required for advertising)
  BLEServer *pServer = BLEDevice::createServer();

  // Get advertising object
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();

  // Disable scan response (saves power, simpler broadcast)
  pAdvertising->setScanResponse(false);

  // Optimize BLE connection parameters (standard values)
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);

  // Start broadcasting BLE packets
  BLEDevice::startAdvertising();

  Serial.println("Beacon advertising...");
}

void loop() {
  // No processing needed — beacon only transmits
  delay(1000);
}
