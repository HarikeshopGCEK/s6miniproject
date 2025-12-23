#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

const char* BEACON_ID = "BEACON_C";  // CHANGE for each beacon (A, B, C, D)

BLEAdvertising *pAdvertising;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=============================");
  Serial.printf("Starting Beacon: %s\n", BEACON_ID);
  Serial.println("=============================");
  
  // Initialize BLE with beacon name
  BLEDevice::init(BEACON_ID);
  
  // Set maximum transmission power for best range and stability
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
  
  Serial.println("✓ TX Power: Maximum (+9 dBm)");
  
  // Create BLE server (required even for beacons)
  BLEServer *pServer = BLEDevice::createServer();
  
  if (pServer == nullptr) {
    Serial.println("❌ ERROR: Failed to create BLE server!");
    Serial.println("   Check board selection and try again");
    while(1) { delay(1000); }  // Stop here
  }
  
  // Configure advertising
  pAdvertising = BLEDevice::getAdvertising();
  
  // Set advertising data with explicit name
  BLEAdvertisementData advertisementData;
  advertisementData.setName(BEACON_ID);
  advertisementData.setFlags(0x06);  // LE General Discoverable, BR/EDR not supported
  
  pAdvertising->setAdvertisementData(advertisementData);
  
  // Disable scan response (beacon mode)
  pAdvertising->setScanResponse(false);
  
  // Set advertising interval
  // Faster = more detectable but more power consumption
  // Values in 0.625ms units
  pAdvertising->setMinInterval(0x20);  // 32 * 0.625ms = 20ms (fast)
  pAdvertising->setMaxInterval(0xA0);  // 160 * 0.625ms = 100ms
  
  Serial.println("✓ Advertising Interval: 20-100ms");
  
  // Legacy compatibility parameters
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);
  
  // Start advertising
  BLEDevice::startAdvertising();
  
  Serial.println("✓ Beacon is now broadcasting!");
  Serial.printf("✓ Device Name: %s\n", BEACON_ID);
  Serial.println("=============================");
  Serial.println("Ready for scanning...\n");
}

void loop() {
  // Heartbeat - shows beacon is alive
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat >= 10000) {  // Every 10 seconds
    lastHeartbeat = millis();
    Serial.printf("[%lu sec] %s active\n", millis()/1000, BEACON_ID);
  }
  
  // Watchdog - restart advertising if it stopped
  if (!pAdvertising->isAdvertising()) {
    Serial.println("⚠ Advertising stopped! Restarting...");
    BLEDevice::startAdvertising();
  }
  
  delay(1000);
}