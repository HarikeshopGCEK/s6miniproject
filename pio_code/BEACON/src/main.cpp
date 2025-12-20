#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

const char *BEACON_ID = "BEACON_A"; // CHANGE for each beacon

void setup()
{
    Serial.begin(115200);

    BLEDevice::init(BEACON_ID);
    BLEServer *pServer = BLEDevice::createServer();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMaxPreferred(0x12);

    BLEDevice::startAdvertising();
    Serial.println("Beacon advertising...");
}

void loop()
{
    delay(1000);
}
