#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <math.h>

BLEScan *pBLEScan;

// Calibration values
float txPower = -59.0; // RSSI at 1 meter (calibrate once)
float n = 2.0;         // environment factor
float D = 2.0;         // distance between beacons (meters)

// RSSI storage
int rssiA = -100, rssiB = -100, rssiC = -100, rssiD = -100;

// Convert RSSI to distance
float rssiToDistance(int rssi)
{
  return pow(10.0, (txPower - rssi) / (10.0 * n));
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
  void onResult(BLEAdvertisedDevice device)
  {
    String name = device.getName().c_str();
    int rssi = device.getRSSI();

    if (name == "BEACON_A")
      rssiA = rssi;
    else if (name == "BEACON_B")
      rssiB = rssi;
    else if (name == "BEACON_C")
      rssiC = rssi;
    else if (name == "BEACON_D")
      rssiD = rssi;
  }
};

void setup()
{
  Serial.begin(115200);

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
}

void loop()
{
  rssiA = rssiB = rssiC = rssiD = -100;

  pBLEScan->start(2, false); // scan for 2 seconds
  pBLEScan->clearResults();

  // Use only A, B, C for trilateration
  float dA = rssiToDistance(rssiA);
  float dB = rssiToDistance(rssiB);
  float dC = rssiToDistance(rssiC);
  float dD = rssiToDistance(rssiD);

  float x = (sq(dA) - sq(dB) + sq(D)) / (2 * D);
  float y = (sq(dA) - sq(dC) + sq(D)) / (2 * D);

  Serial.println("---- Position ----");
  Serial.print("RSSI A: ");
  Serial.println(rssiA);
  Serial.print("RSSI B: ");
  Serial.println(rssiB);
  Serial.print("RSSI C: ");
  Serial.println(rssiC);
  Serial.print("RSSI D: ");
  Serial.println(rssiD);
  Serial.print("X (m): ");
  Serial.println(x, 2);
  Serial.print("Y (m): ");
  Serial.println(y, 2);

  delay(1000);
}
