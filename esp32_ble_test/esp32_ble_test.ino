// ESP32 BLE test peripheral for the Bioliminal mobile app.
//
// GATT contract (must match lib/core/services/hardware_controller.dart):
//   - Device name contains "Bioliminal"
//   - Service UUID contains "FF01"
//   - Characteristic UUID contains "FF02", NOTIFY property
//   - Notifies 10 bytes per packet; each byte is one EMG channel (0-255).
//     The app divides by 255.0 to get a normalized 0.0-1.0 value.
//
// Flash with Arduino IDE:
//   Board:  "ESP32S3 Dev Module" (or your ESP32 variant)
//   Open Serial Monitor at 115200 to watch connection state.
//
// This sketch synthesizes ten sine-wave channels so the app's live
// EMG charts and signal-status LEDs have data to render end-to-end.

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define DEVICE_NAME         "Bioliminal Garment"
#define SERVICE_UUID        "0000ff01-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID "0000ff02-0000-1000-8000-00805f9b34fb"
#define NUM_CHANNELS        10
#define NOTIFY_PERIOD_MS    50   // 20 Hz — matches the mock data cadence

BLECharacteristic* gChar = nullptr;
bool gConnected = false;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    gConnected = true;
    Serial.println("BLE: central connected");
  }
  void onDisconnect(BLEServer*) override {
    gConnected = false;
    Serial.println("BLE: central disconnected — re-advertising");
    BLEDevice::startAdvertising();
  }
};

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("Bioliminal ESP32 BLE test starting…");

  BLEDevice::init(DEVICE_NAME);
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService* service = server->createService(SERVICE_UUID);
  gChar = service->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  gChar->addDescriptor(new BLE2902());

  uint8_t zeros[NUM_CHANNELS] = {0};
  gChar->setValue(zeros, NUM_CHANNELS);
  service->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.print("BLE: advertising as '");
  Serial.print(DEVICE_NAME);
  Serial.println("'");
}

void loop() {
  static uint32_t tick = 0;

  if (gConnected) {
    uint8_t payload[NUM_CHANNELS];
    float t = tick * 0.05f;
    for (int i = 0; i < NUM_CHANNELS; i++) {
      float base = (sinf(t * 2.0f + i) + 1.0f) * 0.5f;
      float jitter = (random(0, 40) / 255.0f);
      float v = base * 0.7f + jitter;
      if (v < 0.0f) v = 0.0f;
      if (v > 1.0f) v = 1.0f;
      payload[i] = (uint8_t)(v * 255.0f);
    }
    gChar->setValue(payload, NUM_CHANNELS);
    gChar->notify();
    tick++;
  }

  delay(NOTIFY_PERIOD_MS);
}
