// Bioliminal Garment raw-EMG streamer for Phase 1 data capture.
//
// Reads all three MyoWare 2.0 analog outputs (RAW, RECT, ENV) at 2 kHz,
// batches 50 sample-triplets per 25 ms window, emits 308-byte BLE notifies
// on characteristic FF02 at 40 Hz.  Zero onboard signal processing — all
// filtering, rectification, rep segmentation, and fatigue analysis lives
// on the phone.
//
// Wiring (esp32-firmware/README.md revision 2026-04-16):
//   MyoWare RAW  -> GPIO35 (ADC1_CH7)
//   MyoWare RECT -> GPIO32 (ADC1_CH4)
//   MyoWare ENV  -> GPIO34 (ADC1_CH6)
//
// Packet layout (308 bytes total — see notes/data-capture-handshake.md):
//   [0]      seq_num u8 (wraps every 256 packets)
//   [1..4]   t_us_start u32 LE (microseconds of first sample in batch,
//            relative to boot)
//   [5]      channel_count u8 = 3
//   [6]      samples_per_channel u8 = 50
//   [7]      flags u8 (bit0=RAW clipped, bit1=RECT clipped, bit2=ENV clipped)
//   [8..107]   RAW  samples  (50 x u16 LE, 0..4095)
//   [108..207] RECT samples  (50 x u16 LE, 0..4095)
//   [208..307] ENV  samples  (50 x u16 LE, 0..4095)
//
// BLE contract matches the existing app-side expectations except payload
// size: phone parser must accept the 308-byte packet per the handshake,
// not the legacy 10-byte synthetic-sine packet.

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define DEVICE_NAME         "Bioliminal Garment"
#define SERVICE_UUID        "0000ff01-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID "0000ff02-0000-1000-8000-00805f9b34fb"

const int RAW_PIN  = 35;
const int RECT_PIN = 32;
const int ENV_PIN  = 34;

const uint32_t SAMPLE_PERIOD_US   = 500;   // 2 kHz
const uint32_t SAMPLES_PER_PACKET = 50;    // -> 25 ms per packet
const uint32_t RING_SIZE          = 128;   // triplets; must exceed SAMPLES_PER_PACKET

const int HEADER_BYTES = 8;
const int PACKET_BYTES = HEADER_BYTES + SAMPLES_PER_PACKET * 3 * sizeof(uint16_t);  // 308

const uint16_t ADC_MAX = 4095;
const uint16_t ADC_MIN = 0;

struct Triplet {
  uint16_t raw;
  uint16_t rect;
  uint16_t env;
};

Triplet gRing[RING_SIZE];
uint32_t gRingHead = 0;  // next write
uint32_t gRingTail = 0;  // next read
uint8_t  gClipFlags = 0; // accumulated since last packet

BLECharacteristic* gChar = nullptr;
bool gConnected = false;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    gConnected = true;
    Serial.println("BLE: connected");
  }
  void onDisconnect(BLEServer*) override {
    gConnected = false;
    Serial.println("BLE: disconnected, re-advertising");
    BLEDevice::startAdvertising();
  }
};

inline uint32_t ring_count() {
  return (gRingHead - gRingTail + RING_SIZE) % RING_SIZE;
}

inline void sample_once() {
  Triplet t;
  t.raw  = analogRead(RAW_PIN);
  t.rect = analogRead(RECT_PIN);
  t.env  = analogRead(ENV_PIN);

  if (t.raw  == ADC_MAX || t.raw  == ADC_MIN) gClipFlags |= 0x1;
  if (t.rect == ADC_MAX || t.rect == ADC_MIN) gClipFlags |= 0x2;
  if (t.env  == ADC_MAX || t.env  == ADC_MIN) gClipFlags |= 0x4;

  uint32_t next = (gRingHead + 1) % RING_SIZE;
  // If the ring is full we drop the sample rather than stalling BLE —
  // phone can detect this via seq_num gaps on the other end.
  if (next != gRingTail) {
    gRing[gRingHead] = t;
    gRingHead = next;
  }
}

void send_packet() {
  static uint8_t seq = 0;
  static uint8_t packet[PACKET_BYTES];

  // Timestamp of the first sample in this batch, relative to boot.
  // Approximation — sample cadence is 500 µs so this is within one tick.
  uint32_t now = micros();
  uint32_t t_us_start = now - SAMPLES_PER_PACKET * SAMPLE_PERIOD_US;

  packet[0] = seq++;
  packet[1] = (uint8_t)(t_us_start       & 0xFF);
  packet[2] = (uint8_t)((t_us_start >> 8)  & 0xFF);
  packet[3] = (uint8_t)((t_us_start >> 16) & 0xFF);
  packet[4] = (uint8_t)((t_us_start >> 24) & 0xFF);
  packet[5] = 3;
  packet[6] = (uint8_t)SAMPLES_PER_PACKET;
  packet[7] = gClipFlags;
  gClipFlags = 0;

  const uint32_t raw_off  = HEADER_BYTES;
  const uint32_t rect_off = HEADER_BYTES + SAMPLES_PER_PACKET * 2;
  const uint32_t env_off  = HEADER_BYTES + SAMPLES_PER_PACKET * 2 * 2;

  for (uint32_t i = 0; i < SAMPLES_PER_PACKET; i++) {
    Triplet t = gRing[gRingTail];
    gRingTail = (gRingTail + 1) % RING_SIZE;

    const uint32_t o = i * 2;
    packet[raw_off  + o]     = (uint8_t)(t.raw  & 0xFF);
    packet[raw_off  + o + 1] = (uint8_t)(t.raw  >> 8);
    packet[rect_off + o]     = (uint8_t)(t.rect & 0xFF);
    packet[rect_off + o + 1] = (uint8_t)(t.rect >> 8);
    packet[env_off  + o]     = (uint8_t)(t.env  & 0xFF);
    packet[env_off  + o + 1] = (uint8_t)(t.env  >> 8);
  }

  gChar->setValue(packet, PACKET_BYTES);
  gChar->notify();
}

void setup() {
  Serial.begin(115200);
  delay(200);

  analogReadResolution(12);
  pinMode(RAW_PIN, INPUT);
  pinMode(RECT_PIN, INPUT);
  pinMode(ENV_PIN, INPUT);

  BLEDevice::init(DEVICE_NAME);
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService* service = server->createService(SERVICE_UUID);
  gChar = service->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  gChar->addDescriptor(new BLE2902());

  uint8_t zeros[PACKET_BYTES] = {0};
  gChar->setValue(zeros, PACKET_BYTES);
  service->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println();
  Serial.print("RAW+RECT+ENV stream @ 2 kHz, 40 Hz notify, ");
  Serial.print(PACKET_BYTES);
  Serial.println(" B/pkt");
  Serial.print("Advertising as '");
  Serial.print(DEVICE_NAME);
  Serial.println("'");
}

void loop() {
  static uint32_t next_sample_us = 0;

  uint32_t now = micros();
  if ((int32_t)(now - next_sample_us) >= 0) {
    next_sample_us = now + SAMPLE_PERIOD_US;
    sample_once();
  }

  if (gConnected && ring_count() >= SAMPLES_PER_PACKET) {
    send_packet();
  }
}
