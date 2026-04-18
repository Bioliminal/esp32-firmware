// Bioliminal Garment realtime firmware for Phase 2 haptic cueing.
//
// This is the BLE-paired production sketch — pair with the Bioliminal
// mobile app.  For hardware-only bench testing without the app (or to
// validate the pulse scheduler before BLE is involved), flash
// bicep_hardware_test/bicep_hardware_test.ino instead — it exercises the
// exact same cue payloads but via BOOT-button and Serial commands.
//
// Combines three jobs into one sketch:
//   1. Raw 3-channel EMG streaming (RAW / RECT / ENV) at 2 kHz, 308 B packets
//      on BLE characteristic FF02 at 40 Hz — unchanged from bicep_raw_stream.
//   2. FF04 command characteristic that accepts opcodes from the phone to
//      drive the haptic actuator and session-state indicator.
//   3. A pulse-burst scheduler that executes PULSE_BURST commands on the
//      motor(s) via LEDC PWM without blocking the sample loop.
//
// Wiring (esp32-firmware/README.md):
//   MyoWare RAW  -> GPIO35 (ADC1_CH7)
//   MyoWare RECT -> GPIO32 (ADC1_CH4)
//   MyoWare ENV  -> GPIO34 (ADC1_CH6)
//   Motor 0 PWM  -> GPIO25 (2 kHz LEDC, 8-bit duty via 2N7000)
//   Motors 1..3  -> NOT WIRED (reserved for future expansion; writes silently ignored)
//
// BLE opcodes on FF04 (phone -> firmware):
//   0x10 PULSE_BURST       [motor u8][duty u8][n u8][on_ms u16 LE][off_ms u16 LE]
//   0x11 STOP_HAPTIC       [motor u8]
//   0x12 SET_SESSION_STATE [state u8]   (0=Idle 1=Calibrating 2=Active, Serial logged)
//
// FF02 packet layout (unchanged from Phase 1):
//   [0]      seq_num u8
//   [1..4]   t_us_start u32 LE
//   [5]      channel_count u8 = 3
//   [6]      samples_per_channel u8 = 50
//   [7]      flags u8 (bit0 RAW clip, bit1 RECT clip, bit2 ENV clip)
//   [8..107]   RAW  samples  (50 x u16 LE)
//   [108..207] RECT samples  (50 x u16 LE)
//   [208..307] ENV  samples  (50 x u16 LE)

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define DEVICE_NAME           "Bioliminal Garment"
#define SERVICE_UUID          "0000ff01-0000-1000-8000-00805f9b34fb"
#define STREAM_CHAR_UUID      "0000ff02-0000-1000-8000-00805f9b34fb"
#define COMMAND_CHAR_UUID     "0000ff04-0000-1000-8000-00805f9b34fb"

// --- ADC / sampling ---
const int RAW_PIN  = 35;
const int RECT_PIN = 32;
const int ENV_PIN  = 34;

const uint32_t SAMPLE_PERIOD_US   = 500;    // 2 kHz
const uint32_t SAMPLES_PER_PACKET = 50;     // -> 25 ms / 40 Hz notify
const uint32_t RING_SIZE          = 128;

const int HEADER_BYTES = 8;
const int PACKET_BYTES = HEADER_BYTES + SAMPLES_PER_PACKET * 3 * sizeof(uint16_t);  // 308

const uint16_t ADC_MAX = 4095;
const uint16_t ADC_MIN = 0;

// --- Haptic motors ---
const int MOTOR_COUNT = 4;                     // protocol allows up to 4; only motor 0 wired
const int MOTOR_PINS[MOTOR_COUNT] = {25, -1, -1, -1};
const int MOTOR_LEDC_FREQ_HZ   = 2000;
const int MOTOR_LEDC_RES_BITS  = 8;

// --- Session state (informational) ---
enum SessionState : uint8_t { Idle = 0, Calibrating = 1, Active = 2 };
SessionState gSessionState = Idle;

// --- Opcodes ---
static const uint8_t OP_PULSE_BURST       = 0x10;
static const uint8_t OP_STOP_HAPTIC       = 0x11;
static const uint8_t OP_SET_SESSION_STATE = 0x12;

// ---------------------------------------------------------------------------
// Sample ring buffer
// ---------------------------------------------------------------------------

struct Triplet {
  uint16_t raw;
  uint16_t rect;
  uint16_t env;
};

Triplet gRing[RING_SIZE];
uint32_t gRingHead = 0;
uint32_t gRingTail = 0;
uint8_t  gClipFlags = 0;

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
  if (next != gRingTail) {
    gRing[gRingHead] = t;
    gRingHead = next;
  }
  // Overrun: drop sample silently; phone detects via seq_num gaps.
}

// ---------------------------------------------------------------------------
// Pulse-burst scheduler (one per motor)
// ---------------------------------------------------------------------------

struct PulseBurst {
  bool     active;
  uint8_t  duty;            // PWM duty while a pulse is ON
  uint8_t  pulses_remaining;
  uint32_t next_event_ms;   // millis() at which the current pulse ends or the next starts
  bool     currently_on;
  uint16_t on_ms;
  uint16_t off_ms;
};

PulseBurst gBurst[MOTOR_COUNT] = {};

inline void motor_write(int idx, uint8_t duty) {
  if (idx < 0 || idx >= MOTOR_COUNT) return;
  int pin = MOTOR_PINS[idx];
  if (pin < 0) return;
  ledcWrite(pin, duty);
}

void start_pulse_burst(uint8_t motor_idx, uint8_t duty, uint8_t n,
                       uint16_t on_ms, uint16_t off_ms) {
  if (motor_idx >= MOTOR_COUNT) return;
  if (MOTOR_PINS[motor_idx] < 0) {
    Serial.print("cmd: PULSE_BURST ignored, motor ");
    Serial.print(motor_idx);
    Serial.println(" not wired");
    return;
  }
  if (n == 0) return;

  PulseBurst& b = gBurst[motor_idx];
  b.active = true;
  b.duty = duty;
  b.pulses_remaining = n;
  b.on_ms = on_ms;
  b.off_ms = off_ms;
  b.currently_on = true;
  b.next_event_ms = millis() + on_ms;
  motor_write(motor_idx, duty);

  Serial.print("cmd: PULSE_BURST motor=");
  Serial.print(motor_idx);
  Serial.print(" duty=");
  Serial.print(duty);
  Serial.print(" n=");
  Serial.print(n);
  Serial.print(" on=");
  Serial.print(on_ms);
  Serial.print(" off=");
  Serial.println(off_ms);
}

void stop_pulse_burst(uint8_t motor_idx) {
  if (motor_idx >= MOTOR_COUNT) return;
  gBurst[motor_idx].active = false;
  gBurst[motor_idx].pulses_remaining = 0;
  motor_write(motor_idx, 0);
  Serial.print("cmd: STOP_HAPTIC motor=");
  Serial.println(motor_idx);
}

void tick_pulse_schedulers() {
  const uint32_t now = millis();
  for (int i = 0; i < MOTOR_COUNT; i++) {
    PulseBurst& b = gBurst[i];
    if (!b.active) continue;
    if ((int32_t)(now - b.next_event_ms) < 0) continue;

    if (b.currently_on) {
      // End of the ON phase. Turn off, schedule off_ms gap unless this was the last pulse.
      motor_write(i, 0);
      if (b.pulses_remaining <= 1) {
        b.active = false;
        b.pulses_remaining = 0;
      } else {
        b.currently_on = false;
        b.next_event_ms = now + b.off_ms;
      }
    } else {
      // End of the OFF phase. Start the next pulse.
      b.pulses_remaining--;
      b.currently_on = true;
      motor_write(i, b.duty);
      b.next_event_ms = now + b.on_ms;
    }
  }
}

// ---------------------------------------------------------------------------
// Command parsing
// ---------------------------------------------------------------------------

void handle_command(const uint8_t* data, size_t len) {
  if (len == 0) return;
  uint8_t op = data[0];

  switch (op) {
    case OP_PULSE_BURST: {
      // [opcode][motor u8][duty u8][n u8][on_ms u16 LE][off_ms u16 LE] = 8 bytes
      if (len < 8) { Serial.println("cmd: PULSE_BURST too short"); return; }
      uint8_t motor = data[1];
      uint8_t duty  = data[2];
      uint8_t n     = data[3];
      uint16_t on   = (uint16_t)data[4] | ((uint16_t)data[5] << 8);
      uint16_t off  = (uint16_t)data[6] | ((uint16_t)data[7] << 8);
      start_pulse_burst(motor, duty, n, on, off);
      break;
    }
    case OP_STOP_HAPTIC: {
      if (len < 2) { Serial.println("cmd: STOP_HAPTIC too short"); return; }
      stop_pulse_burst(data[1]);
      break;
    }
    case OP_SET_SESSION_STATE: {
      if (len < 2) { Serial.println("cmd: SET_SESSION_STATE too short"); return; }
      uint8_t s = data[1];
      if (s <= (uint8_t)Active) {
        gSessionState = (SessionState)s;
        Serial.print("cmd: SET_SESSION_STATE -> ");
        Serial.println(s == Idle ? "Idle" : s == Calibrating ? "Calibrating" : "Active");
      } else {
        Serial.print("cmd: SET_SESSION_STATE invalid state=");
        Serial.println(s);
      }
      break;
    }
    default: {
      Serial.print("cmd: unknown opcode 0x");
      Serial.println(op, HEX);
      break;
    }
  }
}

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* ch) override {
    String v = ch->getValue();
    handle_command((const uint8_t*)v.c_str(), v.length());
  }
};

// ---------------------------------------------------------------------------
// BLE streaming
// ---------------------------------------------------------------------------

BLECharacteristic* gStreamChar = nullptr;
BLECharacteristic* gCmdChar    = nullptr;
bool gConnected = false;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    gConnected = true;
    Serial.println("BLE: connected");
  }
  void onDisconnect(BLEServer*) override {
    gConnected = false;
    Serial.println("BLE: disconnected, re-advertising");
    // Safety: stop any running haptic so a dropped phone doesn't leave the motor on.
    for (int i = 0; i < MOTOR_COUNT; i++) stop_pulse_burst(i);
    BLEDevice::startAdvertising();
  }
};

void send_packet() {
  static uint8_t seq = 0;
  static uint8_t packet[PACKET_BYTES];

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

  gStreamChar->setValue(packet, PACKET_BYTES);
  gStreamChar->notify();
}

// ---------------------------------------------------------------------------
// Arduino setup / loop
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(200);

  analogReadResolution(12);
  pinMode(RAW_PIN, INPUT);
  pinMode(RECT_PIN, INPUT);
  pinMode(ENV_PIN, INPUT);

  for (int i = 0; i < MOTOR_COUNT; i++) {
    int pin = MOTOR_PINS[i];
    if (pin >= 0) {
      ledcAttach(pin, MOTOR_LEDC_FREQ_HZ, MOTOR_LEDC_RES_BITS);
      ledcWrite(pin, 0);
    }
  }

  BLEDevice::init(DEVICE_NAME);
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService* service = server->createService(SERVICE_UUID);

  gStreamChar = service->createCharacteristic(
      STREAM_CHAR_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  gStreamChar->addDescriptor(new BLE2902());
  uint8_t zeros[PACKET_BYTES] = {0};
  gStreamChar->setValue(zeros, PACKET_BYTES);

  gCmdChar = service->createCharacteristic(
      COMMAND_CHAR_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  gCmdChar->setCallbacks(new CommandCallbacks());

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
  Serial.println("FF04 command char ready");
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

  tick_pulse_schedulers();

  if (gConnected && ring_count() >= SAMPLES_PER_PACKET) {
    send_packet();
  }
}
