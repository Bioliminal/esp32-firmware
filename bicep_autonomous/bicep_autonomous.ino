// Bioliminal Garment — autonomous (hardware-led) firmware.
//
// Pair with the Flutter app running the `feat/hardware-led-read-only` branch.
// The phone writes SET_SESSION_STATE(Calibrating) on start and
// SET_SESSION_STATE(Idle) on end — nothing else.  Firmware self-drives the
// session internals:
//   • samples 3-channel EMG at 2 kHz, streams 310-byte FF02 packets at 40 Hz
//   • tracks a calibration peak during the first 5 s of Calibrating state
//   • auto-transitions Calibrating -> Active after 5 s (phone is never asked)
//   • detects reps locally from the ENV channel and publishes rep_count
//   • computes rolling fatigue percent and fires haptic cues autonomously
//   • publishes a "cue fired" flag in every packet immediately after firing
//
// Packet layout (NEW — 310 bytes, 10-byte header):
//   [0]        seq_num u8
//   [1..4]     t_us_start u32 LE
//   [5]        channel_count u8 = 3
//   [6]        samples_per_channel u8 = 50
//   [7]        flags u8 (bit0 RAW clip, bit1 RECT clip, bit2 ENV clip)
//   [8]        rep_count u8          <-- NEW (monotonic per session)
//   [9]        cue_event u8          <-- NEW (bit0 = cue fired since last pkt)
//   [10..109]  RAW  samples (50 x u16 LE)   <-- offset shifted +2
//   [110..209] RECT samples (50 x u16 LE)
//   [210..309] ENV  samples (50 x u16 LE)
//
// FF04 opcodes (phone -> firmware):
//   0x12 SET_SESSION_STATE [state u8]   (0=Idle, 1=Calibrating; Active is
//                                        reached internally — phone does not
//                                        write state=2).
//
// Serial rows (capture_ble_full.py parses [rx]/[evt]/[rep]/[cue]):
//   [rx] t_ms=... len=... op=0x12 bytes=...
//   [evt] t_ms=... kind=(connect|disconnect|session_state)[ state=...]
//   [rep] t_ms=... rep_count=...
//   [cue] t_ms=... fatigue_pct=... duty=... n=... on_ms=... off_ms=...
//
// Wiring (unchanged from bicep_realtime.ino):
//   MyoWare RAW  -> GPIO35 (ADC1_CH7)
//   MyoWare RECT -> GPIO32 (ADC1_CH4)
//   MyoWare ENV  -> GPIO34 (ADC1_CH6)
//   Motor 0 PWM  -> GPIO25 (2 kHz LEDC, 8-bit duty via 2N7000)

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

const int HEADER_BYTES = 10;                // UP FROM 8 (rep_count + cue_event)
const int PACKET_BYTES = HEADER_BYTES + SAMPLES_PER_PACKET * 3 * sizeof(uint16_t);  // 310

const uint16_t ADC_MAX = 4095;
const uint16_t ADC_MIN = 0;

// --- Haptic motors ---
const int MOTOR_COUNT = 4;                     // protocol allows 4; only motor 0 wired
const int MOTOR_PINS[MOTOR_COUNT] = {25, -1, -1, -1};
const int MOTOR_LEDC_FREQ_HZ   = 2000;
const int MOTOR_LEDC_RES_BITS  = 8;

// --- Session state (phone -> Calibrating/Idle, firmware -> Active internally) ---
enum SessionState : uint8_t { Idle = 0, Calibrating = 1, Active = 2 };
SessionState gSessionState = Idle;
uint32_t gCalibratingStartMs = 0;
const uint32_t CAL_DURATION_MS = 5000;

// --- Opcodes (only SET_SESSION_STATE survives in this sketch) ---
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
  // Overrun: drop sample; phone detects via seq_num gaps.
}

// ---------------------------------------------------------------------------
// Envelope rolling mean (50-sample window, updated as samples land in ring)
// Backed by a small mirror ring because the main ring gets drained on every
// packet send and we need continuity across packets.
// ---------------------------------------------------------------------------

const int ENV_WINDOW = 50;      // ~25 ms at 2 kHz
uint16_t gEnvRing[ENV_WINDOW] = {0};
uint32_t gEnvIdx = 0;
uint64_t gEnvSum = 0;
uint32_t gEnvFilled = 0;

inline void env_push(uint16_t v) {
  gEnvSum -= gEnvRing[gEnvIdx];
  gEnvRing[gEnvIdx] = v;
  gEnvSum += v;
  gEnvIdx = (gEnvIdx + 1) % ENV_WINDOW;
  if (gEnvFilled < ENV_WINDOW) gEnvFilled++;
}

inline float env_avg() {
  if (gEnvFilled == 0) return 0.0f;
  return (float)gEnvSum / (float)gEnvFilled;
}

// ---------------------------------------------------------------------------
// Calibration state. gCalibrationPeak is set during the 5 s Calibrating
// window by tracking the envelope rolling-average max. It anchors the rep
// detector's rise threshold. Fatigue scoring is done differently here than
// in emg_fatigue.ino — see evaluate_fatigue_cue() below, which works from
// per-rep peak history rather than instantaneous rolling-average drop.
// ---------------------------------------------------------------------------

float gCalibrationPeak = 0.0f;

// ---------------------------------------------------------------------------
// Rep detection — count at envelope peak (contraction apex), not at the end
// of the eccentric release. Users subjectively count reps "as they squeeze,"
// so firing the counter when avg has just turned over from a local max feels
// synchronous with the motion.
//
// States:
//   RepQuiet   — envelope below rise threshold (arm extended)
//   RepRising  — envelope has crossed rise threshold; tracking running max
//   RepCounted — peak detected (avg dropped ≥ 20% from running max); rep has
//                been counted and we're just waiting for envelope to clear
//                back below rise threshold so the next rep can start
//
// Per-rep peak envelope is recorded in gRepPeakHistory at the instant of the
// count so downstream fatigue logic can compare rep N's peak against the
// first N-1 peaks without re-walking the sample buffer.
// ---------------------------------------------------------------------------

enum RepPhase : uint8_t { RepQuiet = 0, RepRising = 1, RepCounted = 2 };
RepPhase gRepPhase = RepQuiet;
uint8_t  gRepCount = 0;
uint32_t gLastRepMs = 0;
const uint32_t REP_COOLDOWN_MS = 250;
// Rep boundary detection. rise_thresh kicks the machine into "rising" on
// the concentric phase; peak is declared when avg drops ≥ 20% from the
// rising-phase max.
const float REP_RISE_FRAC = 0.30f;
const float REP_PEAK_DROP_FRAC = 0.80f;    // avg < this × running_max = peak

float gCurrentRepPeak = 0.0f;
const int REP_PEAK_HISTORY = 16;
float gRepPeakHistory[REP_PEAK_HISTORY] = {0};

// Fatigue cue evaluation — called once per rep boundary, not on a periodic
// tick, so cues never fire during the quiet phase between reps. First cue is
// gated to gRepCount ≥ CUE_START_REP, i.e. reps 1..CUE_START_REP are treated
// as calibration reps on the firmware side.
const uint8_t CUE_START_REP = 5;           // first cue possible at rep 6

uint8_t gLastCueRep = 0;
const uint8_t CUE_REP_COOLDOWN = 2;
bool gCueEventFired = false;               // set when a burst starts
void evaluate_fatigue_cue(uint32_t now_ms);

void rep_detector_tick(uint32_t now_ms) {
  if (gSessionState != Active) return;
  if (gCalibrationPeak <= 1.0f) return;

  float avg = env_avg();
  float rise_thresh = REP_RISE_FRAC * gCalibrationPeak;

  switch (gRepPhase) {
    case RepQuiet:
      if (avg >= rise_thresh) {
        gRepPhase = RepRising;
        gCurrentRepPeak = avg;
      }
      break;
    case RepRising:
      if (avg > gCurrentRepPeak) {
        gCurrentRepPeak = avg;
      }
      // Declare peak when avg drops meaningfully from the running max AND
      // the running max is comfortably above the rise threshold (avoids
      // false positives from envelope noise that barely clears rise_thresh).
      if (gCurrentRepPeak > 1.5f * rise_thresh &&
          avg < REP_PEAK_DROP_FRAC * gCurrentRepPeak) {
        if (now_ms - gLastRepMs >= REP_COOLDOWN_MS) {
          gRepCount++;
          gLastRepMs = now_ms;
          // Store this rep's peak into the history ring. Keyed by gRepCount
          // so the most recent rep (index gRepCount - 1) is what fatigue
          // evaluation reads.
          gRepPeakHistory[(gRepCount - 1) % REP_PEAK_HISTORY] = gCurrentRepPeak;
          Serial.print("[rep] t_ms=");
          Serial.print(now_ms);
          Serial.print(" rep_count=");
          Serial.print(gRepCount);
          Serial.print(" peak=");
          Serial.println(gCurrentRepPeak, 1);
          evaluate_fatigue_cue(now_ms);
        }
        gRepPhase = RepCounted;
      }
      break;
    case RepCounted:
      if (avg < rise_thresh) {
        gRepPhase = RepQuiet;
      }
      break;
  }
}

// ---------------------------------------------------------------------------
// Pulse-burst scheduler (unchanged from bicep_realtime.ino)
// ---------------------------------------------------------------------------

struct PulseBurst {
  bool     active;
  uint8_t  duty;
  uint8_t  pulses_remaining;
  uint32_t next_event_ms;
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
  if (MOTOR_PINS[motor_idx] < 0) return;
  if (gSessionState != Active) return;          // Gate: only fire during Active.
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
  gCueEventFired = true;
}

void stop_pulse_burst(uint8_t motor_idx) {
  if (motor_idx >= MOTOR_COUNT) return;
  gBurst[motor_idx].active = false;
  gBurst[motor_idx].pulses_remaining = 0;
  motor_write(motor_idx, 0);
}

void tick_pulse_schedulers() {
  const uint32_t now = millis();
  for (int i = 0; i < MOTOR_COUNT; i++) {
    PulseBurst& b = gBurst[i];
    if (!b.active) continue;
    if ((int32_t)(now - b.next_event_ms) < 0) continue;

    if (b.currently_on) {
      motor_write(i, 0);
      if (b.pulses_remaining <= 1) {
        b.active = false;
        b.pulses_remaining = 0;
      } else {
        b.currently_on = false;
        b.next_event_ms = now + b.off_ms;
      }
    } else {
      b.pulses_remaining--;
      b.currently_on = true;
      motor_write(i, b.duty);
      b.next_event_ms = now + b.on_ms;
    }
  }
}

// ---------------------------------------------------------------------------
// Fatigue cue evaluation — called once per rep boundary from
// rep_detector_tick(). Compares the most recent rep's peak envelope against
// the baseline (mean of the first N cal-phase rep peaks). No periodic tick,
// so cues never fire during the quiet phase between reps.
// ---------------------------------------------------------------------------

void evaluate_fatigue_cue(uint32_t now_ms) {
  // Gate: reps 1..CUE_START_REP act as calibration peaks. First cue possible
  // starting at rep CUE_START_REP + 1 (so reps 1-5 collect baseline and the
  // earliest cue fires at rep 6).
  if (gRepCount <= CUE_START_REP) return;
  if (gRepCount - gLastCueRep < CUE_REP_COOLDOWN) return;

  // Baseline = mean of the first CUE_START_REP peaks observed this session.
  float baseline = 0.0f;
  int baseline_n = 0;
  for (int i = 0; i < CUE_START_REP && i < REP_PEAK_HISTORY; i++) {
    if (gRepPeakHistory[i] > 1.0f) {
      baseline += gRepPeakHistory[i];
      baseline_n++;
    }
  }
  if (baseline_n == 0) return;
  baseline /= baseline_n;

  float current = gRepPeakHistory[(gRepCount - 1) % REP_PEAK_HISTORY];
  if (current >= baseline) return;   // no drop, nothing to cue

  int pct = (int)((baseline - current) / baseline * 100.0f);

  uint8_t duty = 0, n = 0;
  uint16_t on_ms = 0, off_ms = 0;
  if (pct < 15) {
    return;                          // below fade threshold
  } else if (pct < 25) {              // FADE
    duty = 180; n = 2; on_ms = 200; off_ms = 150;
  } else if (pct < 50) {              // URGENT
    duty = 230; n = 2; on_ms = 200; off_ms = 150;
  } else {                            // STOP-like
    duty = 255; n = 3; on_ms = 100; off_ms = 80;
  }

  start_pulse_burst(0, duty, n, on_ms, off_ms);
  gLastCueRep = gRepCount;

  Serial.print("[cue] t_ms=");
  Serial.print(now_ms);
  Serial.print(" rep=");
  Serial.print(gRepCount);
  Serial.print(" peak=");
  Serial.print(current, 1);
  Serial.print(" baseline=");
  Serial.print(baseline, 1);
  Serial.print(" drop_pct=");
  Serial.print(pct);
  Serial.print(" duty=");
  Serial.print(duty);
  Serial.print(" n=");
  Serial.print(n);
  Serial.print(" on_ms=");
  Serial.print(on_ms);
  Serial.print(" off_ms=");
  Serial.println(off_ms);
}

// ---------------------------------------------------------------------------
// Command parsing — only SET_SESSION_STATE survives.
// ---------------------------------------------------------------------------

static void log_rx(const uint8_t* data, size_t len) {
  Serial.print("[rx] t_ms=");
  Serial.print(millis());
  Serial.print(" len=");
  Serial.print((uint32_t)len);
  if (len >= 1) {
    Serial.print(" op=0x");
    if (data[0] < 0x10) Serial.print('0');
    Serial.print(data[0], HEX);
  }
  Serial.print(" bytes=");
  for (size_t i = 0; i < len; i++) {
    if (i > 0) Serial.print(' ');
    if (data[i] < 0x10) Serial.print('0');
    Serial.print(data[i], HEX);
  }
  Serial.println();
}

static void log_evt(const char* kind) {
  Serial.print("[evt] t_ms=");
  Serial.print(millis());
  Serial.print(" kind=");
  Serial.println(kind);
}

static void log_session_state(const char* name) {
  Serial.print("[evt] t_ms=");
  Serial.print(millis());
  Serial.print(" kind=session_state state=");
  Serial.println(name);
}

void enter_idle() {
  gSessionState = Idle;
  for (int i = 0; i < MOTOR_COUNT; i++) stop_pulse_burst(i);
  gRingHead = 0;
  gRingTail = 0;
  gClipFlags = 0;
  gRepPhase = RepQuiet;
  gRepCount = 0;
  gLastCueRep = 0;
  gCueEventFired = false;
  gCurrentRepPeak = 0.0f;
  for (int i = 0; i < REP_PEAK_HISTORY; i++) gRepPeakHistory[i] = 0.0f;
  gCalibrationPeak = 0.0f;
  gEnvIdx = 0;
  gEnvSum = 0;
  gEnvFilled = 0;
  for (int i = 0; i < ENV_WINDOW; i++) gEnvRing[i] = 0;
  log_session_state("Idle");
}

void enter_calibrating() {
  gSessionState = Calibrating;
  gCalibratingStartMs = millis();
  gCalibrationPeak = 0.0f;
  gRepPhase = RepQuiet;
  gRepCount = 0;
  gLastCueRep = 0;
  gCurrentRepPeak = 0.0f;
  for (int i = 0; i < REP_PEAK_HISTORY; i++) gRepPeakHistory[i] = 0.0f;
  log_session_state("Calibrating");
}

void enter_active() {
  gSessionState = Active;
  log_session_state("Active");
}

void handle_command(const uint8_t* data, size_t len) {
  log_rx(data, len);

  if (len == 0) return;
  uint8_t op = data[0];

  if (op != OP_SET_SESSION_STATE) {
    Serial.print("cmd: opcode 0x");
    Serial.print(op, HEX);
    Serial.println(" ignored (this sketch only handles SET_SESSION_STATE)");
    return;
  }

  if (len < 2) {
    Serial.println("cmd: SET_SESSION_STATE too short");
    return;
  }
  uint8_t s = data[1];
  if (s == Idle) {
    enter_idle();
  } else if (s == Calibrating) {
    enter_calibrating();
  } else if (s == Active) {
    // Phone is not supposed to write Active in this sketch, but accept it.
    enter_active();
  } else {
    Serial.print("cmd: SET_SESSION_STATE invalid state=");
    Serial.println(s);
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
    log_evt("connect");
  }
  void onDisconnect(BLEServer*) override {
    gConnected = false;
    Serial.println("BLE: disconnected, re-advertising");
    log_evt("disconnect");
    enter_idle();
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
  packet[8] = gRepCount;
  packet[9] = gCueEventFired ? 0x01 : 0x00;
  gClipFlags = 0;
  gCueEventFired = false;

  const uint32_t raw_off  = HEADER_BYTES;
  const uint32_t rect_off = HEADER_BYTES + SAMPLES_PER_PACKET * 2;
  const uint32_t env_off  = HEADER_BYTES + SAMPLES_PER_PACKET * 2 * 2;

  for (uint32_t i = 0; i < SAMPLES_PER_PACKET; i++) {
    Triplet t = gRing[gRingTail];
    gRingTail = (gRingTail + 1) % RING_SIZE;

    // Feed envelope channel into rolling-mean ring for fatigue/rep detection.
    env_push(t.env);

    // Track calibration peak on rolling env avg while Calibrating.
    if (gSessionState == Calibrating) {
      float avg = env_avg();
      if (avg > gCalibrationPeak) gCalibrationPeak = avg;
    }

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
  Serial.print("Bioliminal autonomous: 2 kHz sampling, 40 Hz notify, ");
  Serial.print(PACKET_BYTES);
  Serial.println(" B/pkt (10-B header with rep_count + cue_event)");
  Serial.println("FF04 accepts SET_SESSION_STATE only; other opcodes ignored");
  Serial.print("Advertising as '");
  Serial.print(DEVICE_NAME);
  Serial.println("'");
}

void loop() {
  static uint32_t next_sample_us = 0;

  uint32_t now_us = micros();
  if ((int32_t)(now_us - next_sample_us) >= 0) {
    next_sample_us = now_us + SAMPLE_PERIOD_US;
    if (gSessionState != Idle) {
      sample_once();
    }
  }

  uint32_t now_ms = millis();

  // Auto-advance Calibrating -> Active after 5 s.
  if (gSessionState == Calibrating &&
      (now_ms - gCalibratingStartMs) >= CAL_DURATION_MS) {
    enter_active();
  }

  tick_pulse_schedulers();
  rep_detector_tick(now_ms);
  // Fatigue cues are fired from inside rep_detector_tick at each rep
  // boundary — no periodic tick needed.

  if (gConnected && gSessionState != Idle && ring_count() >= SAMPLES_PER_PACKET) {
    send_packet();
  }
}
