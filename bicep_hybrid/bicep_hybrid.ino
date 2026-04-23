// Bioliminal Garment — hybrid firmware (pose-reconciled).
//
// Implements the ratified architecture from
// `bioliminal-ops/decisions/2026-04-21-pose-authoritative-rep-counting.md`
// (Aaron, 2026-04-21 — amends 2026-04-20-firmware-led-cue-ownership):
//
//   • Phone owns rep counting (pose elbow-angle state machine).
//   • Firmware keeps haptic + fatigue + cue firing, runs a local EMG-peak
//     rep detector as a BLE-dropout fallback, and logs [rep-disagree] when
//     phone and firmware counts diverge by more than 1.
//
// Two cueing tracks, running in parallel:
//
//   • Track A — continuous-gradient fatigue haptic (ported from
//     emg_fatigue.ino): user feels progressive motor intensity as the
//     envelope drops off the calibration peak, independent of rep count.
//   • Track B — categorical cue events emitted on FF02 byte 9 enum,
//     triggered on each pose-confirmed rep boundary.  Phone reads the
//     byte, maps to a TTS phrase, and speaks it.
//
// Co-exists alongside bicep_autonomous.ino (the pre-amendment design,
// kept for reference) and bicep_realtime.ino (legacy phone-led).
//
// Packet layout (IC-1, 310 bytes, 10-byte header, unchanged from
// bicep_autonomous.ino):
//   [0]        seq_num u8
//   [1..4]     t_us_start u32 LE
//   [5]        channel_count u8 = 3
//   [6]        samples_per_channel u8 = 50
//   [7]        flags u8 (bit0 RAW clip, bit1 RECT clip, bit2 ENV clip)
//   [8]        rep_count u8  — PHONE-AUTHORITATIVE (was firmware in prior sketch)
//   [9]        cue_event u8  — ENUM (was 1-bit flag in prior sketch)
//                              0x00 none, 0x10 fade, 0x11 urgent, 0x12 stop,
//                              0x20 calibration_done
//   [10..109]  RAW  samples (50 x u16 LE)
//   [110..209] RECT samples (50 x u16 LE)
//   [210..309] ENV  samples (50 x u16 LE)
//
// FF04 opcodes (phone -> firmware):
//   0x12 SET_SESSION_STATE [state u8]           (0=Idle, 1=Calibrating)
//   0x13 OP_REP_CONFIRMED  [rep_num u8][t_ms u24 LE]  — NEW
//
// Serial rows (capture_ble_full.py parses [rx]/[evt]/[rep]/[cue]/
// [rep-disagree]/[rep-reject]/[cal-reject]/[fatigue-skip]/[rx-rep-confirm]):
//   [rx]              t_ms=... len=... op=0x... bytes=...
//   [evt]             t_ms=... kind=(connect|disconnect|session_state)[ state=...]
//   [rep]             t_ms=... rep_count=...     (LOCAL rep detector only)
//   [rx-rep-confirm]  t_ms=... rep_num=... t_ms_phone=...  (phone confirm arrival)
//   [rep-disagree]    t_ms=... phone=... firmware=...
//   [rep-reject]      t_ms=... reason=spike peak=...
//   [cue]             t_ms=... rep=... drop_pct=... kind=0x...
//   [cal-reject]      t_ms=... reason=low_peak peak=... floor=...
//   [fatigue-skip]    t_ms=... reason=clip
//
// Wiring (unchanged from bicep_autonomous):
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

const int HEADER_BYTES = 10;
const int PACKET_BYTES = HEADER_BYTES + SAMPLES_PER_PACKET * 3 * sizeof(uint16_t);  // 310

const uint16_t ADC_MAX = 4095;
const uint16_t ADC_MIN = 0;

// --- Haptic motors ---
const int MOTOR_COUNT = 4;                     // protocol allows 4; only motor 0 wired
const int MOTOR_PINS[MOTOR_COUNT] = {25, -1, -1, -1};
const int MOTOR_LEDC_FREQ_HZ   = 2000;
const int MOTOR_LEDC_RES_BITS  = 8;

// --- Session state ---
enum SessionState : uint8_t { Idle = 0, Calibrating = 1, Active = 2 };
SessionState gSessionState = Idle;
uint32_t gCalibratingStartMs  = 0;
const uint32_t CAL_DURATION_MS = 5000;
const uint32_t CAL_NOISE_FLOOR_SAMPLE_MS = 300;  // sample floor this many ms into Calibrating

// --- FF04 opcodes ---
static const uint8_t OP_SET_SESSION_STATE = 0x12;
static const uint8_t OP_REP_CONFIRMED     = 0x13;   // NEW: phone-confirmed rep

// --- FF02 byte 9 cue event enum ---
const uint8_t CUE_NONE              = 0x00;
const uint8_t CUE_FATIGUE_FADE      = 0x10;
const uint8_t CUE_FATIGUE_URGENT    = 0x11;
const uint8_t CUE_FATIGUE_STOP      = 0x12;
const uint8_t CUE_CALIBRATION_DONE  = 0x20;
uint8_t gLastCueKind = CUE_NONE;

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
// Envelope rolling mean (50-sample window; ~25 ms at 2 kHz)
// ---------------------------------------------------------------------------

const int ENV_WINDOW = 50;
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
// Calibration state
//
// gCalibrationPeak: max env_avg observed during Calibrating window.
// gNoiseFloor:      env_avg sampled ~300 ms into Calibrating, before user is
//                   expected to have started flexing.  Used as sanity check:
//                   if calibration_peak < 2*noise_floor at end of window,
//                   we reject calibration and extend it another 5 s.
// ---------------------------------------------------------------------------

float gCalibrationPeak = 0.0f;
float gNoiseFloor      = 0.0f;
bool  gNoiseFloorSampled = false;

// ---------------------------------------------------------------------------
// Rep counts
//
// gRepCountConfirmed — authoritative, written to FF02 byte 8 and used for
//                      fatigue baseline windowing.  Set by OP_REP_CONFIRMED
//                      writes from the phone.
// gRepCountLocal     — fallback, driven by local envelope-peak detector.
//                      Compared to confirmed count for [rep-disagree]
//                      telemetry; never written to the wire.
// ---------------------------------------------------------------------------

uint8_t gRepCountConfirmed = 0;
uint8_t gRepCountLocal     = 0;
uint8_t gLastCueRep        = 0;

const uint8_t CUE_START_REP    = 5;    // first categorical cue possible at rep 6
const uint8_t CUE_REP_COOLDOWN = 2;    // min reps between categorical cues

const int REP_PEAK_HISTORY = 16;
float gRepPeakHistory[REP_PEAK_HISTORY] = {0};

// Peak of env_avg since the previous OP_REP_CONFIRMED.  Captured as this
// rep's peak and reset on each confirm.  Unlike bicep_autonomous.ino where
// the local detector wrote peaks, here the phone's rep boundaries define
// the windows; firmware just tracks max(env_avg) between them.
float gPeakSinceLastConfirm = 0.0f;

// --- Motion artifact / clip tracking ---
// Rolling 5-bit history of the ENV clip flag.  If any of the last 5 packets
// saturated ENV, we skip fatigue cue evaluation (likely motion artifact).
uint8_t gRecentClipHistory = 0;

// ---------------------------------------------------------------------------
// Local rep detector (FALLBACK ONLY — never writes to the wire)
//
// Preserves the envelope-peak state machine from bicep_autonomous.ino so
// firmware has a count to report when BLE drops or phone pose tracking is
// lost.  gRepCountLocal is compared to gRepCountConfirmed to detect drift.
// ---------------------------------------------------------------------------

enum RepPhase : uint8_t { RepQuiet = 0, RepRising = 1, RepCounted = 2 };
RepPhase gRepPhase = RepQuiet;
uint32_t gLastLocalRepMs = 0;
const uint32_t REP_COOLDOWN_MS = 250;
const float REP_RISE_FRAC      = 0.30f;
const float REP_PEAK_DROP_FRAC = 0.80f;
float gCurrentLocalRepPeak = 0.0f;

void rep_detector_local_tick(uint32_t now_ms) {
  if (gSessionState != Active) return;
  if (gCalibrationPeak <= 1.0f) return;

  float avg = env_avg();
  float rise_thresh = REP_RISE_FRAC * gCalibrationPeak;

  switch (gRepPhase) {
    case RepQuiet:
      if (avg >= rise_thresh) {
        gRepPhase = RepRising;
        gCurrentLocalRepPeak = avg;
      }
      break;

    case RepRising:
      if (avg > gCurrentLocalRepPeak) {
        gCurrentLocalRepPeak = avg;
      }
      if (gCurrentLocalRepPeak > 1.5f * rise_thresh &&
          avg < REP_PEAK_DROP_FRAC * gCurrentLocalRepPeak) {

        // Spike rejection: if the rep peak is >5x the ambient rolling mean,
        // treat as motion artifact and skip the count.  Comparing to
        // env_avg() here is a cheap stand-in for "ambient" — the rolling
        // window includes the current contraction so peaks during quiet
        // between reps dominate the ratio.
        float ambient = env_avg();
        if (ambient > 1.0f && gCurrentLocalRepPeak > 5.0f * ambient) {
          Serial.print("[rep-reject] t_ms=");
          Serial.print(now_ms);
          Serial.print(" reason=spike peak=");
          Serial.println(gCurrentLocalRepPeak, 1);
          gRepPhase = RepQuiet;
          break;
        }

        if (now_ms - gLastLocalRepMs >= REP_COOLDOWN_MS) {
          gRepCountLocal++;
          gLastLocalRepMs = now_ms;
          Serial.print("[rep] t_ms=");
          Serial.print(now_ms);
          Serial.print(" rep_count=");
          Serial.print(gRepCountLocal);
          Serial.print(" peak=");
          Serial.println(gCurrentLocalRepPeak, 1);
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
// Track A — continuous-gradient fatigue (ported from emg_fatigue.ino)
//
// Drives motor 0 continuously based on how far env_avg has dropped from
// gCalibrationPeak.  Gated on (Active && gRepCountConfirmed >= CUE_START_REP)
// so reps 1-5 stay silent.
// ---------------------------------------------------------------------------

struct PulsePattern {
  uint32_t onMs;
  uint32_t offMs;
  uint8_t  duty;   // 0-255
};

PulsePattern gCurrentPattern = {0, 0, 0};
bool     gMotorPhaseOn   = false;
uint32_t gMotorPhaseStart = 0;

// fatigue % = how far env_avg has dropped from gCalibrationPeak.
//   avg >= peak          ->   0%
//   avg <= 0.30 * peak   -> 100%
int compute_fatigue_pct(float avg, float peak) {
  if (peak <= 1.0f) return 0;
  float floorVal = 0.30f * peak;
  if (avg >= peak)     return 0;
  if (avg <= floorVal) return 100;
  float drop  = peak - avg;
  float range = peak - floorVal;
  return (int)((drop / range) * 100.0f);
}

PulsePattern pattern_for_fatigue(int pct) {
  if (pct < 20)  return {0,   0,   0};                         // off
  if (pct < 40)  return {300, 700, (uint8_t)(0.40f * 255)};    // slow pulse
  if (pct < 60)  return {200, 300, (uint8_t)(0.60f * 255)};    // medium
  if (pct < 80)  return {100, 100, (uint8_t)(0.80f * 255)};    // fast
  return               {500, 50,  255};                        // near-continuous
}

inline void motor0_write(uint8_t duty) {
  int pin = MOTOR_PINS[0];
  if (pin >= 0) ledcWrite(pin, duty);
}

void update_motor_continuous(uint32_t now) {
  if (gCurrentPattern.duty == 0) {
    motor0_write(0);
    gMotorPhaseOn = false;
    gMotorPhaseStart = now;
    return;
  }

  uint32_t phaseDur = gMotorPhaseOn ? gCurrentPattern.onMs : gCurrentPattern.offMs;
  if (now - gMotorPhaseStart >= phaseDur) {
    gMotorPhaseOn = !gMotorPhaseOn;
    gMotorPhaseStart = now;
  }
  motor0_write(gMotorPhaseOn ? gCurrentPattern.duty : 0);
}

// ---------------------------------------------------------------------------
// Track B — categorical cue evaluation
//
// Called from the OP_REP_CONFIRMED handler on each pose-confirmed rep
// boundary.  Does NOT drive the motor (Track A owns that); only writes the
// cue_event byte that the phone decodes to a TTS phrase.
// ---------------------------------------------------------------------------

float mean_of_first_n_peaks(int n) {
  float sum = 0.0f;
  int count = 0;
  for (int i = 0; i < n && i < REP_PEAK_HISTORY; i++) {
    if (gRepPeakHistory[i] > 1.0f) {
      sum += gRepPeakHistory[i];
      count++;
    }
  }
  if (count == 0) return 0.0f;
  return sum / count;
}

void evaluate_fatigue_cue(uint32_t now_ms) {
  // Calibration silence: first 5 reps build baseline, no cues.
  if (gRepCountConfirmed <= CUE_START_REP) return;

  // Cooldown: at least CUE_REP_COOLDOWN reps between categorical fires.
  if (gRepCountConfirmed - gLastCueRep < CUE_REP_COOLDOWN) return;

  // Motion artifact / signal integrity: skip if any of last 5 packets clipped.
  if (gRecentClipHistory != 0) {
    Serial.print("[fatigue-skip] t_ms=");
    Serial.print(now_ms);
    Serial.println(" reason=clip");
    return;
  }

  float baseline = mean_of_first_n_peaks(CUE_START_REP);
  if (baseline <= 1.0f) return;

  float current = gRepPeakHistory[(gRepCountConfirmed - 1) % REP_PEAK_HISTORY];
  if (current >= baseline) return;          // no drop — nothing to cue

  int drop_pct = (int)((baseline - current) / baseline * 100.0f);

  uint8_t kind = CUE_NONE;
  if      (drop_pct >= 50) kind = CUE_FATIGUE_STOP;
  else if (drop_pct >= 25) kind = CUE_FATIGUE_URGENT;
  else if (drop_pct >= 15) kind = CUE_FATIGUE_FADE;
  else return;

  gLastCueKind = kind;
  gLastCueRep  = gRepCountConfirmed;

  Serial.print("[cue] t_ms=");
  Serial.print(now_ms);
  Serial.print(" rep=");
  Serial.print(gRepCountConfirmed);
  Serial.print(" peak=");
  Serial.print(current, 1);
  Serial.print(" baseline=");
  Serial.print(baseline, 1);
  Serial.print(" drop_pct=");
  Serial.print(drop_pct);
  Serial.print(" kind=0x");
  if (kind < 0x10) Serial.print('0');
  Serial.println(kind, HEX);
}

// ---------------------------------------------------------------------------
// FF04 command parsing
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
  motor0_write(0);
  gCurrentPattern = {0, 0, 0};
  gMotorPhaseOn = false;
  gRingHead = 0;
  gRingTail = 0;
  gClipFlags = 0;
  gRecentClipHistory = 0;
  gRepPhase = RepQuiet;
  gRepCountConfirmed = 0;
  gRepCountLocal = 0;
  gLastCueRep = 0;
  gLastCueKind = CUE_NONE;
  gCurrentLocalRepPeak = 0.0f;
  gPeakSinceLastConfirm = 0.0f;
  for (int i = 0; i < REP_PEAK_HISTORY; i++) gRepPeakHistory[i] = 0.0f;
  gCalibrationPeak = 0.0f;
  gNoiseFloor = 0.0f;
  gNoiseFloorSampled = false;
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
  gNoiseFloor = 0.0f;
  gNoiseFloorSampled = false;
  gRepPhase = RepQuiet;
  gRepCountConfirmed = 0;
  gRepCountLocal = 0;
  gLastCueRep = 0;
  gLastCueKind = CUE_NONE;
  gCurrentLocalRepPeak = 0.0f;
  gPeakSinceLastConfirm = 0.0f;
  for (int i = 0; i < REP_PEAK_HISTORY; i++) gRepPeakHistory[i] = 0.0f;
  log_session_state("Calibrating");
}

void enter_active() {
  gSessionState = Active;
  gLastCueKind = CUE_CALIBRATION_DONE;   // phone sees calibration_done on first packet after entry
  log_session_state("Active");
}

void handle_command(const uint8_t* data, size_t len) {
  log_rx(data, len);

  if (len == 0) return;
  uint8_t op = data[0];

  switch (op) {
    case OP_SET_SESSION_STATE: {
      if (len < 2) {
        Serial.println("cmd: SET_SESSION_STATE too short");
        return;
      }
      uint8_t s = data[1];
      if      (s == Idle)        enter_idle();
      else if (s == Calibrating) enter_calibrating();
      else if (s == Active)      enter_active();   // phone normally doesn't set this, but accept
      else {
        Serial.print("cmd: SET_SESSION_STATE invalid state=");
        Serial.println(s);
      }
      break;
    }

    case OP_REP_CONFIRMED: {
      if (len < 5) {
        Serial.println("cmd: REP_CONFIRMED too short");
        return;
      }
      uint8_t rep_num = data[1];
      uint32_t t_ms_phone = (uint32_t)data[2]
                          | ((uint32_t)data[3] << 8)
                          | ((uint32_t)data[4] << 16);
      uint32_t now_ms = millis();

      Serial.print("[rx-rep-confirm] t_ms=");
      Serial.print(now_ms);
      Serial.print(" rep_num=");
      Serial.print(rep_num);
      Serial.print(" t_ms_phone=");
      Serial.println(t_ms_phone);

      // Disagreement diagnostic — local count drifting from phone count by >1.
      int disagree = (int)rep_num - (int)gRepCountLocal;
      if (disagree < 0) disagree = -disagree;
      if (disagree > 1) {
        Serial.print("[rep-disagree] t_ms=");
        Serial.print(now_ms);
        Serial.print(" phone=");
        Serial.print(rep_num);
        Serial.print(" firmware=");
        Serial.println(gRepCountLocal);
      }

      // Drift-until-next-confirm semantics (Aaron ratification §3b): accept
      // whatever phone sent.  Out-of-order confirms overwrite identically to
      // in-order ones.  Phone is responsible for monotonic rep_num.
      gRepCountConfirmed = rep_num;

      // Capture the rep's peak envelope for fatigue baseline calibration.
      // gPeakSinceLastConfirm is max(env_avg) since the prior confirm;
      // reset here so the next rep window starts fresh.
      if (rep_num >= 1) {
        gRepPeakHistory[(rep_num - 1) % REP_PEAK_HISTORY] = gPeakSinceLastConfirm;
      }
      gPeakSinceLastConfirm = 0.0f;

      evaluate_fatigue_cue(now_ms);
      break;
    }

    default:
      Serial.print("cmd: opcode 0x");
      if (op < 0x10) Serial.print('0');
      Serial.print(op, HEX);
      Serial.println(" ignored");
      break;
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
  packet[8] = gRepCountConfirmed;      // PHONE-AUTHORITATIVE (was local in prior sketch)
  packet[9] = gLastCueKind;            // ENUM (was 1-bit flag in prior sketch)

  // Update rolling clip history before clearing flags.  Bit 0 of the shift
  // register tracks whether ENV clipped in this packet.
  gRecentClipHistory = ((gRecentClipHistory << 1) | ((gClipFlags & 0x04) ? 1 : 0)) & 0x1F;

  gClipFlags = 0;
  gLastCueKind = CUE_NONE;

  const uint32_t raw_off  = HEADER_BYTES;
  const uint32_t rect_off = HEADER_BYTES + SAMPLES_PER_PACKET * 2;
  const uint32_t env_off  = HEADER_BYTES + SAMPLES_PER_PACKET * 2 * 2;

  for (uint32_t i = 0; i < SAMPLES_PER_PACKET; i++) {
    Triplet t = gRing[gRingTail];
    gRingTail = (gRingTail + 1) % RING_SIZE;

    // Feed envelope rolling mean for fatigue + fallback rep detection.
    env_push(t.env);

    // Track calibration peak + noise floor during Calibrating.
    if (gSessionState == Calibrating) {
      float avg = env_avg();
      if (avg > gCalibrationPeak) gCalibrationPeak = avg;

      if (!gNoiseFloorSampled &&
          (millis() - gCalibratingStartMs) >= CAL_NOISE_FLOOR_SAMPLE_MS &&
          gEnvFilled >= ENV_WINDOW) {
        gNoiseFloor = avg;
        gNoiseFloorSampled = true;
      }
    }

    // Track max env_avg since previous phone-confirmed rep (for fatigue baseline).
    if (gSessionState == Active) {
      float avg = env_avg();
      if (avg > gPeakSinceLastConfirm) gPeakSinceLastConfirm = avg;
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
  adv->setMinPreferred(0x06);       // Apple accessory design compliance
  adv->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println();
  Serial.print("Bioliminal hybrid: 2 kHz sampling, 40 Hz notify, ");
  Serial.print(PACKET_BYTES);
  Serial.println(" B/pkt");
  Serial.println("Phone owns rep counting via OP_REP_CONFIRMED (0x13) on FF04");
  Serial.println("Firmware owns haptic (continuous-gradient) + categorical cue byte");
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

  // Auto-advance Calibrating -> Active after 5 s.  Reject calibration if
  // the peak envelope never cleared 2x the noise floor — likely bad
  // electrode contact or the user never flexed.  Extend Calibrating
  // another 5 s in that case.
  if (gSessionState == Calibrating &&
      (now_ms - gCalibratingStartMs) >= CAL_DURATION_MS) {
    if (gNoiseFloorSampled && gCalibrationPeak < 2.0f * gNoiseFloor) {
      Serial.print("[cal-reject] t_ms=");
      Serial.print(now_ms);
      Serial.print(" reason=low_peak peak=");
      Serial.print(gCalibrationPeak, 1);
      Serial.print(" floor=");
      Serial.println(gNoiseFloor, 1);
      // Restart Calibrating window.
      gCalibratingStartMs = now_ms;
      gCalibrationPeak = 0.0f;
      gNoiseFloor = 0.0f;
      gNoiseFloorSampled = false;
    } else {
      enter_active();
    }
  }

  // Local rep detector runs only for [rep-disagree] telemetry + BLE-drop
  // fallback.  Phone-confirmed count is authoritative.
  rep_detector_local_tick(now_ms);

  // Track A — continuous-gradient fatigue haptic.  Gated on Active state
  // and at least CUE_START_REP confirmed reps (no haptic during calibration).
  if (gSessionState == Active && gRepCountConfirmed >= CUE_START_REP) {
    int fatigue_pct = compute_fatigue_pct(env_avg(), gCalibrationPeak);
    gCurrentPattern = pattern_for_fatigue(fatigue_pct);
  } else {
    gCurrentPattern = {0, 0, 0};
  }
  update_motor_continuous(now_ms);

  if (gConnected && gSessionState != Idle && ring_count() >= SAMPLES_PER_PACKET) {
    send_packet();
  }
}
