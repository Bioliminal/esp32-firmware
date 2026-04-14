// EMG fatigue detection — single sensor, single motor.
// Flow: reset (EN) -> 5s calibration flex -> active detection -> haptic feedback.

const int EMG_PIN   = 34;   // ADC1_CH6, input-only
const int MOTOR_PIN = 25;   // MOSFET gate via 220Ω

const uint32_t SAMPLE_PERIOD_MS = 2;       // ~500 Hz
const uint32_t CAL_DURATION_MS  = 5000;    // 5 s calibration window
const int      WINDOW_SIZE      = 50;      // rolling average samples

const int     MOTOR_PWM_FREQ = 2000;       // 2 kHz — above audible hum
const uint8_t MOTOR_PWM_RES  = 8;          // 8-bit duty (0–255)

int   ringBuf[WINDOW_SIZE];
int   ringIdx     = 0;
long  ringSum     = 0;
int   ringFilled  = 0;

float calibrationPeak = 0.0f;
bool  calibrating     = true;
uint32_t sessionStartMs = 0;
uint32_t lastSampleMs   = 0;

// Motor pulse pattern state
struct PulsePattern {
  uint32_t onMs;
  uint32_t offMs;
  uint8_t  duty;   // 0–255
};
PulsePattern currentPattern = {0, 0, 0};
bool     motorPhaseOn   = false;
uint32_t motorPhaseStart = 0;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  ledcAttach(MOTOR_PIN, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcWrite(MOTOR_PIN, 0);

  for (int i = 0; i < WINDOW_SIZE; i++) ringBuf[i] = 0;

  sessionStartMs = millis();
  Serial.println("# EMG fatigue session start");
  Serial.println("# CAL phase: flex hard for 5 seconds");
}

void loop() {
  uint32_t now = millis();

  if (now - lastSampleMs >= SAMPLE_PERIOD_MS) {
    lastSampleMs = now;
    sampleAndProcess(now);
  }

  updateMotor(now);
}

void sampleAndProcess(uint32_t now) {
  int raw = analogRead(EMG_PIN);

  ringSum -= ringBuf[ringIdx];
  ringBuf[ringIdx] = raw;
  ringSum += raw;
  ringIdx = (ringIdx + 1) % WINDOW_SIZE;
  if (ringFilled < WINDOW_SIZE) ringFilled++;

  float avg = (float)ringSum / (float)ringFilled;
  uint32_t elapsed = now - sessionStartMs;

  if (calibrating) {
    if (avg > calibrationPeak) calibrationPeak = avg;

    Serial.print("CAL:");
    Serial.print(elapsed);
    Serial.print(",");
    Serial.print(avg, 1);
    Serial.print(",");
    Serial.println(calibrationPeak, 1);

    if (elapsed >= CAL_DURATION_MS) {
      calibrating = false;
      Serial.print("# CAL done. peak=");
      Serial.println(calibrationPeak, 1);
      Serial.println("timestamp_ms,avg,calibration_peak,fatigue_pct,motor_on");
    }
    return;
  }

  int fatiguePct = computeFatiguePct(avg, calibrationPeak);
  currentPattern = patternForFatigue(fatiguePct);
  int motorOn = (currentPattern.duty > 0) ? 1 : 0;

  Serial.print(elapsed);
  Serial.print(",");
  Serial.print(avg, 1);
  Serial.print(",");
  Serial.print(calibrationPeak, 1);
  Serial.print(",");
  Serial.print(fatiguePct);
  Serial.print(",");
  Serial.println(motorOn);
}

// Fatigue % = how far avg has dropped from peak.
// avg >= peak        -> 0%
// avg <= 0.30 * peak -> 100%
int computeFatiguePct(float avg, float peak) {
  if (peak <= 1.0f) return 0;
  float floorVal = 0.30f * peak;
  if (avg >= peak)     return 0;
  if (avg <= floorVal) return 100;
  float drop  = peak - avg;
  float range = peak - floorVal;
  return (int)((drop / range) * 100.0f);
}

PulsePattern patternForFatigue(int pct) {
  if (pct < 20)  return {0,   0,   0};                         // off
  if (pct < 40)  return {300, 700, (uint8_t)(0.40f * 255)};    // slow pulse
  if (pct < 60)  return {200, 300, (uint8_t)(0.60f * 255)};    // medium
  if (pct < 80)  return {100, 100, (uint8_t)(0.80f * 255)};    // fast
  return               {500, 50,  255};                        // near-continuous
}

void updateMotor(uint32_t now) {
  if (currentPattern.duty == 0) {
    ledcWrite(MOTOR_PIN, 0);
    motorPhaseOn = false;
    motorPhaseStart = now;
    return;
  }

  uint32_t phaseDur = motorPhaseOn ? currentPattern.onMs : currentPattern.offMs;
  if (now - motorPhaseStart >= phaseDur) {
    motorPhaseOn = !motorPhaseOn;
    motorPhaseStart = now;
  }
  ledcWrite(MOTOR_PIN, motorPhaseOn ? currentPattern.duty : 0);
}
