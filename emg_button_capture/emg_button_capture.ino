// Button-triggered 3-channel EMG capture, no BLE.
//
// Samples RAW + RECT + ENV at 2 kHz and streams verbatim CSV over Serial
// at 921600 baud.  The BOOT button on the ESP32 dev board (GPIO0) drives
// the session state machine so no extra wiring is needed.
//
// Usage:
//   1. Flash.  Open Serial Monitor at 921600 (or redirect to a file with
//      `cat /dev/cu.usbserial-3110 > session.csv`).
//   2. Short-press BOOT once to start a session.  A "t_us,raw,rect,env"
//      header line prints and sampling begins.
//   3. Short-press BOOT at each rep onset (or whenever you want to mark a
//      rep boundary).  Each press writes a rep-marker row and increments
//      the rep counter.
//   4. Long-press BOOT (>1.5 s) to end the session.  A summary row prints.
//
// Row format:
//   S,<t_us>,<raw>,<rect>,<env>         -- sample, t_us relative to start
//   R,<t_us>,<rep_num>                   -- rep marker
//   X,<t_us>,<total_reps>                -- session end
//
// Wiring (esp32-firmware/README.md):
//   MyoWare RAW  -> GPIO35 (ADC1_CH7)
//   MyoWare RECT -> GPIO32 (ADC1_CH4)
//   MyoWare ENV  -> GPIO34 (ADC1_CH6)
//   BOOT button  -> GPIO0  (built in to the ESP32 dev board)

#include <Arduino.h>

const int RAW_PIN  = 35;
const int RECT_PIN = 32;
const int ENV_PIN  = 34;
const int BUTTON_PIN = 0;

const uint32_t SAMPLE_PERIOD_US = 500;    // 2 kHz
const uint32_t DEBOUNCE_MS      = 30;
const uint32_t LONG_PRESS_MS    = 1500;

enum class SessionState { Idle, Active };
SessionState gState = SessionState::Idle;
uint32_t gSessionStartUs = 0;
uint32_t gRepCount = 0;

void printHeader() {
  Serial.println();
  Serial.println("# Bioliminal button-triggered 3-channel EMG capture");
  Serial.println("# Sampling at 2 kHz.  RAW=GPIO35 RECT=GPIO32 ENV=GPIO34.");
  Serial.println("# Rows: S,t_us,raw,rect,env | R,t_us,rep_num | X,t_us,total_reps");
  Serial.println("# Short-press BOOT: start session / mark rep.  Long-press: end.");
  Serial.println("# Ready — waiting for start.");
}

void setup() {
  // Oversize TX buffer so 2 kHz sample prints don't stall loop() timing.
  Serial.setTxBufferSize(4096);
  Serial.begin(921600);
  delay(200);

  analogReadResolution(12);
  pinMode(RAW_PIN, INPUT);
  pinMode(RECT_PIN, INPUT);
  pinMode(ENV_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  printHeader();
}

void onShortPress() {
  if (gState == SessionState::Idle) {
    gState = SessionState::Active;
    gSessionStartUs = micros();
    gRepCount = 0;
    Serial.println("# --- session started ---");
    Serial.println("S,t_us,raw,rect,env");
  } else {
    gRepCount++;
    Serial.print("R,");
    Serial.print(micros() - gSessionStartUs);
    Serial.print(',');
    Serial.println(gRepCount);
  }
}

void onLongPress() {
  if (gState == SessionState::Active) {
    Serial.print("X,");
    Serial.print(micros() - gSessionStartUs);
    Serial.print(',');
    Serial.println(gRepCount);
    Serial.println("# --- session ended ---");
    gState = SessionState::Idle;
    gRepCount = 0;
  }
}

void loop() {
  static bool last_button_down = false;
  static uint32_t button_down_at_ms = 0;
  static uint32_t next_sample_us = 0;

  bool button_down = (digitalRead(BUTTON_PIN) == LOW);
  uint32_t now_ms = millis();

  if (button_down && !last_button_down) {
    button_down_at_ms = now_ms;
  }
  if (!button_down && last_button_down) {
    uint32_t duration = now_ms - button_down_at_ms;
    if (duration >= DEBOUNCE_MS) {
      if (duration >= LONG_PRESS_MS) {
        onLongPress();
      } else {
        onShortPress();
      }
    }
  }
  last_button_down = button_down;

  if (gState == SessionState::Active) {
    uint32_t now = micros();
    if ((int32_t)(now - next_sample_us) >= 0) {
      next_sample_us = now + SAMPLE_PERIOD_US;

      uint16_t raw  = analogRead(RAW_PIN);
      uint16_t rect = analogRead(RECT_PIN);
      uint16_t env  = analogRead(ENV_PIN);

      Serial.print('S');
      Serial.print(',');
      Serial.print(now - gSessionStartUs);
      Serial.print(',');
      Serial.print(raw);
      Serial.print(',');
      Serial.print(rect);
      Serial.print(',');
      Serial.println(env);
    }
  }
}
