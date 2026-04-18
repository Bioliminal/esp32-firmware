// Bioliminal Garment hardware bench test — NO BLE, no app required.
//
// Purpose: validate the Phase 2 firmware's ADC pipeline, pulse-burst
// scheduler, and motor output independently of the phone app.  Use this
// sketch to confirm the hardware works before flashing bicep_realtime.ino
// (its BLE-paired twin).
//
// Two ways to fire a cue without a phone:
//   1. BOOT button
//        short press  -> FADE preset   (duty 180, 2 pulses, 200 ms on / 150 ms off)
//        long press   -> URGENT preset (duty 230, 2 pulses, 200 ms on / 150 ms off)
//   2. Serial commands (115200 baud, newline-terminated):
//        fade                                -> FADE preset
//        urgent                              -> URGENT preset
//        form                                -> FORM staccato (duty 230, 3 × 100/80)
//        pulse <duty> <n> <on_ms> <off_ms>   -> arbitrary PULSE_BURST on motor 0
//        stop                                -> immediately stop motor 0
//        help                                -> reprint this usage
//
// Every sample is also emitted as CSV so you can watch EMG in Serial
// Plotter at the same time.  Format:
//   S,<t_us>,<raw>,<rect>,<env>
//   H,<t_us>,<cue_name>      when a cue fires (so it aligns visually with the EMG trace)
//
// Wiring (same as bicep_realtime.ino):
//   MyoWare RAW  -> GPIO35 (ADC1_CH7)
//   MyoWare RECT -> GPIO32 (ADC1_CH4)
//   MyoWare ENV  -> GPIO34 (ADC1_CH6)
//   Motor 0 PWM  -> GPIO25 (via 2N7000 MOSFET, 2 kHz LEDC, 8-bit duty)
//   BOOT button  -> GPIO0 (built-in on the ESP32 dev board)

#include <Arduino.h>

const int RAW_PIN    = 35;
const int RECT_PIN   = 32;
const int ENV_PIN    = 34;
const int MOTOR_PIN  = 25;
const int BUTTON_PIN = 0;

const uint32_t SAMPLE_PERIOD_US = 500;   // 2 kHz sampling; print every 10th row
const uint32_t PRINT_EVERY      = 10;    // 200 Hz rows to Serial

const int      MOTOR_LEDC_FREQ_HZ  = 2000;
const int      MOTOR_LEDC_RES_BITS = 8;

const uint32_t DEBOUNCE_MS   = 30;
const uint32_t LONG_PRESS_MS = 1500;

const uint16_t ADC_MAX = 4095;
const uint16_t ADC_MIN = 0;

// ---------------------------------------------------------------------------
// Pulse-burst scheduler (single motor; mirrors the logic in bicep_realtime)
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

PulseBurst gBurst = {};

void start_pulse_burst(uint8_t duty, uint8_t n, uint16_t on_ms, uint16_t off_ms,
                       const char* label) {
  if (n == 0) return;
  gBurst.active = true;
  gBurst.duty = duty;
  gBurst.pulses_remaining = n;
  gBurst.on_ms = on_ms;
  gBurst.off_ms = off_ms;
  gBurst.currently_on = true;
  gBurst.next_event_ms = millis() + on_ms;
  ledcWrite(MOTOR_PIN, duty);

  Serial.print("H,");
  Serial.print(micros());
  Serial.print(',');
  Serial.println(label);
}

void stop_pulse_burst() {
  gBurst.active = false;
  gBurst.pulses_remaining = 0;
  ledcWrite(MOTOR_PIN, 0);
  Serial.print("H,");
  Serial.print(micros());
  Serial.println(",stop");
}

void tick_pulse_scheduler() {
  if (!gBurst.active) return;
  const uint32_t now = millis();
  if ((int32_t)(now - gBurst.next_event_ms) < 0) return;

  if (gBurst.currently_on) {
    ledcWrite(MOTOR_PIN, 0);
    if (gBurst.pulses_remaining <= 1) {
      gBurst.active = false;
      gBurst.pulses_remaining = 0;
    } else {
      gBurst.currently_on = false;
      gBurst.next_event_ms = now + gBurst.off_ms;
    }
  } else {
    gBurst.pulses_remaining--;
    gBurst.currently_on = true;
    ledcWrite(MOTOR_PIN, gBurst.duty);
    gBurst.next_event_ms = now + gBurst.on_ms;
  }
}

// Cue presets (exactly match the BLE payloads in bicep_realtime.ino)
void fire_fade()   { start_pulse_burst(180, 2, 200, 150, "fade"); }
void fire_urgent() { start_pulse_burst(230, 2, 200, 150, "urgent"); }
void fire_form()   { start_pulse_burst(230, 3, 100,  80, "form"); }

// ---------------------------------------------------------------------------
// Serial command parser
// ---------------------------------------------------------------------------

String serial_line;

void print_help() {
  Serial.println("# commands: fade | urgent | form | stop | help");
  Serial.println("#           pulse <duty 0-255> <n 1-20> <on_ms> <off_ms>");
}

void handle_serial_line(String line) {
  line.trim();
  if (line.length() == 0) return;
  line.toLowerCase();

  if (line == "fade")        { fire_fade(); return; }
  if (line == "urgent")      { fire_urgent(); return; }
  if (line == "form")        { fire_form(); return; }
  if (line == "stop")        { stop_pulse_burst(); return; }
  if (line == "help")        { print_help(); return; }

  if (line.startsWith("pulse")) {
    int duty, n, on, off;
    // "pulse 180 2 200 150"
    if (sscanf(line.c_str(), "pulse %d %d %d %d", &duty, &n, &on, &off) == 4) {
      if (duty < 0 || duty > 255 || n < 1 || n > 20 ||
          on < 0 || on > 5000 || off < 0 || off > 5000) {
        Serial.println("# pulse: out-of-range arg");
        return;
      }
      start_pulse_burst((uint8_t)duty, (uint8_t)n, (uint16_t)on, (uint16_t)off,
                        "custom");
      return;
    }
    Serial.println("# pulse: expected 'pulse <duty> <n> <on_ms> <off_ms>'");
    return;
  }

  Serial.print("# unknown command: ");
  Serial.println(line);
  print_help();
}

void poll_serial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      handle_serial_line(serial_line);
      serial_line = "";
    } else {
      serial_line += c;
      if (serial_line.length() > 80) serial_line = "";  // anti-runaway
    }
  }
}

// ---------------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(200);

  analogReadResolution(12);
  pinMode(RAW_PIN, INPUT);
  pinMode(RECT_PIN, INPUT);
  pinMode(ENV_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  ledcAttach(MOTOR_PIN, MOTOR_LEDC_FREQ_HZ, MOTOR_LEDC_RES_BITS);
  ledcWrite(MOTOR_PIN, 0);

  Serial.println();
  Serial.println("# Bioliminal hardware bench test (no BLE)");
  Serial.println("# RAW=GPIO35 RECT=GPIO32 ENV=GPIO34 MOTOR=GPIO25 BTN=GPIO0");
  Serial.println("# EMG sampling at 2 kHz, printing every 10th row (200 Hz)");
  Serial.println("# BOOT: short=FADE, long=URGENT.  Also accepts Serial cmds.");
  print_help();
  Serial.println("S,t_us,raw,rect,env");
}

void loop() {
  static bool     last_button_down = false;
  static uint32_t button_down_at_ms = 0;
  static uint32_t next_sample_us   = 0;
  static uint32_t sample_count     = 0;

  // --- button handling ---
  bool button_down = (digitalRead(BUTTON_PIN) == LOW);
  uint32_t now_ms = millis();

  if (button_down && !last_button_down) {
    button_down_at_ms = now_ms;
  }
  if (!button_down && last_button_down) {
    uint32_t duration = now_ms - button_down_at_ms;
    if (duration >= DEBOUNCE_MS) {
      if (duration >= LONG_PRESS_MS) fire_urgent();
      else                           fire_fade();
    }
  }
  last_button_down = button_down;

  // --- serial command input ---
  poll_serial();

  // --- pulse scheduler tick ---
  tick_pulse_scheduler();

  // --- EMG sampling ---
  uint32_t now_us = micros();
  if ((int32_t)(now_us - next_sample_us) >= 0) {
    next_sample_us = now_us + SAMPLE_PERIOD_US;

    uint16_t raw  = analogRead(RAW_PIN);
    uint16_t rect = analogRead(RECT_PIN);
    uint16_t env  = analogRead(ENV_PIN);
    sample_count++;

    if (sample_count % PRINT_EVERY == 0) {
      Serial.print('S');
      Serial.print(',');
      Serial.print(now_us);
      Serial.print(',');
      Serial.print(raw);
      Serial.print(',');
      Serial.print(rect);
      Serial.print(',');
      Serial.println(env);
    }
  }
}
