// No-BLE bench test for the MyoWare 2.0 three-channel wiring.
//
// Samples RAW + RECT + ENV at 2 kHz (500 µs period) using polled micros().
// Prints every 10th sample as CSV at 115200 baud -> 200 Hz display rate,
// safe for Arduino IDE Serial Plotter while the underlying sample rate
// stays at 2 kHz (preserving Nyquist margin for the 20-500 Hz MyoWare band).
//
// Wiring (matches esp32-firmware/README.md revision 2026-04-16):
//   MyoWare RAW  -> GPIO35 (ADC1_CH7)
//   MyoWare RECT -> GPIO32 (ADC1_CH4)
//   MyoWare ENV  -> GPIO34 (ADC1_CH6)
//   MyoWare VIN  -> 3V3
//   MyoWare GND  -> GND
//
// Expected values (3.3V supply, 12-bit ADC):
//   RAW  at rest ~2048 (VCC/2 midpoint); contraction ~800-3300 excursions
//   RECT at rest ~0-200;                 contraction peaks 1500-3500
//   ENV  at rest ~0-200;                 contraction rises smoothly 2000-3500

#include <Arduino.h>

const int RAW_PIN  = 35;
const int RECT_PIN = 32;
const int ENV_PIN  = 34;

const uint32_t SAMPLE_PERIOD_US = 500;   // 2 kHz
const uint32_t PRINT_EVERY = 10;         // 200 Hz to Serial

const uint16_t ADC_MAX = 4095;
const uint16_t ADC_MIN = 0;

void setup() {
  Serial.begin(115200);
  delay(200);

  analogReadResolution(12);
  pinMode(RAW_PIN, INPUT);
  pinMode(RECT_PIN, INPUT);
  pinMode(ENV_PIN, INPUT);

  Serial.println();
  Serial.println("# MyoWare 2.0 three-channel bench test");
  Serial.println("# RAW=GPIO35 RECT=GPIO32 ENV=GPIO34");
  Serial.println("# Sampling @ 2 kHz, printing every 10th sample (200 Hz)");
  Serial.println("# Columns: t_us,raw,rect,env,clip");
  Serial.println("t_us,raw,rect,env,clip");
}

void loop() {
  static uint32_t next_sample_us = 0;
  static uint32_t sample_count = 0;

  uint32_t now = micros();
  if ((int32_t)(now - next_sample_us) < 0) return;
  next_sample_us = now + SAMPLE_PERIOD_US;

  uint16_t raw  = analogRead(RAW_PIN);
  uint16_t rect = analogRead(RECT_PIN);
  uint16_t env  = analogRead(ENV_PIN);
  sample_count++;

  uint8_t clip = 0;
  if (raw  == ADC_MAX || raw  == ADC_MIN) clip |= 0x1;
  if (rect == ADC_MAX || rect == ADC_MIN) clip |= 0x2;
  if (env  == ADC_MAX || env  == ADC_MIN) clip |= 0x4;

  if (sample_count % PRINT_EVERY == 0) {
    Serial.print(now);
    Serial.print(',');
    Serial.print(raw);
    Serial.print(',');
    Serial.print(rect);
    Serial.print(',');
    Serial.print(env);
    Serial.print(',');
    Serial.println(clip);
  }
}
