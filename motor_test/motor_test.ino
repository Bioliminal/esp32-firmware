// Motor wiring test — no EMG sensor required.
// Cycles through: full-on, PWM sweep, then the four fatigue pulse patterns.
// Watch the Serial Monitor to see which phase you're in so you can tell
// whether what you're feeling matches what the firmware is sending.

const int MOTOR_PIN = 25;

const int     MOTOR_PWM_FREQ = 2000;   // 2 kHz
const uint8_t MOTOR_PWM_RES  = 8;      // 8-bit duty (0–255)

void setup() {
  Serial.begin(115200);
  delay(200);

  ledcAttach(MOTOR_PIN, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcWrite(MOTOR_PIN, 0);

  Serial.println();
  Serial.println("=== Motor wiring test ===");
  Serial.println("GPIO25 -> 220ohm -> 2N7000 gate");
  Serial.println("Source -> GND, Drain -> motor(-), 3V3 -> motor(+)");
  Serial.println();
}

void loop() {
  // 1. Simplest possible check: full on for 2 seconds.
  Serial.println("[1/3] FULL ON (duty=255) for 2s");
  ledcWrite(MOTOR_PIN, 255);
  delay(2000);
  ledcWrite(MOTOR_PIN, 0);
  delay(500);

  // 2. Slow ramp 0 -> 255 -> 0. You should feel the intensity change smoothly.
  Serial.println("[2/3] PWM ramp 0 -> 255 -> 0 (tests that PWM actually works)");
  for (int duty = 0; duty <= 255; duty += 5) {
    ledcWrite(MOTOR_PIN, duty);
    delay(20);
  }
  for (int duty = 255; duty >= 0; duty -= 5) {
    ledcWrite(MOTOR_PIN, duty);
    delay(20);
  }
  ledcWrite(MOTOR_PIN, 0);
  delay(500);

  // 3. Walk through the four fatigue patterns so you can feel each one.
  Serial.println("[3/3] Fatigue pattern preview");
  runPattern("  20-40%  slow   ", 300, 700, (uint8_t)(0.40f * 255), 4000);
  runPattern("  40-60%  medium ", 200, 300, (uint8_t)(0.60f * 255), 4000);
  runPattern("  60-80%  fast   ", 100, 100, (uint8_t)(0.80f * 255), 4000);
  runPattern("  80-100% max    ", 500,  50, 255,                    4000);

  Serial.println("--- cycle complete, looping ---");
  Serial.println();
  delay(1000);
}

void runPattern(const char* label, uint32_t onMs, uint32_t offMs,
                uint8_t duty, uint32_t totalMs) {
  Serial.print(label);
  Serial.print("  duty=");
  Serial.print(duty);
  Serial.print("  on=");
  Serial.print(onMs);
  Serial.print("ms  off=");
  Serial.print(offMs);
  Serial.println("ms");

  uint32_t start = millis();
  while (millis() - start < totalMs) {
    ledcWrite(MOTOR_PIN, duty);
    delay(onMs);
    ledcWrite(MOTOR_PIN, 0);
    delay(offMs);
  }
  ledcWrite(MOTOR_PIN, 0);
  delay(400);
}
