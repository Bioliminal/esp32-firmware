const int EMG_PIN = 34;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
}

void loop() {
  int raw = analogRead(EMG_PIN);
  Serial.println(raw);
  delay(2);
}