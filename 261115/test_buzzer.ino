#define BUZZER_PIN 5     // Buzzer

void setup() {
}

void loop() {
  tone(BUZZER_PIN, 64);
  delay(1000);
  tone(BUZZER_PIN, 128);
  delay(1000);
  tone(BUZZER_PIN, 256);
  delay(1000);
  tone(BUZZER_PIN, 0);
  delay(1000);
}
