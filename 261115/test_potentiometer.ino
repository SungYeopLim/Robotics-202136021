#define POT_PIN 39       // Potentiometer

void setup() {
  Serial.begin(115200);
}

void loop() {
  int value = analogRead(POT_PIN);

  Serial.println(value);
  
  delay(10);
}