#include <ESP32Servo.h>

#define SERVO_PIN 14
#define POT_PIN 39       // Potentiometer

Servo myServo;


void setup() {
    myServo.attach(SERVO_PIN);
    myServo.write(0);
    Serial.begin(115200);
}

void loop() {
  float value = analogRead(POT_PIN);
  Serial.println(value);
  value = value / 4096 * 180;
  myServo.write(int(value));
  delay(100);
}