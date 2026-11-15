#include <ESP32Servo.h>

#define SERVO_PIN 14

Servo myServo;


void setup() {
    myServo.attach(SERVO_PIN);
    myServo.write(0);
}

void loop() {
  for (int degree = 0; degree <= 180; ++degree){
    myServo.write(degree);
    delay(1);

  }
  for (int degree = 180; degree >= 0; --degree){
    myServo.write(degree);
    delay(1);
  }
}