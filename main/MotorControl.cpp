#include "MotorControl.h"
#include <Arduino.h>

void MotorControl::init() {
  pinMode(pwmPin, OUTPUT);
  pinMode(directionPin, OUTPUT);
  pinMode(startstopPin, OUTPUT);
  digitalWrite(startstopPin, HIGH); // Enable motor
}

void MotorControl::update(double output) {
  output = constrain(output, -154, 154);
  if (output < 0) {
    digitalWrite(directionPin, HIGH); // Reverse
    analogWrite(pwmPin, 255 - int(abs(output)));
  } else {
    digitalWrite(directionPin, LOW); // Forward
    analogWrite(pwmPin, 255 - int(abs(output)));
  }
}
