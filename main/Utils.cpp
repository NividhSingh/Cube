#include "Utils.h"
#include <Arduino.h>

extern double Kp, Ki, Kd;

void processSerialInput() {
  static String inputString = "";
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      if (inputString.startsWith("Kp ")) {
        Kp = inputString.substring(3).toFloat();
      } else if (inputString.startsWith("Ki ")) {
        Ki = inputString.substring(3).toFloat();
      } else if (inputString.startsWith("Kd ")) {
        Kd = inputString.substring(3).toFloat();
      } else {
        Serial.println("Unknown command.");
      }
      inputString = "";
    } else {
      inputString += inChar;
    }
  }
}
