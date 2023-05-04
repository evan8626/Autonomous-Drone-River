#include <Arduino.h>
#include "Headers/double_from_serial.h"

double read_double_from_serial() {
  double value;
  char buffer[32];
  int index = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      buffer[index] = '\0';
      break;
    }
    buffer[index++] = c;
  }
  value = atof(buffer);
  return value;
}
