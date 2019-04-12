#include "utils.h"
#include <Arduino.h>
#include "time.h"

void blinkLED(byte pin, uint delayMs)
{
    digitalWrite(pin, HIGH);
    delay(delayMs);
    digitalWrite(pin, LOW);
    delay(delayMs);
}

bool printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo,0)){
    Serial.println("Failed to obtain time");
    return false;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  return true;
}