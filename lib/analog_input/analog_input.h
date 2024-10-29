#pragma once

#include <Adafruit_ADS1X15.h>
#include <Arduino.h>
#include <SPI.h>

enum class AnalogPin : uint8_t {
  PIN_A0 = 0,
  PIN_A1 = 1,
  PIN_A2 = 2,
  PIN_A3 = 3
};

class AnalogInput {
 public:
  AnalogInput();
  AnalogInput(uint8_t address, adsGain_t gain);
  ~AnalogInput();
  void printRawAnalogInputValue();
  float readAnalogInput(AnalogPin pin);

 private:
  Adafruit_ADS1115 _ads;
};
