#pragma once

#include <Arduino.h>
#include <Adafruit_ADS1X15.h>
#include <SPI.h>

class AnalogInput
{
public:
    AnalogInput();
    ~AnalogInput();
    void printAnalogInputValue();

private:
    Adafruit_ADS1115 _ads;
    float readjustAnalogIn();
};
