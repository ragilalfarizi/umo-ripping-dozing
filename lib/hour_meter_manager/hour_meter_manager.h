#pragma once

#include <Arduino.h>

class HourMeter
{
public:
    HourMeter();
    ~HourMeter();

    int32_t getSavedHourMeter();
    int32_t saveToEEPROM();
    int32_t loadFromEEPROM();

private:
    int32_t savedHourMeter;
};
