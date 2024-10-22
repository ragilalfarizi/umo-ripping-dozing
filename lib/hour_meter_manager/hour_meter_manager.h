#pragma once

#include <Arduino.h>
#include <EEPROM.h>
#include "common.h"
#include <rtc.h>

#define STORAGE_ADDRESS_HOUR_METER 0 // Size of Hour Meter is 4 bytes
#define STORAGE_ADDRESS_SETTINGS 4   // Size of Settings is 4 * 4 bytes

class HourMeter
{
public:
    HourMeter(time_t& currentHourMeter, uint16_t storageSize = 512);
    ~HourMeter();

    int32_t getSavedHourMeter();

    template <typename T>
    bool saveToStorage(const T &data);

    time_t loadHMFromStorage();

    Setting_t loadSettingFromStorage();

    void printStorage();

    DateTime convertHMToHourFormat(DateTime seconds);

    DateTime convertHMToMinuteFormat(DateTime seconds);

    void resetHourMeter();

private:
    time_t savedHourMeter; // currently not used
};

template <typename T>
bool HourMeter::saveToStorage(const T &data)
{
    if (std::is_same<T, time_t>::value)
    {
        EEPROM.put(STORAGE_ADDRESS_HOUR_METER, data);
        savedHourMeter =+ data;

        Serial.printf("Now Your hour meter is %u\n", savedHourMeter);
    }
    else if (std::is_same<T, Setting_t>::value)
    {
        EEPROM.put(STORAGE_ADDRESS_SETTINGS, data);
    }
    else
    {
        return false;
    }

    EEPROM.commit();

    return true;
}
