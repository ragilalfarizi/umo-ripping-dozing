#pragma once

#include <Arduino.h>
#include <EEPROM.h>
#include <LittleFS.h>
#include "common.h"
#include <rtc.h>

// WARN: DEPRECIATED
#define STORAGE_ADDRESS_HOUR_METER 0 // Size of Hour Meter is 4 bytes
#define STORAGE_ADDRESS_SETTINGS 4   // Size of Settings is 4 * 4 bytes
#define FORMAT_LITTLEFS_IF_FAILED true

class HourMeter
{
public:
    HourMeter(uint8_t format_on_fail = FORMAT_LITTLEFS_IF_FAILED);
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
    void checkAndCreateFiles();
};

template <typename T>
bool HourMeter::saveToStorage(const T &data)
{
    if (std::is_same<T, time_t>::value)
    {
        // EEPROM.put(STORAGE_ADDRESS_HOUR_METER, data);

        FILE *file = fopen("/littlefs/data.txt", "w");
        if (file)
        {
            // Write the latest value in JSON format
            fprintf(file, "%ld", data);
            fclose(file);
        }
        else
        {
            Serial.println("Failed to open data.txt for writing");
            return false;
        }
    }

    else if (std::is_same<T, Setting_t>::value)
    {
        EEPROM.put(STORAGE_ADDRESS_SETTINGS, data);
    }
    else
    {
        return false;
    }

    // EEPROM.commit();

    return true;
}