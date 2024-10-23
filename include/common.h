#pragma once

#include <Arduino.h>

#define PIN_RX_RS485 18
#define PIN_TX_RS485 19

// New Way
struct GPSData_t
{
    double longitude;
    double latitude;
    char status;
};

struct BeaconData_t {
    GPSData_t gps;
    float voltageSupply;
    time_t hourMeter;
};

struct Setting_t
{
    // TODO: Adjust the type data
    int32_t settingDevice;
    int32_t voltageThreshold;
    int32_t codeUnit;
    int32_t offsetThreshold;
};
