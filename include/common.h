#pragma once

#include <Arduino.h>

// New Way
struct GPSData_s
{
    double longitude;
    double latitude;
    char status;
};

struct BeaconData_t {
    GPSData_s gps;
    float voltageSupply;
    time_t hourMeter;
};