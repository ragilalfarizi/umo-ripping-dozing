#pragma once

#include <Arduino.h>

// struct GPSData_s
// {
//     uint32_t lon_i_g;
//     uint32_t lat_i_g; // integer global
//     double lon_f_g; // float global
//     double lat_f_g;
//     bool lonValueNegative;
//     bool latValueNegative;
//     char GPSStatus;
// };

// New Way
struct GPSData_s
{
    // uint32_t lon_i_g;
    // uint32_t lat_i_g;
    // double lon_f_g;
    // double lat_f_g;
    double longitude;
    double latitude;
    // bool lonValueNegative;
    // bool latValueNegative;
    char status;
};

struct BeaconData_t {
    GPSData_s gps;
    float voltageSupply;
    time_t hourMeter;
};