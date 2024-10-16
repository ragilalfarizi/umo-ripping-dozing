#pragma once

#include <Arduino.h>
#include <TinyGPSPlus.h>

class GPS
{
public:
    GPS();
    ~GPS();
    double getLongitude();
    double getlatitude();
    bool getValidation();
    int32_t getCharProcessed();
    void encode(char c);

private:
    TinyGPSPlus _gps;
};
