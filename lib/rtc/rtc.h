#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>

class RTC
{
public:
    RTC();
    ~RTC();

    void printRTCData();

private:
    // something
    RTC_DS3231 _rtc;
    DateTime startTime;
    char _daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
};
