#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>

// TODO: MERGE THIS INTO HOUR METER MANAGER CLASS

class RTC : public RTC_DS3231
{
public:
    RTC();
    ~RTC();

    void printRTCData(DateTime& startTime);
    std::string getCurrentTimeString(DateTime& time);
    std::string getCurrentDateString(DateTime& time);

private:
    // something
    // RTC_DS3231 _rtc;
    // DateTime startTime;
    char _daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
};
