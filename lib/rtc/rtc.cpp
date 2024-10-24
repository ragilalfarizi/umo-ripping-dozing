#include "rtc.h"

RTC::RTC()
{
    Serial.println("RTC Initialization");

    Wire.begin();

    if (!begin())
    {
        Serial.println("Couldn't find RTC");
        Serial.flush();
        while (1)
            delay(10);
    }

    if (lostPower())
    {
        Serial.println("RTC lost power, let's set the time!");
        // When time needs to be set on a new device, or after a power loss, the
        // following line sets the RTC to the date & time this sketch was compiled
        adjust(DateTime(F(__DATE__), F(__TIME__)));
        // This line sets the RTC with an explicit date & time, for example to set
        // January 21, 2014 at 3am you would call:
        // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    }
}

RTC::~RTC()
{
}

std::string RTC::getCurrentDateString(DateTime &time)
{
    std::string dateString, day, month, year;

    day = std::to_string(time.day());
    month = std::to_string(time.month());
    year = std::to_string(time.year());

    dateString = day + "/" + month + "/" + year;
    return dateString;
}

std::string RTC::getCurrentTimeString(DateTime &time)
{
    std::string timeString, hour, minute;

    hour = std::to_string(time.hour());
    minute = std::to_string(time.minute());

    timeString = hour + ":" + minute;
    return timeString;
}

void RTC::printRTCData(DateTime &startTime)
{
    Serial.print(startTime.year(), DEC);
    Serial.print('/');
    Serial.print(startTime.month(), DEC);
    Serial.print('/');
    Serial.print(startTime.day(), DEC);
    Serial.print(" (");
    Serial.print(_daysOfTheWeek[startTime.dayOfTheWeek()]);
    Serial.print(") ");
    Serial.print(startTime.hour(), DEC);
    Serial.print(':');
    Serial.print(startTime.minute(), DEC);
    Serial.print(':');
    Serial.print(startTime.second(), DEC);
    Serial.println();

    Serial.print(" since midnight 1/1/1970 = ");
    Serial.print(startTime.unixtime());
    Serial.print("s = ");
    Serial.print(startTime.unixtime() / 86400L);
    Serial.println("d");

    // calculate a date which is 7 days, 12 hours, 30 minutes, 6 seconds into the future
    DateTime future(startTime + TimeSpan(7, 12, 30, 6));

    Serial.print(" now + 7d + 12h + 30m + 6s: ");
    Serial.print(future.year(), DEC);
    Serial.print('/');
    Serial.print(future.month(), DEC);
    Serial.print('/');
    Serial.print(future.day(), DEC);
    Serial.print(' ');
    Serial.print(future.hour(), DEC);
    Serial.print(':');
    Serial.print(future.minute(), DEC);
    Serial.print(':');
    Serial.print(future.second(), DEC);
    Serial.println();

    Serial.print("Temperature: ");
    Serial.print(getTemperature());
    Serial.println(" C");

    // Serial.println();
    delay(3000);
}