#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <memory>

#include "rtc.h"

RTC *rtc;

void setup()
{
    /* SERIAL INIT */
    Serial.begin(115200);

    /* RTC INIT */
    rtc = new RTC();

    Serial.println("setup done");
}

void loop()
{
    // Serial.println("Halo World!");
    rtc->printRTCData();
    delay(1000);
}