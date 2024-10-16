#include "hour_meter_manager.h"

HourMeter::HourMeter()
{
    // if Setting_t and savedHourMeter exist
    //      load from eeprom
    // else
    //      set the all the value to be default
}

HourMeter::~HourMeter()
{
}

int32_t HourMeter::getSavedHourMeter()
{
    return savedHourMeter;
}
int32_t HourMeter::saveToEEPROM()
{
    // TODO: Implement
}
int32_t HourMeter::loadFromEEPROM()
{
    // TODO: Impelement
}