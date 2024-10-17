#include "hour_meter_manager.h"

HourMeter::HourMeter(time_t& currentHourMeter, uint16_t storageSize)
{
    if (EEPROM.begin(512))
    {
        Serial.printf("Storage has been allocated for 512\n");

        time_t tempHourMeter;
        EEPROM.get(STORAGE_ADDRESS_HOUR_METER, tempHourMeter);

        currentHourMeter = tempHourMeter;

        Serial.printf("Your saved Hour Meter is %u\n", savedHourMeter);
    }
    else
    {
        Serial.printf("Failed to allocaate storage\n");
    }
}

HourMeter::~HourMeter()
{
}

// int32_t HourMeter::getSavedHourMeter()
// {
//     return savedHourMeter;
// }

time_t HourMeter::loadHMFromStorage()
{
    time_t hourMeter;
    EEPROM.get(STORAGE_ADDRESS_HOUR_METER, hourMeter);

    return hourMeter;
}

Setting_t HourMeter::loadSettingFromStorage()
{
    Setting_t dummy{-1};
    return dummy;
}