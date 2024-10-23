#include "hour_meter_manager.h"

HourMeter::HourMeter(uint8_t format_on_fail)
{
    if (LittleFS.begin(format_on_fail))
    {
        Serial.printf("Storage has been allocated\n");
        checkAndCreateFiles();
    }
    else
    {
        Serial.printf("Failed to allocate storage\n");
    }
}

HourMeter::~HourMeter()
{
}

void HourMeter::checkAndCreateFiles()
{
    // Check and create data.txt
    if (!LittleFS.exists("/data.txt"))
    {
        // Create data.txt and write initial value
        File file = LittleFS.open("/littlefs/data.txt", "w"); // Open for writing (creates if not exists)
        if (file)
        {
            file.println("0"); // Write initial value
            file.close();      // Close the file
            Serial.println("Created data.txt with initial value 0");
        }
        else
        {
            Serial.println("Failed to create data.txt");
        }
    }
    else
    {
        Serial.println("data.txt already exists");
    }

    // Check and create setting.txt
    if (!LittleFS.exists("/setting.txt"))
    {
        // Create setting.txt and write initial value
        File file = LittleFS.open("/littlefs/setting.txt", "w"); // Open for writing (creates if not exists)
        if (file)
        {
            file.println("0"); // Write initial value
            file.close();      // Close the file
            Serial.println("Created setting.txt with initial value 0");
        }
        else
        {
            Serial.println("Failed to create setting.txt");
        }
    }
    else
    {
        Serial.println("setting.txt already exists");
    }
}

// int32_t HourMeter::getSavedHourMeter()
// {
//     return savedHourMeter;
// }

time_t HourMeter::loadHMFromStorage()
{
    time_t hourMeter;
    // EEPROM.get(STORAGE_ADDRESS_HOUR_METER, hourMeter);

    Serial.println("about to open data.txt");
    FILE *file = fopen("/littlefs/data.txt", "r");
    if (file)
    {
        fscanf(file, "%ld", &hourMeter); // Read a single float value
        fclose(file);
    }
    else
    {
        Serial.println("Failed to open data.txt for reading");
    }

    return hourMeter;
}

Setting_t HourMeter::loadSettingFromStorage()
{
    Setting_t dummy{-1};
    return dummy;
}