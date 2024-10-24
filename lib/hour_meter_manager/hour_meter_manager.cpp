#include "hour_meter_manager.h"

HourMeter::HourMeter(std::string fileName, uint8_t formatOnFail)
{
    if (LittleFS.begin(formatOnFail))
    {
        Serial.printf("Storage has been allocated\n");
        checkAndCreateFiles(fileName);
    }
    else
    {
        Serial.printf("Failed to allocate storage\n");
    }
}

HourMeter::~HourMeter()
{
}

void HourMeter::checkAndCreateFiles(std::string fileName)
{
    std::string path = "/" + fileName;
    std::string fullPath = "/littlefs/" + fileName;

    // Check and create data.txt
    if (!LittleFS.exists(path.c_str()))
    {
        // Create data.txt and write initial value
        File file = LittleFS.open(fullPath.c_str(), "w"); // Open for writing (creates if not exists)
        if (file)
        {
            file.println("0"); // Write initial value
            file.close();      // Close the file
            Serial.printf("Created %s with initial value 0\n", path.c_str());
        }
        else
        {
            Serial.printf("Failed to create %s\n", path.c_str());
        }
    }
    else
    {
        Serial.printf("%s already exists\n", path.c_str());
    }

    // Check and create setting.txt
    // if (!LittleFS.exists("/setting.txt"))
    // {
    //     // Create setting.txt and write initial value
    //     File file = LittleFS.open("/littlefs/setting.txt", "w"); // Open for writing (creates if not exists)
    //     if (file)
    //     {
    //         file.println("0"); // Write initial value
    //         file.close();      // Close the file
    //         Serial.println("Created setting.txt with initial value 0");
    //     }
    //     else
    //     {
    //         Serial.println("Failed to create setting.txt");
    //     }
    // }
    // else
    // {
    //     Serial.println("setting.txt already exists");
    // }
}

time_t HourMeter::loadHMFromStorage(std::string path)
{
    time_t hourMeter;

    std::string fullPath = "/littlefs/" + path;
    // EEPROM.get(STORAGE_ADDRESS_HOUR_METER, hourMeter);

    FILE *file = fopen(fullPath.c_str(), "r");
    if (file)
    {
        fscanf(file, "%ld", &hourMeter); // Read a single float value
        fclose(file);
    }
    else
    {
        Serial.printf("Failed to open %s for reading\n", fullPath.c_str());
    }

    return hourMeter;
}

Setting_t HourMeter::loadSettingFromStorage()
{
    Setting_t dummy{-1};
    return dummy;
}

time_t convertHMToHourFormat(time_t seconds)
{
    return seconds / 3600;
}

time_t convertHMToMinuteFormat(time_t seconds)
{
    return seconds / 60;
}