#include "hour_meter_manager.h"
#include <unity.h>
#include "common.h"
#include <Wire.h>
#include <SPI.h>

HourMeter *hm;
time_t currentHourMeter = 0;

void setUp(void)
{
    // Serial.begin(9600);
    hm = new HourMeter(currentHourMeter, 512);
}

void tearDown(void)
{
    // Clear specific EEPROM addresses if needed, or clear all
    for (int i = 0; i < 512; ++i)
    {
        EEPROM.write(i, 0); // Clear EEPROM
    }
    EEPROM.commit();

    delete hm;
    hm = nullptr;
}

void test_canSaveHourMeterToStorage(void)
{
    time_t hourMeter = 64613123;

    TEST_ASSERT_TRUE(hm->saveToStorage(hourMeter));

    TEST_ASSERT_EQUAL(hourMeter, hm->loadHMFromStorage());

    hourMeter = 11111;

    TEST_ASSERT_TRUE(hm->saveToStorage(hourMeter));

    TEST_ASSERT_EQUAL(hourMeter, hm->loadHMFromStorage());
}

void test_canSyncHourMeterWithStorage(void)
{
    // // the currentHourMeter is 64613123

    // time_t savedCurrentMeter;
    // time_t newHourMeter = 120;
    // time_t updatedHourMeter;

    // EEPROM.get(STORAGE_ADDRESS_HOUR_METER, savedCurrentMeter);

    // // EEPROM.put(STORAGE_ADDRESS_HOUR_METER, newHourMeter);
    // hm->saveToStorage(newHourMeter);

    // TEST_ASSERT_EQUAL(newHourMeter, (updatedHourMeter - savedCurrentMeter));
    time_t hourMeter = 11400;

    TEST_ASSERT_TRUE(hm->saveToStorage(hourMeter));
    
    time_t newHourMeterInStorage;

    EEPROM.get(STORAGE_ADDRESS_HOUR_METER, newHourMeterInStorage);
    // TEST_ASSERT_EQUAL(currentHourMeter, hm-);
}

int runUnityTests(void)
{
    UNITY_BEGIN();
    // RUN_TEST(test_copySavedtoCurrentHourMeter);
    RUN_TEST(test_canSaveHourMeterToStorage);
    // RUN_TEST(test_canUpdateHourMeterToStorage);

    return UNITY_END();
}

void setup()
{
    delay(2000);

    runUnityTests();
}

void loop() {}