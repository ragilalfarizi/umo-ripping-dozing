#pragma once

#include <Arduino.h>
#include <EEPROM.h>
#include <LittleFS.h>
#include <rtc.h>

#include "common.h"

// WARN: DEPRECIATED
#define STORAGE_ADDRESS_HOUR_METER 0  // Size of Hour Meter is 4 bytes
#define STORAGE_ADDRESS_SETTINGS   4  // Size of Settings is 4 * 4 bytes
#define FORMAT_LITTLEFS_IF_FAILED  true

class HourMeter {
 public:
  HourMeter(std::string fileName = "data.txt",
            uint8_t formatOnFail = FORMAT_LITTLEFS_IF_FAILED);

  ~HourMeter();

  int32_t getSavedHourMeter();

  template <typename T>
  bool saveToStorage(const T &data, std::string path);

  time_t loadHMFromStorage(std::string path = "data.txt");

  Setting_t loadSettingFromStorage();

  void printStorage();

  time_t convertHMToHourFormat(time_t seconds);

  time_t convertHMToMinuteFormat(time_t seconds);

  void resetHourMeter();

 private:
  void checkAndCreateFiles(std::string fileName);
};

template <typename T>
bool HourMeter::saveToStorage(const T &data, std::string path) {
  std::string fullPath = "/" + path;

  if (std::is_same<T, time_t>::value) {
    File file = LittleFS.open(fullPath.c_str(), "w");
    if (file) {
      // Write the latest value in JSON format
      file.printf("%ld", data);
      file.close();
    } else {
      Serial.printf("Failed to open %s for writing\n", path.c_str());
      return false;
    }
  }

  else if (std::is_same<T, Setting_t>::value) {
    EEPROM.put(STORAGE_ADDRESS_SETTINGS, data);
  } else {
    return false;
  }

  // EEPROM.commit();

  return true;
}