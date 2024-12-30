#pragma once

#include <Arduino.h>

#define PIN_RX_RS485     18
#define PIN_TX_RS485     19
#define PIN_DIGITAL_IN_1 33
#define PIN_DIGITAL_IN_2 25
#define PIN_DIGITAL_IN_3 26
#define PIN_DIGITAL_IN_4 27
#define PIN_TX_SERIAL2   17
#define PIN_RX_SERIAL2   16

// #define DEFAULT_ALTERNATOR_THRESHOLD 27.00
#define DEFAULT_ALTERNATOR_THRESHOLD 26.00

// New Way
struct GPSData_t {
  double longitude;
  double latitude;
  char status;
};

struct DozerData_t {
  float alternatorValue;
  bool neutralStatus;
  time_t alternatorHourMeter;
  bool rippingStatus;
  time_t rippingHourMeter;
  bool dozingStatus;
  time_t dozingHourMeter;
  std::string ID;  // maybe refactor this in the future
  std::string currentDate;
  std::string currentTime;
};

struct Setting_t {
  // TODO: Adjust the type data
  int32_t settingDevice;
  int32_t voltageThreshold;
  int32_t codeUnit;
  int32_t offsetThreshold;
};

enum class MachineState {
  NEUTRAL,
  ACTIVE,
};

enum class AlternatorState {
  ON,
  OFF,
};