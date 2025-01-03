#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "NimBLEAdvertising.h"
#include "NimBLEBeacon.h"
#include "NimBLEDevice.h"
#include "NimBLEEddystoneURL.h"
#include "analog_input.h"
#include "common.h"
#include "gps.h"
#include "hour_meter_manager.h"
#include "rtc.h"

/* DEKLARASI OBJEK YANG DIGUNAKAN TERSIMPAN DI HEAP */
RTC *rtc;
AnalogInput *ain;
HourMeter *alternator;
HourMeter *ripping;
HourMeter *dozing;

/* FORWARD DECLARATION UNTUK FUNGSI-FUNGSI DI DEPAN*/
static void getTimeAndDate(void *pvParam);
static void sendToDisplay(void *pvParam);
static void alternatorMonitoring(void *pvParam);
static void rippingCounter(void *pvParam);
static void dozingCounter(void *pvParam);
static void neutralMonitoring(void *pvParam);
static void printDebug(void *pvParam);
static void digitalInputInit();

/* FORWARD DECLARATION UNTUK HANDLER DAN SEMAPHORE RTOS */
TaskHandle_t getDateHandler              = NULL;
TaskHandle_t displayComHandler           = NULL;
TaskHandle_t alternatorMonitoringHandler = NULL;
TaskHandle_t rippingCounterHandler       = NULL;
TaskHandle_t dozingCounterHandler        = NULL;
TaskHandle_t neutralMonitoringHandler    = NULL;
TaskHandle_t printDebugHandler           = NULL;

/* GLOBAL VARIABLES */
HardwareSerial displayCom(2);
DozerData_t data;
DateTime now;
MachineState machineState;
AlternatorState alternatorState;
// TODO: Declare Setting_t

void setup() {
  /* SERIAL INIT */
  Serial.begin(9600);
  Serial.println("[Serial] Mesin dinyalakan");

  Serial.printf("===================\n");
  Serial.printf(
      "Mode: DIGITAL INPUT ACTIVE WHEN THERE IS VOLTAGE INPUT. (Active "
      "HIGH)\n");
  // Serial.printf("Mode: DIGITAL INPUT ACTIVE WHEN THERE IS VOLTAGE INPUT.
  // (Active LOW)\n");
  Serial.printf("===================\n");

  /* RTC INIT */
  Serial.println("[RTC] Inisialisasi RTC");
  rtc = new RTC();
  if (rtc == nullptr) {
    Serial.println(
        "[ERROR] Failed to allocate memory for RTC object, retrying in 5 "
        "seconds...");
    vTaskDelay(pdMS_TO_TICKS(5000));  // Wait for 5 seconds before retrying
  }

  if (rtc->lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc->adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));

    Serial.println("FIRMWARE VERSI ACTIVE LOW");
  }

  /* ANALOG INPUT FOR ALTERNATOR INIT */
  Serial.println("[AIN] Inisialisasi Analog Input");
  ain                      = new AnalogInput();
  data.alternatorHourMeter = ain->readAnalogInput(
      AnalogPin::PIN_A0);  // To read from A0 (PIN 1 on the silkscreen)

  /* DIGITAL INPUT FOR RIPPING AND DOZING INIT */
  digitalInputInit();

  machineState    = MachineState::NEUTRAL;
  alternatorState = AlternatorState::OFF;

  /* HOUR METER INIT */
  alternator               = new HourMeter("alternator.txt");
  data.alternatorHourMeter = alternator->loadHMFromStorage("alternator.txt");
  Serial.printf("[HM] Hour Meter ALTERNATOR yang tersimpan adalah %ld\n",
                data.alternatorHourMeter);

  ripping               = new HourMeter("ripping.txt");
  data.rippingHourMeter = ripping->loadHMFromStorage("ripping.txt");
  Serial.printf("[HM] Hour Meter RIPPING yang tersimpan adalah %ld\n",
                data.rippingHourMeter);

  dozing               = new HourMeter("dozing.txt");
  data.dozingHourMeter = dozing->loadHMFromStorage("dozing.txt");
  Serial.printf("[HM] Hour Meter DOZING yang tersimpan adalah %ld\n",
                data.dozingHourMeter);

  // LOAD DEVICE ID
  // The ID IS HARD-CODED FOR A WHILE
  data.ID = "CD001";

  /* DISPLAY COM INIT */
  // TODO: ganti jadi display com
  Serial.println("[485] Inisialisasi RS485");
  displayCom.begin(9600, SERIAL_8N1, PIN_RX_SERIAL2, PIN_TX_SERIAL2);

  xTaskCreatePinnedToCore(rippingCounter, "Updating Ripping Hour Meter", 4096,
                          NULL, 2, &rippingCounterHandler, 0);
  xTaskCreatePinnedToCore(dozingCounter, "Updating Dozing Hour Meter", 4096,
                          NULL, 2, &dozingCounterHandler, 0);
  xTaskCreatePinnedToCore(getTimeAndDate, "get time and date", 2048, NULL, 0,
                          &getDateHandler, 0);
  xTaskCreatePinnedToCore(sendToDisplay, "send data to Display", 4096, NULL, 3,
                          &displayComHandler, 1);
  xTaskCreatePinnedToCore(alternatorMonitoring, "Alternator Monitor", 2048,
                          NULL, 2, &alternatorMonitoringHandler, 0);
  xTaskCreatePinnedToCore(neutralMonitoring, "Neutral Monitor", 2048, NULL, 2,
                          &neutralMonitoringHandler, 0);
  xTaskCreatePinnedToCore(printDebug, "Print Debug", 2048, NULL, 3,
                          &printDebugHandler, 0);
}

void loop() {}

static void getTimeAndDate(void *pvParam) {
  DateTime date;

  while (1) {
    date = rtc->now();

    data.currentTime = rtc->getCurrentTimeString(date);
    data.currentDate = rtc->getCurrentDateString(date);

    // sprintf(data.currentTime, )

    Serial.printf("[TIME] Date : %s\n", data.currentDate.c_str());
    Serial.printf("[TIME] Time : %s\n", data.currentTime.c_str());

    vTaskDelay(pdMS_TO_TICKS(60 * 1000));  // delay for one minute
  }
}

static void sendToDisplay(void *pvParam) {
  while (1) {
    // Define character buffers for the hour meters
    char rippingBuffer[20];
    char dozingBuffer[20];

    // Convert time_t values to C-strings using snprintf
    snprintf(rippingBuffer, sizeof(rippingBuffer), "%.2f",
             (data.rippingHourMeter / 3600.0));
    snprintf(dozingBuffer, sizeof(dozingBuffer), "%.2f",
             (data.dozingHourMeter / 3600.0));

    displayCom.printf("DATA1,%s,%s,%s,%s,%s,\r\n", rippingBuffer, dozingBuffer,
                      data.ID.c_str(), data.currentDate.c_str(),
                      data.currentTime.c_str());

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

static void alternatorMonitoring(void *pvParam) {
  while (1) {
    data.alternatorValue = ain->readAnalogInput(AnalogPin::PIN_A0);

    if (data.alternatorValue > DEFAULT_ALTERNATOR_THRESHOLD) {
      alternatorState = AlternatorState::ON;
    } else {
      alternatorState = AlternatorState::OFF;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

static void rippingCounter(void *pvParam) {
  DateTime startTime, currentTime, previousTime;
  int8_t intervalRaw  = 0;
  time_t intervalTime = 0;
  // time_t runTimeAccrued = 0;
  bool counting = false;  // Tracks whether counting is active

  while (1) {
    // Poll the conditions
    bool isAlternatorOn  = (alternatorState == AlternatorState::ON);
    bool isMachineActive = (machineState == MachineState::ACTIVE);

    // NOTE: on isDI4Active - isDI3Active
    // Active High -> LOW - LOW
    // Active Low -> HIGH - HIGH
    bool isDI4Active = (digitalRead(PIN_DIGITAL_IN_4) == LOW);
    bool isDI3Active = (digitalRead(PIN_DIGITAL_IN_3) == LOW);

    if (isAlternatorOn && isMachineActive && isDI4Active && isDI3Active) {
      // All conditions are met; start or continue counting
      if (!counting) {
        startTime = rtc->now();
        Serial.printf("[HM] Ripping Counter started at %02d:%02d:%02d\n",
                      startTime.hour(), startTime.minute(), startTime.second());
        counting     = true;
        previousTime = startTime;
      }

      currentTime = rtc->now();

      // NOTE: DEPRECIATED
      // runTimeAccrued = static_cast<time_t>(currentTime.secondstime() -
      //                                      startTime.secondstime());

      intervalRaw =
          (int8_t)(currentTime.secondstime() - previousTime.secondstime());

      if (intervalRaw > 11 || intervalRaw < 0) {
        intervalTime = 10;
      } else {
        intervalTime = (time_t)abs(intervalRaw);
      }

      data.rippingHourMeter += intervalTime;

      previousTime = currentTime;

      Serial.printf("[HM] Total ripping time: %ld s\n", data.rippingHourMeter);

      // Save the ripping time to storage
      if (ripping->saveToStorage(data.rippingHourMeter, "ripping.txt")) {
        Serial.println("[HM] Ripping time saved to storage.");
      } else {
        Serial.println("[HM] Failed to save ripping time.");
      }

      startTime = currentTime;  // Update start time for the next interval
    } else if (counting) {
      // One of the conditions is no longer met; stop counting
      Serial.println("[HM] Ripping Counter stopped.");
      counting = false;
    }

    vTaskDelay(pdMS_TO_TICKS(10000));  // Check conditions every second
  }
}

static void dozingCounter(void *pvParam) {
  DateTime startTime, currentTime, previousTime;
  int8_t intervalRaw  = 0;
  time_t intervalTime = 0;
  // time_t runTimeAccrued = 0;
  bool counting = false;  // Tracks whether counting is active

  while (1) {
    // Poll the conditions
    bool isAlternatorOn  = (alternatorState == AlternatorState::ON);
    bool isMachineActive = (machineState == MachineState::ACTIVE);

    // NOTE: on isDI4Active - isDI3NotActive
    // Active High -> LOW - HIGH
    // Active Low -> HIGH - LOW
    bool isDI4Active    = (digitalRead(PIN_DIGITAL_IN_4) == LOW);
    bool isDI3NotActive = (digitalRead(PIN_DIGITAL_IN_3) == HIGH);

    if (isAlternatorOn && isMachineActive && isDI4Active && isDI3NotActive) {
      // All conditions are met; start or continue counting
      if (!counting) {
        startTime = rtc->now();
        Serial.printf("[HM] Dozing Counter started at %02d:%02d:%02d\n",
                      startTime.hour(), startTime.minute(), startTime.second());
        counting = true;

        previousTime = startTime;
      }

      currentTime = rtc->now();

      // NOTE: DEPRECIATED
      // runTimeAccrued = static_cast<time_t>(currentTime.secondstime() -
      //                                      startTime.secondstime());

      intervalRaw =
          (int8_t)(currentTime.secondstime() - previousTime.secondstime());
      if (intervalRaw > 11 || intervalRaw < 0) {
        intervalTime = 10;
      } else {
        intervalTime = (time_t)abs(intervalRaw);
      }

      data.dozingHourMeter += intervalTime;

      previousTime = currentTime;

      Serial.printf("[HM] Total dozing time: %ld s\n", data.dozingHourMeter);

      // Save the dozing time to storage
      if (ripping->saveToStorage(data.dozingHourMeter, "dozing.txt")) {
        Serial.println("[HM] Dozing time saved to storage.");
      } else {
        Serial.println("[HM] Failed to save dozing time.");
      }

      startTime = currentTime;  // Update start time for the next interval
    } else if (counting) {
      // One of the conditions is no longer met; stop counting
      Serial.println("[HM] Dozing Counter stopped.");
      counting = false;
    }

    vTaskDelay(pdMS_TO_TICKS(10000));  // Check conditions every second
  }
}

static void neutralMonitoring(void *pvParam) {
  while (1) {
    // NOTE: on digitalRead(PIN_DIGITAL_IN_2)
    // Active High -> LOW
    // Active Low -> HIGH
    if (digitalRead(PIN_DIGITAL_IN_2) == LOW) {
      machineState = MachineState::ACTIVE;
    } else {
      machineState = MachineState::NEUTRAL;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void digitalInputInit() {
  pinMode(PIN_DIGITAL_IN_3, INPUT_PULLUP);
  pinMode(PIN_DIGITAL_IN_4, INPUT_PULLUP);
  pinMode(PIN_DIGITAL_IN_2, INPUT_PULLUP);
}

static void printDebug(void *pvParam) {
  while (1) {
    Serial.println("===================================================");
    Serial.printf("ID\t\t\t: %s\n", data.ID.c_str());
    Serial.printf("Alternator Status\t: %s\n",
                  (alternatorState == AlternatorState::ON) ? "ON" : "OFF");
    Serial.printf("Alternator\t\t: %.2f V\n", data.alternatorValue);
    Serial.printf(
        "Machine Status\t\t: %s\n",
        (machineState == MachineState::ACTIVE) ? "ACTIVE" : "NEUTRAL");
    Serial.printf("Dozing HM\t\t: %ld s\n", data.dozingHourMeter);
    Serial.printf("Ripping HM\t\t: %ld s\n", data.rippingHourMeter);
    Serial.println("===================================================");

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

/*==========================================================================*/
/* FUNCTIONS THAT MIGHT BE USED IN THE FUTURE */
/*==========================================================================*/

/**
 * @brief   : Task to receive new config and update them.
 *            Default state is not running. Only the interrupt
 *            from RS485 input will make it run.
 * @param   : none
 * @retval  : none
 */
// static void serialConfig(void *pvParam)
// {
//     while (1)
//     {
//         // TODO: Print RS485 input
//         // TODO: Parse
//         // TODO: update to EEPROM
//     }
// }
/* =============================================== */
