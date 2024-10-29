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
static void alternatorCounter(void *pvParam);
static void rippingCounter(void *pvParam);
static void dozingCounter(void *pvParam);
void IRAM_ATTR onDI3Change();
void IRAM_ATTR onDI4Change();
void IRAM_ATTR onDIChange();
static void digitalInputInit();

/* FORWARD DECLARATION UNTUK HANDLER DAN SEMAPHORE RTOS */
TaskHandle_t getDateHandler           = NULL;
TaskHandle_t displayComHandler        = NULL;
TaskHandle_t alternatorCounterHandler = NULL;
TaskHandle_t rippingCounterHandler    = NULL;
TaskHandle_t dozingCounterHandler     = NULL;

/* GLOBAL VARIABLES */
HardwareSerial displayCom(2);
DozerData_t data;
DateTime now;
// TODO: Declare Setting_t

void setup() {
  /* SERIAL INIT */
  Serial.begin(9600);
  Serial.println("[Serial] Mesin dinyalakan");

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
  }

  /* ANALOG INPUT FOR ALTERNATOR INIT */
  Serial.println("[AIN] Inisialisasi Analog Input");
  ain                      = new AnalogInput();
  data.alternatorHourMeter = ain->readAnalogInput(
      AnalogPin::PIN_A3);  // untuk membaca di pin_a3 (PIN 4 pada silkscreen)
  // TODO: Check alternator status

  /* DIGITAL INPUT FOR RIPPING AND DOZING INIT */
  digitalInputInit();

  /* BLE INIT */
  // BLEDevice::init("OMU BLE BEACON");
  // BLEDevice::setPower(ESP_PWR_LVL_N12);
  // pAdvertising = BLEDevice::getAdvertising();

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
  // TODO: Print juga hour meter dalam jam

  // LOAD DEVICE ID
  // Sementara hard-code dulu
  data.ID = "CD001";

  /* DISPLAY COM INIT */
  // TODO: ganti jadi display com
  Serial.println("[485] Inisialisasi RS485");
  // modbus.begin(9600, SERIAL_8N1, PIN_RX_RS485, PIN_TX_RS485);
  displayCom.begin(9600, SERIAL_8N1, PIN_RX_SERIAL2, PIN_TX_SERIAL2);

  // xTaskCreatePinnedToCore(alternatorCounter, "Updating Alternator Hour
  // Meter", 2048, NULL, 3, &alternatorCounterHandler, 0);
  xTaskCreatePinnedToCore(rippingCounter, "Updating Ripping Hour Meter", 4096,
                          NULL, 2, &rippingCounterHandler, 0);
  xTaskCreatePinnedToCore(dozingCounter, "Updating Dozing Hour Meter", 4096,
                          NULL, 2, &dozingCounterHandler, 0);
  xTaskCreatePinnedToCore(getTimeAndDate, "get time and date", 2048, NULL, 0,
                          &getDateHandler, 0);
  xTaskCreatePinnedToCore(sendToDisplay, "send data to Display", 4096, NULL, 3,
                          &displayComHandler, 1);
  // xTaskCreatePinnedToCore(sendBLEData, "Send BLE
  // Data", 2048, NULL, 3, &sendBLEDataHandler, 0);
  // xTaskCreatePinnedToCore(retrieveGPSData, "get GPS Data", 2048, NULL, 4,
  // &retrieveGPSHandler, 1);
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
    snprintf(rippingBuffer, sizeof(rippingBuffer), "%ld",
             static_cast<long>(data.rippingHourMeter));
    snprintf(dozingBuffer, sizeof(dozingBuffer), "%ld",
             static_cast<long>(data.dozingHourMeter));

    displayCom.printf("DATA1,%s,%s,%s,%s,%s,\r\n", rippingBuffer, dozingBuffer,
                      data.ID.c_str(), data.currentDate.c_str(),
                      data.currentTime.c_str());

    Serial.println("data is sent to the display");

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

static void alternatorCounter(void *pvParam) {
  // TODO:
  // while(1)
  //      if(alternatorValue > Threshold)
  //          startCounting()
  //      else
  //          stopCouting()
}

static void rippingCounter(void *pvParam) {
  DateTime startTime, pollingTime;
  time_t runHour = 0;

  while (1) {
    // Menunggu notifikasi dari interrupt digital input 3
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    startTime = rtc->now();  // Ambil data waktu mulai
    Serial.printf("[HM] Ripping Counter start at %02d:%02d:%02d\n",
                  startTime.hour(), startTime.minute(), startTime.second());

    while (digitalRead(PIN_DIGITAL_IN_3) &&
           digitalRead(PIN_DIGITAL_IN_4) == LOW) {
      pollingTime = rtc->now();

      // Kalkulasi selisih polling dan start time
      runHour = static_cast<time_t>(pollingTime.secondstime() -
                                    startTime.secondstime());

      data.rippingHourMeter += runHour;
      Serial.printf("[HM] The total ripping hour : %ld s\n",
                    data.rippingHourMeter);

      // Menyimpan ke Storage
      if (ripping->saveToStorage(data.rippingHourMeter, "ripping.txt")) {
        Serial.println("[HM] total ripping hour is saved");
      } else {
        Serial.println("[HM] total ripping hour is failed to be saved");
      }

      startTime = pollingTime;  // Update start time

      vTaskDelay(pdMS_TO_TICKS(1000));
    }

    Serial.println("[HM] rippingCounter is stopped");

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

static void dozingCounter(void *pvParam) {
  DateTime startTime, pollingTime;
  time_t runHour = 0;

  while (1) {
    // Menunggu notifikasi dari interrupt digital input 3
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    startTime = rtc->now();  // Ambil data waktu mulai
    Serial.printf("[HM] Dozing Counter start at %02d:%02d:%02d\n",
                  startTime.hour(), startTime.minute(), startTime.second());

    while (digitalRead(PIN_DIGITAL_IN_4) == LOW) {
      pollingTime = rtc->now();

      // Kalkulasi selisih polling dan start time
      runHour = static_cast<time_t>(pollingTime.secondstime() -
                                    startTime.secondstime());

      data.dozingHourMeter += runHour;
      Serial.printf("[HM] The total dozing hour : %ld s\n",
                    data.dozingHourMeter);

      // Menyimpan ke Storage
      if (ripping->saveToStorage(data.dozingHourMeter, "dozing.txt")) {
        Serial.println("[HM] total dozing hour is saved");
      } else {
        Serial.println("[HM] total dozing hour is failed to be saved");
      }

      startTime = pollingTime;  // Update start time

      vTaskDelay(pdMS_TO_TICKS(1000));
    }

    Serial.println("[HM] dozingCounter is stopped");

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void IRAM_ATTR onDI3Change() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  xTaskNotifyFromISR(rippingCounterHandler, 0, eNoAction,
                     &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

void IRAM_ATTR onDI4Change() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  xTaskNotifyFromISR(dozingCounterHandler, 0, eNoAction,
                     &xHigherPriorityTaskWoken);

  if (xHigherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

void IRAM_ATTR onDIChange() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  // Check the state of digital inputs
  bool isDI3Active = (digitalRead(PIN_DIGITAL_IN_3) == LOW);  // Ripping trigger
  bool isDI4Active = (digitalRead(PIN_DIGITAL_IN_4) == LOW);  // Dozing trigger

  // if standby, return

  // Check alternator voltage and machine status
  // if (alternatorVoltage > 27.0 && machineStatus != MachineStatus::NEUTRAL) {
  if (isDI3Active && isDI4Active) {
    // Both DI3 and DI4 are active, start ripping task
    xTaskNotifyFromISR(rippingCounterHandler, 0, eNoAction,
                       &xHigherPriorityTaskWoken);
  } else if (isDI4Active) {
    // Only DI3 is active, start dozing task
    xTaskNotifyFromISR(dozingCounterHandler, 0, eNoAction,
                       &xHigherPriorityTaskWoken);
  }

  if (xHigherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

void digitalInputInit() {
  pinMode(PIN_DIGITAL_IN_3, INPUT);
  pinMode(PIN_DIGITAL_IN_4, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_DIGITAL_IN_3), onDIChange, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_DIGITAL_IN_4), onDIChange, FALLING);
}

/*==========================================================================*/
/* DEPRECIATED FUNCTIONS. MIGHT BE USED IN THE FUTURE */
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

// static void countingHourMeter(void *pvParam)
// {
//     DateTime startTime = rtc->now();
//     Serial.printf("[HM] Start Time : \n");

//     DateTime pollingTime;
//     time_t runHour, totalRunHour;

//     // time_t currentHourMeter = 900;
//     while (1)
//     {
//         pollingTime = rtc->now();

//         // Serial.printf("Polling Time: %ld s, Start Time: %lu s\n",
//         pollingTime.secondstime(), startTime.secondstime());

//         runHour = static_cast<time_t>(pollingTime.secondstime()) -
//         static_cast<time_t>(startTime.secondstime());
//         // BUG: this printf gives me big number. but the totalRunHour is
//         right
//         // Serial.printf("[HM] run Hour : %lu s\n");

//         totalRunHour = currentHourMeter + runHour;
//         Serial.printf("[HM] this machine has running hour of %ld s\n",
//         totalRunHour);

//         if (hm->saveToStorage(totalRunHour))
//         {
//             Serial.println("[HM] total run hour is saved to storage");
//         }
//         else
//         {
//             Serial.println("[HM] total run hour is failed to be saved");
//         }

//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
// }

/* =============================================== */
