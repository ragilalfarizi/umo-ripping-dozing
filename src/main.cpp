#include "analog_input.h"
#include "rtc.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "NimBLEDevice.h"
#include "NimBLEBeacon.h"
#include "NimBLEAdvertising.h"
#include "NimBLEEddystoneURL.h"
#include "common.h"
#include "gps.h"
#include "hour_meter_manager.h"

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

/* FORWARD DECLARATION UNTUK HANDLER DAN SEMAPHORE RTOS */
TaskHandle_t getDateHandler = NULL;
TaskHandle_t displayComHandler = NULL;
TaskHandle_t alternatorCounterHandler = NULL;
TaskHandle_t rippingCounterHandler = NULL;
TaskHandle_t dozingCounterHandler = NULL;

/* GLOBAL VARIABLES */
HardwareSerial modbus(1);
DozerData_t data;
DateTime now;
// TODO: Declare Setting_t

void setup()
{
    /* SERIAL INIT */
    Serial.begin(9600);
    Serial.println("[Serial] Mesin dinyalakan");

    /* RTC INIT */
    Serial.println("[RTC] Inisialisasi RTC");
    rtc = new RTC();
    if (rtc == nullptr)
    {
        Serial.println("[ERROR] Failed to allocate memory for RTC object, retrying in 5 seconds...");
        vTaskDelay(pdMS_TO_TICKS(5000)); // Wait for 5 seconds before retrying
    }

    if (rtc->lostPower())
    {
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
    ain = new AnalogInput();
    data.alternatorHourMeter = ain->readAnalogInput(AnalogPin::PIN_A3); // untuk membaca di pin_a3 (PIN 4 pada silkscreen)
    // TODO: Check alternator status

    /* DIGITAL INPUT FOR RIPPING AND DOZING INIT */
    // TODO: set to input
    // TODO: attach interrupt

    /* BLE INIT */
    // BLEDevice::init("OMU BLE BEACON");
    // BLEDevice::setPower(ESP_PWR_LVL_N12);
    // pAdvertising = BLEDevice::getAdvertising();

    /* HOUR METER INIT */
    alternator = new HourMeter();
    data.alternatorHourMeter = alternator->loadHMFromStorage();
    Serial.printf("[HM] Hour Meter ALTERNATOR yang tersimpan adalah %ld\n", data.alternatorHourMeter);

    ripping = new HourMeter();
    data.rippingHourMeter = ripping->loadHMFromStorage();
    Serial.printf("[HM] Hour Meter RIPPING yang tersimpan adalah %ld\n", data.rippingHourMeter);

    dozing = new HourMeter();
    data.dozingHourMeter = dozing->loadHMFromStorage();
    Serial.printf("[HM] Hour Meter dozing yang tersimpan adalah %ld\n", data.dozingHourMeter);
    // TODO: Print juga hour meter dalam jam

    // LOAD DEVICE ID
    // Ssementara hard-code dulu
    data.ID = "CD001";

    /* DISPLAY COM INIT */
    // TODO: ganti jadi display com
    Serial.println("[485] Inisialisasi RS485");
    modbus.begin(9600, SERIAL_8N1, PIN_RX_RS485, PIN_TX_RS485);

    // xTaskCreatePinnedToCore(alternatorCounter, "Updating Alternator Hour Meter", 2048, NULL, 3, &alternatorCounterHandler, 0);
    // xTaskCreatePinnedToCore(rippingCounter, "Updating Ripping Hour Meter", 2048, NULL, 3, &rippingCounterHandler, 0);
    // xTaskCreatePinnedToCore(dozingCounter, "Updating Dozing Hour Meter", 2048, NULL, 3, &dozingCounterHandler, 0);
    // xTaskCreatePinnedToCore(sendToDisplay, "send data to Display", 2048, NULL, 3, &displayComHandler, 0);
    xTaskCreatePinnedToCore(getTimeAndDate, "get time and date", 2048, NULL, 3, &getDateHandler, 0);
    // xTaskCreatePinnedToCore(sendBLEData, "Send BLE Data", 2048, NULL, 3, &sendBLEDataHandler, 0);
    // xTaskCreatePinnedToCore(retrieveGPSData, "get GPS Data", 2048, NULL, 4, &retrieveGPSHandler, 1);
}

void loop()
{
}

static void getTimeAndDate(void *pvParam)
{
    DateTime date;

    while (1)
    {
        date = rtc->now();

        data.currentTime = rtc->getCurrentTimeString(date);
        data.currentDate = rtc->getCurrentDateString(date);

        Serial.printf("[TIME] Date : %s\n", data.currentDate.c_str());
        Serial.printf("[TIME] Time : %s\n", data.currentTime.c_str());

        vTaskDelay(pdMS_TO_TICKS(60 * 1000)); // delay for one minute
    }
}

static void sendToDisplay(void *pvParam)
{
    // TODO:
    // while(1)
    //      printf with modbus
    //      delay 1s
    while (1)
    {
        // modbus.printf("%c,%f,%f,%.2f", data.gps.status, data.gps.latitude, data.gps.longitude, data.voltageSupply);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void alternatorCounter(void *pvParam)
{
    // TODO:
    // while(1)
    //      if(alternatorValue > Threshold)
    //          startCounting()
    //      else
    //          stopCouting()
}

static void rippingCounter(void *pvParam)
{
    // TODO:
    // this whole things start/stop by an interrupt
    // while(1)
    //      startcouting()
    //      savecounting()
    //      delay 1s
}

static void dozingCounter(void *pvParam)
{
    // TODO:
    // this whole things start/stop by an interrupt
    // while(1)
    //      startcouting()
    //      savecounting()
    //      delay 1s
}

void IRAM_ATTR onDI3Change()
{
    // TODO:
    // if (digitalRead(PIN3) == LOW) //artinya aktif
    //      data.rippingStatus = ON
    //      vTaskResume()
    // else
    //      data.rippingStatus = OFF
    //      vTaskResume()
}

void IRAM_ATTR onDI4Change()
{
    // TODO:
    // if (digitalRead(PIN4) == LOW) //artinya aktif
    //      data.rippingStatus = ON
    //      vTaskResume()
    // else
    //      data.rippingStatus = OFF
    //      vTaskResume()
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

//         // Serial.printf("Polling Time: %ld s, Start Time: %lu s\n", pollingTime.secondstime(), startTime.secondstime());

//         runHour = static_cast<time_t>(pollingTime.secondstime()) - static_cast<time_t>(startTime.secondstime());
//         // BUG: this printf gives me big number. but the totalRunHour is right
//         // Serial.printf("[HM] run Hour : %lu s\n");

//         totalRunHour = currentHourMeter + runHour;
//         Serial.printf("[HM] this machine has running hour of %ld s\n", totalRunHour);

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
