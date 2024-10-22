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
GPS *gps;
HourMeter *hm;

/* FORWARD DECLARATION UNTUK FUNGSI-FUNGSI DI DEPAN*/
static void RTCDemo(void *pvParam);
static void dataAcquisition(void *pvParam);
static void sendBLEData(void *pvParam);
static void retrieveGPSData(void *pvParam);
static void sendToRS485(void *pvParam);
static void countingHourMeter(void *pvParam);
static void serialConfig(void *pvParam);
static void setCustomBeacon();

/* FORWARD DECLARATION UNTUK HANDLER DAN SEMAPHORE RTOS */
// TaskHandle_t RTCDemoHandler = NULL;
TaskHandle_t dataAcquisitionHandler = NULL;
TaskHandle_t sendBLEDataHandler = NULL;
TaskHandle_t retrieveGPSHandler = NULL;
TaskHandle_t sendToRS485Handler = NULL;
TaskHandle_t countingHMHandler = NULL;
SemaphoreHandle_t xSemaphore = NULL;

/* GLOBAL VARIABLES */
BLEAdvertising *pAdvertising;
BeaconData_t data;
HardwareSerial modbus(1);
time_t currentHourMeter;
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

    /* ANALOG INPUT INIT */
    Serial.println("[AIN] Inisialisasi Analog Input");
    ain = new AnalogInput();
    data.voltageSupply = ain->readAnalogInput(AnalogPin::PIN_A0); // untuk membaca di pin_a0 (PIN 1 pada silkscreen)

    /* GPS INIT */
    Serial.println("[GPS] Inisialisasi GPS");
    gps = new GPS();
    data.gps.latitude = 0;
    data.gps.longitude = 0;
    data.gps.status = 'A';

    /* RS485 INIT */
    Serial.println("[485] Inisialisasi RS485");
    modbus.begin(9600, SERIAL_8N1, PIN_RX_RS485, PIN_TX_RS485);

    xSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(xSemaphore);

    /* BLE INIT */
    BLEDevice::init("OMU BLE BEACON");
    BLEDevice::setPower(ESP_PWR_LVL_N12);
    pAdvertising = BLEDevice::getAdvertising();

    hm = new HourMeter(currentHourMeter);
    currentHourMeter = hm->loadHMFromStorage();
    Serial.printf("[HM] Hour Meter yang tersimpan adalah %ld\n", currentHourMeter);
    // TODO: Print juga hour meter dalam jam

    // xTaskCreatePinnedToCore(RTCDemo, "RTC Demo", 2048, NULL, 3, &RTCDemoHandler, 1); // TODO: Depreciating
    xTaskCreatePinnedToCore(dataAcquisition, "Data Acquisition", 4096, NULL, 3, &dataAcquisitionHandler, 1);
    xTaskCreatePinnedToCore(sendBLEData, "Send BLE Data", 2048, NULL, 3, &sendBLEDataHandler, 0);
    xTaskCreatePinnedToCore(retrieveGPSData, "get GPS Data", 2048, NULL, 4, &retrieveGPSHandler, 1);
    xTaskCreatePinnedToCore(sendToRS485, "send data to RS485", 2048, NULL, 3, &sendToRS485Handler, 0);
    xTaskCreatePinnedToCore(countingHourMeter, "Updating Hour Meter", 8192, NULL, 3, &countingHMHandler, 0);
}

void loop()
{
}

static void RTCDemo(void *pvParam)
{
    while (1)
    {
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY))
        {
            Serial.println();
            Serial.printf("============================================\n");
            rtc->printRTCData();
            Serial.printf("============================================\n");

            xSemaphoreGive(xSemaphore);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

static void dataAcquisition(void *pvParam)
{
    while (1)
    {
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY))
        {
            data.gps.latitude = gps->getlatitude();
            data.gps.longitude = gps->getLongitude();
            data.voltageSupply = ain->readAnalogInput(AnalogPin::PIN_A0);

            Serial.printf("============================================\n");
            Serial.printf("GPS STATUS\t\t= %c\n", data.gps.status);
            Serial.printf("GPS LATITUDE\t\t= %f\n", data.gps.latitude);
            Serial.printf("GPS LONGITUDE\t\t= %f\n", data.gps.longitude);
            Serial.printf("Analog Input\t\t= %.2f\n", data.voltageSupply);
            Serial.printf("============================================\n");

            xSemaphoreGive(xSemaphore);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

static void setCustomBeacon()
{
    // atur data advertising
    BLEAdvertisementData oAdvertisementData = BLEAdvertisementData();
    BLEAdvertisementData oScanResponseData = BLEAdvertisementData();

    const uint16_t beaconUUID = 0xFEAA;
    oScanResponseData.setFlags(0x06); // GENERAL_DISC_MODE 0x02 | BR_EDR_NOT_SUPPORTED 0x04
    oScanResponseData.setCompleteServices(BLEUUID(beaconUUID));

    // uint16_t voltage = random(2800, 3700); // dalam millivolts
    // analogInputVal = ain->readAnalogInput(AnalogPin::PIN_A1);
    // float current = 1.5;             // dalam ampere
    // uint32_t timestamp = 1678801234; // contoh Unix TimeStamp

    // Convert current to a 16-bit fixed-point format (e.g., 1.5 A -> 384 in 8.8 format)
    // Konversi arus ke format (contoh: 1.5 A -> 384 di format 8.8)
    // int16_t currentFixedPoint = (int16_t)(current * 256);
    // int16_t analogInputFixedPoint = (int16_t)(analogInputVal * 256);

    /* PROCESSING DATA SEBELUM DIKIRIM MELALUI BLE */
    char beacon_data[15];
    uint16_t volt = data.voltageSupply * 1000; // 3300mV = 3.3V
    int32_t latitudeFixedPoint = (int32_t)(data.gps.latitude * 256);
    int32_t longitudeFixedPoint = (int32_t)(data.gps.longitude * 256);

    beacon_data[0] = 0x01; // Eddystone Frame Type (Unencrypted Eddystone-TLM)
    beacon_data[1] = 0x00; // TLM version
    beacon_data[2] = 0x01;
    beacon_data[3] = (volt >> 8);                                // Battery voltage, 1 mV/bit i.e. 0xCE4 = 3300mV = 3.3V
    beacon_data[4] = (volt & 0xFF);                              //
    beacon_data[5] = 0;                                          // Eddystone Frame Type (Unencrypted Eddystone-TLM)
    beacon_data[6] = data.gps.status;                            //
    beacon_data[7] = ((longitudeFixedPoint & 0xFF000000) >> 24); //
    beacon_data[8] = ((longitudeFixedPoint & 0xFF0000) >> 16);   //
    beacon_data[9] = ((longitudeFixedPoint & 0xFF00) >> 8);      //
    beacon_data[10] = (longitudeFixedPoint & 0xFF);              //
    beacon_data[11] = ((latitudeFixedPoint & 0xFF000000) >> 24); //
    beacon_data[12] = ((latitudeFixedPoint & 0xFF0000) >> 16);   //
    beacon_data[13] = ((latitudeFixedPoint & 0xFF00) >> 8);      //
    beacon_data[14] = (latitudeFixedPoint & 0xFF);               //
    // beacon_data[15] = (((lastTenth / 10) & 0xFF000000) >> 24);    //
    // beacon_data[16] = (((lastTenth / 10) & 0xFF0000) >> 16);      //
    // beacon_data[17] = (((lastTenth / 10) & 0xFF00) >> 8);         //
    // beacon_data[18] = ((lastTenth / 10) & 0xFF);                  //

    oScanResponseData.setServiceData(BLEUUID(beaconUUID), std::string(beacon_data, sizeof(beacon_data)));
    oAdvertisementData.setName("OMU Demo Data");
    pAdvertising->setAdvertisementData(oAdvertisementData);
    pAdvertising->setScanResponseData(oScanResponseData);
}

static void sendBLEData(void *pvParam)
{

    while (1)
    {
        setCustomBeacon();
        pAdvertising->start();
        Serial.println("[BLE] Advertising...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * WARNING: Haven't fully tested. It should be working.
 *          But when I tested it, it took too long to retreive
 *          GPS Data.
 * */
static void retrieveGPSData(void *pvParam)
{
    bool isValid = false;

    while (1) // void loop
    {
        Serial.println("[GPS] encoding...");

        while (Serial.available() > 0)
        {
            char gpsChar = Serial.read();
            gps->encode(gpsChar);
        }

        isValid = gps->getValidation();

        if ((gps->getCharProcessed()) < 10)
        {
            Serial.println("[GPS] GPS module not sending data, check wiring or module power");
        }
        else
        {
            if (isValid)
            {
                Serial.printf("[GPS] Latitude : %f", data.gps.longitude);
                Serial.printf("[GPS] Longitude : %f", data.gps.longitude);
            }
            else
            {
                Serial.println("[GPS] GPS is searching for a signal...");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static void sendToRS485(void *pvParam)
{

    while (1)
    {
        modbus.printf("%c,%f,%f,%.2f", data.gps.status, data.gps.latitude, data.gps.longitude, data.voltageSupply);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief   : Task to receive new config and update them.
 *            Default state is not running. Only the interrupt
 *            from RS485 input will make it run.
 * @param   : none
 * @retval  : none
 */
static void serialConfig(void *pvParam)
{
    while (1)
    {
        // TODO: Print RS485 input
        // TODO: Parse
        // TODO: update to EEPROM
    }
}

static void countingHourMeter(void *pvParam)
{
    Serial.println("[DEBUG] BARU MULAI COUNTING HOUR METER");
    DateTime startTime = rtc->now();
    Serial.printf("[HM] Start Time : \n");

    DateTime pollingTime;
    time_t runHour, totalRunHour;

    // time_t currentHourMeter = 900;
    while (1)
    {
        pollingTime = rtc->now();

        // Serial.printf("Polling Time: %ld s, Start Time: %lu s\n", pollingTime.secondstime(), startTime.secondstime());

        runHour = static_cast<time_t>(pollingTime.secondstime()) - static_cast<time_t>(startTime.secondstime());
        // BUG: this printf gives me big number. but the totalRunHour is right
        // Serial.printf("[HM] run Hour : %lu s\n"); 

        totalRunHour = currentHourMeter + runHour;
        Serial.printf("[HM] this machine has running hour of %ld s\n", totalRunHour);
        // TODO: save to eeprom

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}