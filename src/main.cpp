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

/* DEKLARASI OBJEK YANG DIGUNAKAN TERSIMPAN DI HEAP */
RTC *rtc;
AnalogInput *ain;
GPS *gps;

/* FORWARD DECLARATION UNTUK FUNGSI-FUNGSI DI DEPAN*/
static void RTCDemo(void *pvParam);
static void dataAcquisition(void *pvParam);
static void sendBLEData(void *pvParam);
static void retrieveGPSData(void *pvParam);
static void sendToRS485(void *pvParam);
static void setCustomBeacon();

/* FORWARD DECLARATION UNTUK HANDLER DAN SEMAPHORE RTOS */
TaskHandle_t RTCDemoHandler = NULL;
TaskHandle_t dataAcquisitionHandler = NULL;
TaskHandle_t sendBLEDataHandler = NULL;
TaskHandle_t retrieveGPSHandler = NULL;
TaskHandle_t sendToRS485Handler = NULL;
SemaphoreHandle_t xSemaphore = NULL;

/* GLOBAL VARIABLES */
BLEAdvertising *pAdvertising;
BeaconData_t data;

// float analogInputVal = 0;
// GPSData_s Internalgps;
// float voltageSupply = 0;
// time_t lastTenth = 0;

void setup()
{
    /* SERIAL INIT */
    Serial.begin(115200);
    Serial.println("Mesin dinyalakan");

    /* RTC INIT */
    Serial.println("Inisialisasi RTC");
    rtc = new RTC();

    /* ANALOG INPUT INIT */
    Serial.println("Inisialisasi Analog Input");
    ain = new AnalogInput();
    data.voltageSupply = ain->readAnalogInput(AnalogPin::PIN_A0); // untuk membaca di pin_a0 (PIN 1 pada silkscreen)

    /* GPS INIT */
    Serial.println("Inisialisasi GPS");
    gps = new GPS();
    data.gps.latitude = 0;
    data.gps.longitude = 0;
    data.gps.status = 'A';

    xSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(xSemaphore);

    /* BLE INIT */
    BLEDevice::init("OMU BLE BEACON");
    BLEDevice::setPower(ESP_PWR_LVL_N12);
    pAdvertising = BLEDevice::getAdvertising();

    xTaskCreatePinnedToCore(RTCDemo, "RTC Demo", 2048, NULL, 3, &RTCDemoHandler, 1);
    xTaskCreatePinnedToCore(dataAcquisition, "Analog Demo", 2048, NULL, 3, &dataAcquisitionHandler, 1);
    xTaskCreatePinnedToCore(sendBLEData, "Send BLE Data", 2048, NULL, 3, &sendBLEDataHandler, 0);
    xTaskCreatePinnedToCore(retrieveGPSData, "get GPS Data", 2048, NULL, 3, &retrieveGPSHandler, 1);
    // xTaskCreatePinnedToCore(sendToRS485, "send data to RS485", 2048, NULL, 3, &sendToRS485Handler, 0);
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
            vTaskDelay(pdMS_TO_TICKS(100));
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

    // char customData[4]; // 2 bytes untu voltage, 2 bytes untuk current, 4 bytes untuk timestamp

    // customData[0] = 0x20;
    // customData[1] = 0x00;
    // customData[2] = (analogInputFixedPoint >> 8) & 0xFF;
    // customData[3] = (analogInputFixedPoint & 0xFF);

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
        // vTaskDelay(pdMS_TO_TICKS(3000)); // advertising selama 3 detik
        Serial.println("[BLE] Advertising...");
        // pAdvertising->stop();
        vTaskDelay(pdMS_TO_TICKS(1000)); // advertising selama 3 detik
    }
}

static void retrieveGPSData(void *pvParam)
{
    bool isValid = false;

    while (1)
    {
        isValid = gps->getValidation();

        if ((gps->getCharProcessed()) < 10)
        {
            Serial.println("GPS module not sending data, check wiring or module power");
        }
        else
        {
            if (isValid)
            {
                Serial.printf("Latitude : %f", data.gps.longitude);
                Serial.printf("Longitude : %f", data.gps.longitude);
            }
            else
            {
                Serial.println("GPS is searching for a signal...");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static void sendToRS485(void *pvParam)
{
    while (1)
    {
        // do something
    }
}