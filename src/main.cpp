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
static void analogDemo(void *pvParam);
static void sendBLEData(void *pvParam);
static void retrieveGPSData(void *pvParam);
static void sendToRS485(void *pvParam);
static void setCustomBeacon();

TaskHandle_t RTCDemoHandler = NULL;
TaskHandle_t analogDemoHandler = NULL;
TaskHandle_t sendBLEDataHandler = NULL;
TaskHandle_t retrieveGPSHandler = NULL;
TaskHandle_t sendToRS485Handler = NULL;
SemaphoreHandle_t xSemaphore = NULL;

/* GLOBAL VARIABLES */
BLEAdvertising *pAdvertising;
float analogInputVal = 0;
GPSData_s Internalgps;
float voltageSupply = 0;
time_t lastTenth = 0;

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

    /* GPS INIT */
    Serial.println("Inisialisasi GPS");
    gps = new GPS();

    xSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(xSemaphore);

    /* BLE INIT */
    // Buat BLE Device
    BLEDevice::init("OMU BLE BEACON");
    BLEDevice::setPower(ESP_PWR_LVL_N12);
    pAdvertising = BLEDevice::getAdvertising();

    xTaskCreatePinnedToCore(RTCDemo, "RTC Demo", 2048, NULL, 3, &RTCDemoHandler, 0);
    xTaskCreatePinnedToCore(analogDemo, "Analog Demo", 2048, NULL, 3, &analogDemoHandler, 0);
    xTaskCreatePinnedToCore(sendBLEData, "Send BLE Data", 2048, NULL, 3, &sendBLEDataHandler, 1);
    xTaskCreatePinnedToCore(retrieveGPSData, "get GPS Data", 2048, NULL, 3, &retrieveGPSHandler, 0);
    // xTaskCreatePinnedToCore(sendToRS485, "send data to RS485", 2048, NULL, 3, &sendToRS485Handler, 1);
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
            rtc->printRTCData();

            xSemaphoreGive(xSemaphore);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

static void analogDemo(void *pvParam)
{
    while (1)
    {
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY))
        {
            ain->printRawAnalogInputValue();

            xSemaphoreGive(xSemaphore);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

static void setCustomBeacon()
{
    /**
     * Contoh : mau mengirimkan data-data sebagai berikut.
     *
     * Voltage: 3300 mV (2 bytes)
     * Current: 1.5 A (2 bytes, fixed-point)
     * Timestamp: 1678801234 (Unix timestamp, 4 bytes)
     * Total size: ~8 bytes (well within BLE’s advertisement limits).
     * */

    char beacon_data[25];
    uint16_t volt = voltageSupply * 1000; // 3300mV = 3.3V
    const uint16_t beaconUUID = 0xFEAA;

    // atur data advertising
    BLEAdvertisementData oAdvertisementData = BLEAdvertisementData();
    BLEAdvertisementData oScanResponseData = BLEAdvertisementData();

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

    beacon_data[0] = 0x01; // Eddystone Frame Type (Unencrypted Eddystone-TLM)
    beacon_data[1] = 0x00; // TLM version
    beacon_data[2] = 0x01;
    beacon_data[3] = (volt >> 8);                                 // Battery voltage, 1 mV/bit i.e. 0xCE4 = 3300mV = 3.3V
    beacon_data[4] = (volt & 0xFF);                               //
    beacon_data[5] = 0;                                           // Eddystone Frame Type (Unencrypted Eddystone-TLM)
    beacon_data[6] = Internalgps.GPSStatus;                       //
    beacon_data[7] = ((Internalgps.lon_i_g & 0xFF000000) >> 24);  //
    beacon_data[8] = ((Internalgps.lon_i_g & 0xFF0000) >> 16);    //
    beacon_data[9] = ((Internalgps.lon_i_g & 0xFF00) >> 8);       //
    beacon_data[10] = (Internalgps.lon_i_g & 0xFF);               //
    beacon_data[11] = ((Internalgps.lat_i_g & 0xFF000000) >> 24); //
    beacon_data[12] = ((Internalgps.lat_i_g & 0xFF0000) >> 16);   //
    beacon_data[13] = ((Internalgps.lat_i_g & 0xFF00) >> 8);      //
    beacon_data[14] = (Internalgps.lat_i_g & 0xFF);               //
    beacon_data[15] = (((lastTenth / 10) & 0xFF000000) >> 24);    //
    beacon_data[16] = (((lastTenth / 10) & 0xFF0000) >> 16);      //
    beacon_data[17] = (((lastTenth / 10) & 0xFF00) >> 8);         //
    beacon_data[18] = ((lastTenth / 10) & 0xFF);                  //

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
        Serial.println("Advertising...");
        // pAdvertising->stop();
        vTaskDelay(pdMS_TO_TICKS(1000)); // advertising selama 3 detik
    }
}

static void retrieveGPSData(void *pvParam)
{
    double longitude, latitude;
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

                Serial.printf("Latitude : %f", latitude);
                Serial.printf("Longitude : %f", longitude);
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