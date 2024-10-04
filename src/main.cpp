#include "analog_input.h"
#include "rtc.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "NimBLEDevice.h"
#include "NimBLEBeacon.h"
#include "NimBLEAdvertising.h"
#include "NimBLEEddystoneURL.h"

/* DEKLARASI OBJEK YANG DIGUNAKAN TERSIMPAN DI HEAP */
RTC *rtc;
AnalogInput *ain;

/* FORWARD DECLARATION UNTUK FUNGSI-FUNGSI DI DEPAN*/
static void RTCDemo(void *pvParam);
static void analogDemo(void *pvParam);
static void setCustomBeacon();

TaskHandle_t RTCDemoHandler = NULL;
TaskHandle_t analogDemoHandler = NULL;
SemaphoreHandle_t xSemaphore = NULL;

BLEAdvertising *pAdvertising;

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

    xSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(xSemaphore);

    /* BLE INIT */
    // Buat BLE Device
    BLEDevice::init("OMU BLE BEACON");
    BLEDevice::setPower(ESP_PWR_LVL_N12);
    pAdvertising = BLEDevice::getAdvertising();

    // Mengatur Beacon config
    setCustomBeacon();

    // Mulai advertising
    pAdvertising->start();
    Serial.println("Advertising dimulai...");

    Serial.println("setup done");

    xTaskCreatePinnedToCore(RTCDemo, "RTC Demo", 2048, NULL, 3, &RTCDemoHandler, 0);
    xTaskCreatePinnedToCore(analogDemo, "Analog Demo", 2048, NULL, 3, &analogDemoHandler, 0);
}

void loop()
{
    // Serial.println("Halo World!");
    // rtc->printRTCData();
    // delay(1000);
}

void RTCDemo(void *pvParam)
{
    while (1)
    {
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY))
        {

            rtc->printRTCData();

            xSemaphoreGive(xSemaphore);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

void analogDemo(void *pvParam)
{
    while (1)
    {
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY))
        {

            ain->printAnalogInputValue();

            xSemaphoreGive(xSemaphore);
            vTaskDelay(pdMS_TO_TICKS(1000));
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

    const uint16_t beaconUUID = 0xFEAA;

    // atur data advertising
    BLEAdvertisementData oAdvertisementData = BLEAdvertisementData();
    BLEAdvertisementData oScanResponseData = BLEAdvertisementData();

    oScanResponseData.setFlags(0x06); // GENERAL_DISC_MODE 0x02 | BR_EDR_NOT_SUPPORTED 0x04
    oScanResponseData.setCompleteServices(BLEUUID(beaconUUID));

    uint16_t voltage = 3300;         // dalam millivolts
    float current = 1.5;             // dalam ampere
    uint32_t timestamp = 1678801234; // contoh Unix TimeStamp

    // Convert current to a 16-bit fixed-point format (e.g., 1.5 A -> 384 in 8.8 format)
    // Konversi arus ke format (contoh: 1.5 A -> 384 di format 8.8)
    int16_t currentFixedPoint = (int16_t)(current * 256);

    char customData[9]; // 2 bytes untu voltage, 2 bytes untuk current, 4 bytes untuk timestamp

    // data dikemas
    customData[0] = (voltage >> 8);
    customData[1] = (voltage & 0xFF);
    customData[2] = (currentFixedPoint >> 8);
    customData[3] = (currentFixedPoint & 0xFF);
    customData[4] = (timestamp >> 24);
    customData[5] = (timestamp >> 16);
    customData[6] = (timestamp >> 8);
    customData[7] = (timestamp & 0xFF);

    oScanResponseData.setServiceData(BLEUUID(beaconUUID), std::string(customData, sizeof(customData)));
    oAdvertisementData.setName("OMU Demo Data");
    pAdvertising->setAdvertisementData(oAdvertisementData);
    pAdvertising->setScanResponseData(oScanResponseData);
}
