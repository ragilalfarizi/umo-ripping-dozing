#include "analog_input.h"
#include "rtc.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

RTC *rtc;
AnalogInput *ain;

void RTCDemo(void *pvParam);
TaskHandle_t RTCDemoHandler = NULL;

void analogDemo(void *pvParam);
TaskHandle_t analogDemoHandler = NULL;

SemaphoreHandle_t xSemaphore = NULL;

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
