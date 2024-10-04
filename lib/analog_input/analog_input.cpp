#include "analog_input.h"

AnalogInput::AnalogInput()
{
    Serial.println("Getting single-ended readings from AIN0..3");
    Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");

    if (!_ads.begin(73))
    {
        Serial.println("Failed to initialize ADS.");
        while (1)
            ;
    }
}

AnalogInput::~AnalogInput() {}

/** @brief
 * pengujian kalibrasi dilakukan dengan cara memberikan
 * tegangan tertentu, yaitu 0 dan 3V ke pada pin analog
 * input. kemudian hasil pembacaan ads dilihat dan
 * menunjukkan angka 56 dan 1061.
 * kita bisa menggunakan persamaan linear (y = m.x + b)
 * untuk menemukan persamaan linear-nya.
 * Persamaan -> V = (0.002985 * ADC) - 0.1672
 */
float AnalogInput::readjustAnalogIn()
{
    float voltage;
    int16_t adcReading = _ads.readADC_SingleEnded(1);

    // math equation
    voltage = (0.002985 * adcReading) - 0.1672;

    return voltage;
}

void AnalogInput::printAnalogInputValue()
{
    auto adc = _ads.readADC_SingleEnded(1);
    auto volt = _ads.computeVolts(adc);
    auto readjustedVolt = readjustAnalogIn();

    Serial.println("-----------------------------------------------------------");
    Serial.printf("AIN2: %d\tVolt: %.2f\n", adc, volt);
    Serial.printf("NILAI AIN4 HASIL READJUST : %.2f\n", readjustAnalogIn());
    Serial.println("-----------------------------------------------------------");

    vTaskDelay(pdMS_TO_TICKS(1000));
}
