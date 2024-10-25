#include "analog_input.h"

AnalogInput::AnalogInput() {
  Serial.println("Getting single-ended readings from AIN0..3");
  Serial.println(
      "ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");

  if (!_ads.begin(73)) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }
}

AnalogInput::AnalogInput(uint8_t address, adsGain_t gain) {
  Serial.println("Getting single-ended readings from AIN0..3");
  Serial.println(
      "ADC Range: +/- 6.144V (1 bit = 3mV/ADS1015, 0.1875mV/ADS1115)");

  if (!_ads.begin(address)) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }

  _ads.setGain(gain);
}

AnalogInput::~AnalogInput() {}

/** @brief
 * pengujian kalibrasi dilakukan dengan cara memberikan
 * tegangan tertentu, yaitu 0 dan 3V pada pin analog
 * input. kemudian hasil pembacaan ads dilihat dan
 * menunjukkan angka 56 dan 1061.
 * kita bisa menggunakan persamaan linear (y = m.x + b)
 * untuk menemukan persamaan linear-nya.
 * Persamaan -> V = (0.002985 * ADC) - 0.1672
 */
float AnalogInput::readAnalogInput(AnalogPin pin) {
  float voltage;
  int16_t adcReading = _ads.readADC_SingleEnded(static_cast<uint8_t>(pin));

  // persamaan linear. sesuaikan apabila perlu.
  voltage = (0.002985 * adcReading) - 0.1672;

  return voltage;
}

void AnalogInput::printRawAnalogInputValue() {
  auto adc0 = _ads.readADC_SingleEnded(0);
  auto adc1 = _ads.readADC_SingleEnded(1);
  auto adc2 = _ads.readADC_SingleEnded(2);
  auto adc3 = _ads.readADC_SingleEnded(3);

  auto volt0 = _ads.computeVolts(adc0);
  auto volt1 = _ads.computeVolts(adc1);
  auto volt2 = _ads.computeVolts(adc2);
  auto volt3 = _ads.computeVolts(adc3);

  auto readjustedVolt0 = readAnalogInput(AnalogPin::PIN_A0);
  auto readjustedVolt1 = readAnalogInput(AnalogPin::PIN_A1);
  auto readjustedVolt2 = readAnalogInput(AnalogPin::PIN_A2);
  auto readjustedVolt3 = readAnalogInput(AnalogPin::PIN_A3);

  Serial.println("-----------------------------------------------------------");
  Serial.printf("AIN0: %d\tVolt: %.2f\n", adc0, volt0);
  Serial.printf("NILAI AIN0 HASIL READJUST : %.2f\n", readjustedVolt0);
  Serial.printf("AIN1: %d\tVolt: %.2f\n", adc1, volt1);
  Serial.printf("NILAI AIN1 HASIL READJUST : %.2f\n", readjustedVolt1);
  Serial.printf("AIN2: %d\tVolt: %.2f\n", adc2, volt2);
  Serial.printf("NILAI AIN2 HASIL READJUST : %.2f\n", readjustedVolt2);
  Serial.printf("AIN3: %d\tVolt: %.2f\n", adc3, volt3);
  Serial.printf("NILAI AIN3 HASIL READJUST : %.2f\n", readjustedVolt3);
  Serial.println("-----------------------------------------------------------");

  vTaskDelay(pdMS_TO_TICKS(1000));
}
