#include <Arduino.h>
#include <driver/dac.h>
#include <driver/adc.h>

int initial_time_us = 0;

void setup() {
  Serial.begin(115200);

  dac_output_enable(DAC_CHANNEL_1); // Enable DAC channel 1

  adc1_config_width(ADC_WIDTH_BIT_12); // Set ADC width to 12 bits

  initial_time_us = micros(); // Get the initial time in microseconds
}

void loop() {
  float frequency = 1;
  float omega = 2 * PI * frequency;
  int dac_value = cos(omega * (micros() - initial_time_us) / 1000000.0) * 50 + 205; // Scale to 0-255

  dac_output_voltage(DAC_CHANNEL_1, dac_value); // Output to DAC channel 1

  delayMicroseconds(10000);

  int dac_adc_value = 4095 - adc1_get_raw(ADC1_CHANNEL_5);
  int resistor_adc_value = 4095 - adc1_get_raw(ADC1_CHANNEL_4);

  delayMicroseconds(10000);

  Serial.print(micros() - initial_time_us);
  Serial.print(", ");
  Serial.print(dac_value);
  Serial.print(", ");
  Serial.print(dac_adc_value);
  Serial.print(", ");
  Serial.println(resistor_adc_value);
}
