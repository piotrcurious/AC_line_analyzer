// analog.h

#ifndef ANALOG_H
#define ANALOG_H

#include <Arduino.h>
#include "esp_adc_cal.h" // For ADC calibration
#include "driver/adc.h"   // For ADC driver

// Helper function to get the ADC channel number from the Arduino pin number for ADC1
static inline int get_adc1_channel(int pin) {
  if (pin == 36) return 0; // GPIO36, ADC1_CHANNEL_0
  if (pin == 37) return 1; // GPIO37, ADC1_CHANNEL_1
  if (pin == 38) return 2; // GPIO38, ADC1_CHANNEL_2
  if (pin == 39) return 3; // GPIO39, ADC1_CHANNEL_3
  if (pin == 32) return 4; // GPIO32, ADC1_CHANNEL_4
  if (pin == 33) return 5; // GPIO33, ADC1_CHANNEL_5
  if (pin == 34) return 6; // GPIO34, ADC1_CHANNEL_6
  if (pin == 35) return 7; // GPIO35, ADC1_CHANNEL_7
  if (pin == 25) return 8; // GPIO25, ADC1_CHANNEL_8
  if (pin == 26) return 9; // GPIO26, ADC1_CHANNEL_9
  return -1; // Invalid pin for ADC1
}

/**
 * Perform one-time setup for the ADC pin.
 * Call this once in your setup() or when attenuation needs to change.
 */
bool analogInit(int pin, adc_atten_t attenuation);

/**
 * Ultra-fast read. Assumes analogInit was already called.
 */
int analogReadMillivolts(int pin, int oversampling);

#endif // ANALOG_H
