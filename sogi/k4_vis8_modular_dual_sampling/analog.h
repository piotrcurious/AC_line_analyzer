// analog.h

#ifndef ANALOG_H
#define ANALOG_H

#include <Arduino.h>
#include "esp_adc_cal.h" // For ADC calibration
#include "driver/adc.h"   // For ADC driver

// Define a structure to hold ADC calibration characteristics
typedef struct {
  esp_adc_cal_characteristics_t *adc_chars;
  adc_atten_t current_atten;
} adc_calibration_data_t;

// Global array to store calibration data for each ADC channel (ADC1)
#define ADC1_CHANNEL_COUNT 10 // Assuming all ADC1 channels are potentially used
extern adc_calibration_data_t adc1_cal_data[ADC1_CHANNEL_COUNT];
extern bool adc1_cal_initialized[ADC1_CHANNEL_COUNT];

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
 * @brief Replacement for analogReadMillivolts on ESP32 using IDF functions (for ESP-IDF 3.1.3).
 *
 * This function reads the analog value from the specified pin, using fuse calibration data
 * and supporting oversampling. It also allows setting the attenuation before reading.
 *
 * @param pin The Arduino pin number to read.
 * @param attenuation The ADC attenuation setting (ADC_ATTEN_DB_0, ADC_ATTEN_DB_2_5,
 * ADC_ATTEN_DB_6, ADC_ATTEN_DB_11).
 * @param oversampling The number of samples to read and average for oversampling.
 * A value of 1 disables oversampling. Higher values increase
 * resolution at the cost of reading time.
 * @return The analog reading in millivolts, or -1 if an error occurred.
 */
int analogReadMillivolts(int pin, adc_atten_t attenuation, int oversampling);

#endif // ANALOG_H
