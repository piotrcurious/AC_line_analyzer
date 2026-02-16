#ifndef ANALOG_H
#define ANALOG_H

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Define the maximum number of channels for ADC1 (typically 8 on ESP32/S3)
#define ADC_UNIT_1_CH_MAX 8

/**
 * @brief Initializes the ADC1 unit. 
 * Call this once during system startup (e.g., in setup()).
 */
void analogSystemInit(void);

/**
 * @brief Configures a specific GPIO pin for ADC reading and sets up calibration.
 * * @param pin The GPIO number to configure.
 * @param attenuation The attenuation level (e.g., ADC_ATTEN_DB_12 for 0-3.3V).
 * @return true if configuration and calibration succeeded, false otherwise.
 */
bool analogPinInit(int pin, adc_atten_t attenuation);

/**
 * @brief Reads the voltage from a pin in millivolts.
 * * @param pin The GPIO number to read.
 * @param oversampling The number of samples to average (e.g., 1, 16, 64).
 * @return The voltage in millivolts, or -1 if the pin is invalid or uninitialized.
 */
int IRAM_ATTR analogReadMillivolts(int pin);

/**
 * @brief Helper to map a GPIO pin to an ADC1 channel.
 * This should be implemented in your .cpp based on your specific ESP32 variant.
 */
int get_adc1_channel(int pin);

#ifdef __cplusplus
}
#endif

#endif // ANALOG_H
