// analog.cpp

#include "analog.h"

#define ADC1_CHANNEL_COUNT 10

// Keep calibration data in a static array to persist across calls
static esp_adc_cal_characteristics_t adc_chars[ADC1_CHANNEL_COUNT];
static bool calibrated[ADC1_CHANNEL_COUNT] = {false};

/**
 * Perform one-time setup for the ADC pin.
 * Call this once in your setup() or when attenuation needs to change.
 */
bool analogInit(int pin, adc_atten_t attenuation) {
    int adc1_chan = get_adc1_channel(pin);
    if (adc1_chan == -1) return false;

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten((adc1_channel_t)adc1_chan, attenuation);

    // Characterize ADC
    // Note: Assuming 1100mV as default Vref for characterization if not specified.
    // User provided 1100 in the snippet.
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, attenuation, ADC_WIDTH_BIT_12, 1100, &adc_chars[adc1_chan]);
    calibrated[adc1_chan] = true;
    return true;
}

/**
 * Ultra-fast read. Assumes analogInit was already called.
 */
int analogReadMillivolts(int pin, int oversampling) {
    int adc1_chan = get_adc1_channel(pin);
    if (adc1_chan == -1 || !calibrated[adc1_chan]) return -1;

    uint32_t sum = 0;

    // Fast oversampling loop
    for (int i = 0; i < oversampling; i++) {
        sum += adc1_get_raw((adc1_channel_t)adc1_chan);
    }

    uint32_t raw_avg = sum / (oversampling > 0 ? oversampling : 1);

    // Convert to mV using pre-calculated characteristics
    return esp_adc_cal_raw_to_voltage(raw_avg, &adc_chars[adc1_chan]);
}
