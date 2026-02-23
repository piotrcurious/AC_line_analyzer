#include "analog.h"
#include "esp_log.h"

// Initialize static handles
static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_cali_handle_t adc1_cali_handles[ADC_UNIT_1_CH_MAX] = {NULL};
static bool adc1_chan_configured[ADC_UNIT_1_CH_MAX] = {false};

// Helper function to map GPIO to ADC1 Channel (Standard ESP32)
int IRAM_ATTR get_adc1_channel(int pin) {
    switch (pin) {
        case 36: return ADC_CHANNEL_0;
        case 37: return ADC_CHANNEL_1;
        case 38: return ADC_CHANNEL_2;
        case 39: return ADC_CHANNEL_3;
        case 32: return ADC_CHANNEL_4;
        case 33: return ADC_CHANNEL_5;
        case 34: return ADC_CHANNEL_6;
        case 35: return ADC_CHANNEL_7;
        default: return -1; // Not an ADC1 pin
    }
}

void analogSystemInit() {
    if (adc1_handle != NULL) return; // Already initialized
    
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));
}

bool analogPinInit(int pin, adc_atten_t attenuation) {
    int channel = get_adc1_channel(pin);
    if (channel == -1 || adc1_handle == NULL) return false;

    adc_oneshot_chan_cfg_t config = {
        .atten = attenuation,
        .bitwidth = ADC_BITWIDTH_DEFAULT, // Typically 12-bit
    };
    
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, (adc_channel_t)channel, &config));

    // Create calibration handle if it doesn't exist
    if (adc1_cali_handles[channel] == NULL) {
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = ADC_UNIT_1,
            .atten = attenuation,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        // This scheme is compatible with most ESP32 variants
        adc_cali_create_scheme_line_fitting(&cali_config, &adc1_cali_handles[channel]);
        
    }

    adc1_chan_configured[channel] = true;
    return true;
}

int IRAM_ATTR analogReadMillivolts(int pin) {
    int channel = get_adc1_channel(pin);
    if (channel == -1 || !adc1_chan_configured[channel]) return -1;

    int raw_val = 0;

        adc_oneshot_read(adc1_handle, (adc_channel_t)channel, &raw_val);

    int voltage_mv = 0;

    if (adc1_cali_handles[channel]) {
        adc_cali_raw_to_voltage(adc1_cali_handles[channel], raw_val, &voltage_mv);
    } else {
        // Fallback if calibration fails
        voltage_mv = (raw_val * 3300) / 4095;
    }

    return voltage_mv;

    //return raw_val;
}
