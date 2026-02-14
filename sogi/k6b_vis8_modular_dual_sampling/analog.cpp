// analog.cpp

#include "analog.h"

// Note: We removed the manual legacy calibration data to avoid conflicts with the modern driver.
// The modern ESP32 Arduino core handles calibration internally via analogReadMilliVolts().

int analogReadMillivolts(int pin, adc_atten_t attenuation, int oversampling) {
  // Use the Arduino-native way to set attenuation to ensure compatibility with driver_ng
  analogSetPinAttenuation(pin, attenuation);

  if (oversampling > 1) {
    uint32_t sum = 0;
    for (int i = 0; i < oversampling; i++) {
      // analogReadMilliVolts is the modern Arduino way to get calibrated readings
      sum += analogReadMilliVolts(pin);
    }
    return (int)(sum / oversampling);
  } else {
    return (int)analogReadMilliVolts(pin);
  }
}
