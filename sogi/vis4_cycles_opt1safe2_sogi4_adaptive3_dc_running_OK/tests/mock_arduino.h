#ifndef MOCK_ARDUINO_H
#define MOCK_ARDUINO_H

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <algorithm>

#define PI 3.14159265358979323846f

#ifndef constrain
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#endif

// Mock ESP class
class MockESP {
public:
    uint32_t getCpuFreqMHz() { return 240; }
};
extern MockESP ESP;

// Mock Serial class
class MockSerial {
public:
    template<typename... Args>
    void printf(const char* fmt, Args... args) {
        ::printf(fmt, args...);
    }
    void begin(int baud) {}
};
extern MockSerial Serial;

// Arduino functions to mock
extern int analogRead(int pin);
extern void analogReadResolution(int res);

#define ADC_PIN 36

#endif
