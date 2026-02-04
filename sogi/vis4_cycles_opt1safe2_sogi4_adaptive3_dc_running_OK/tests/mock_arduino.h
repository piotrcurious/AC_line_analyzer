#ifndef MOCK_ARDUINO_H
#define MOCK_ARDUINO_H

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <algorithm>

#define PI 3.14159265358979323846f

inline float constrain(float x, float a, float b) {
    if(x < a) return a;
    if(x > b) return b;
    return x;
}

inline int constrain(int x, int a, int b) {
    if(x < a) return a;
    if(x > b) return b;
    return x;
}

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

#endif
