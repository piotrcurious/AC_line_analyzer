#ifndef SOGI_VISUALIZER_H
#define SOGI_VISUALIZER_H

#include <Arduino.h>

// Include LovyanGFX
#define LGFX_USE_V1
#include <LovyanGFX.hpp>

/**
 * @brief Class to handle visualization of SOGI-PLL data on 128x64 OLED
 */
class SOGIVisualizer {
public:
    static constexpr int SCREEN_WIDTH = 128;
    static constexpr int SCREEN_HEIGHT = 64;

    SOGIVisualizer();

    /** @brief Initialize the display hardware */
    void begin();

    /**
     * @brief Render the waveform and telemetry
     * @param buffer Pointer to the circular sample buffer
     * @param bufLen Total length of the circular buffer
     * @param startIdx Start index of the cycle to visualize
     * @param count Number of samples in that cycle
     * @param freq Current calculated frequency
     * @param magnitude Current signal magnitude
     * @param error Current PLL phase error
     */
    void update(const float* buffer, int bufLen, int startIdx, int count, 
                float freq, float magnitude, float error);

private:
    LGFX_Sprite _canvas; // Use a sprite for flicker-free double buffering
};

#endif // SOGI_VISUALIZER_H
