#ifndef SOGI_VISUALIZER_H
#define SOGI_VISUALIZER_H

#include <Arduino.h>

// Include LovyanGFX
#define LGFX_USE_V1
#include <LovyanGFX.hpp>

// --- Configuration for Tile-based Compositor ---
#define TILE_SIZE 4
// #define TILE_SHIFT 2

/**
 * @brief Simple Tile structure for dirty-rect rendering
 */
struct Tile {
    uint16_t x0, y0;
    uint16_t w, h;
    uint8_t *buf;
    bool dirty_curr;
    bool dirty_prev;

    Tile();
    void init(uint16_t _x0, uint16_t _y0, uint16_t _w, uint16_t _h);
    inline void prepareFrame() {
        if (dirty_prev || dirty_curr) {
            if (buf) memset(buf, 0, (size_t)w * (size_t)h);
        }
        dirty_prev = dirty_curr;
        dirty_curr = false;
    }
};

/**
 * @brief Manages a grid of Tiles to minimize SPI traffic
 */
struct TileManager {
    uint16_t screen_w, screen_h;
    uint16_t tile_size;
    uint16_t cols, rows;
    Tile *tiles;

    TileManager();
    void init(uint16_t sw, uint16_t sh, uint16_t tsize);

    inline Tile* tileAtIdx(uint16_t tx, uint16_t ty) {
        if ((uint32_t)tx >= cols || (uint32_t)ty >= rows) return nullptr;
        return &tiles[ty * cols + tx];
    }

    void writePixelGlobal(int16_t x, int16_t y, uint8_t color);
    void drawLine(int x0, int y0, int x1, int y1, uint8_t color);
    void startFrame();
    void flush(lgfx::LGFX_Device &dev);
};

/**
 * @brief Class to handle visualization of SOGI-PLL data
 */
class SOGIVisualizer {
public:
    static constexpr int SCREEN_WIDTH = 128;
    static constexpr int SCREEN_HEIGHT = 64;

    SOGIVisualizer();
    void begin();
    void update(const float* vBuffer, const float* iBuffer, int bufLen, int startIdx, int count,
                float freq, float magnitude, float v_dc, float i_dc);

private:
    // Any private members if needed
};

#endif // SOGI_VISUALIZER_H
