#include "SOGIvisualizer.h"
#include <math.h>

// Configuration for a standard SSD1306 SPI OLED using LovyanGFX
class LGFX_SOGI : public lgfx::LGFX_Device {
    lgfx::Panel_SSD1306 _panel_instance;
    lgfx::Bus_SPI        _bus_instance;
public:
    LGFX_SOGI() {
        {
            auto cfg = _bus_instance.config();
            cfg.spi_host = VSPI_HOST;
            cfg.spi_mode = 0;
            cfg.freq_write = 40000000;
            cfg.pin_sclk = 18;
            cfg.pin_mosi = 23;
            cfg.pin_miso = -1;
            cfg.pin_dc   = 2;
            cfg.dma_channel = 1;
            _bus_instance.config(cfg);
            _panel_instance.setBus(&_bus_instance);
        }
        {
            auto cfg = _panel_instance.config();
            cfg.pin_cs           = 5;
            cfg.pin_rst          = 4;
            cfg.panel_width      = SOGIVisualizer::SCREEN_WIDTH;
            cfg.panel_height     = SOGIVisualizer::SCREEN_HEIGHT;
            cfg.offset_x         = 0;
            cfg.offset_y         = 0;
            _panel_instance.config(cfg);
        }
        setPanel(&_panel_instance);
    }
};

static LGFX_SOGI& get_hw() {
    static LGFX_SOGI dev;
    return dev;
}

// ------------------ Tile Implementations ------------------

Tile::Tile() : x0(0), y0(0), w(0), h(0), buf(nullptr), dirty_curr(false), dirty_prev(false) {}

void Tile::init(uint16_t _x0, uint16_t _y0, uint16_t _w, uint16_t _h) {
    x0 = _x0; y0 = _y0; w = _w; h = _h;
    size_t n = (size_t)w * (size_t)h;
    if (buf) heap_caps_free(buf);
    buf = (uint8_t*) heap_caps_malloc(n, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (!buf) buf = (uint8_t*) malloc(n);
    dirty_curr = true;
    dirty_prev = true;
    if (buf) memset(buf, 0, n);
}

// ------------------ TileManager Implementations ------------------

TileManager::TileManager() : tiles(nullptr) {}

void TileManager::init(uint16_t sw, uint16_t sh, uint16_t tsize) {
    screen_w = sw; screen_h = sh; tile_size = tsize;
    cols = (screen_w + tile_size - 1) / tile_size;
    rows = (screen_h + tile_size - 1) / tile_size;
    tiles = (Tile*)malloc(sizeof(Tile) * cols * rows);
    for (uint16_t r = 0; r < rows; ++r) {
        for (uint16_t c = 0; c < cols; ++c) {
            uint16_t x0 = c * tile_size;
            uint16_t y0 = r * tile_size;
            uint16_t tw = (x0 + tile_size <= screen_w) ? tile_size : (screen_w - x0);
            uint16_t th = (y0 + tile_size <= screen_h) ? tile_size : (screen_h - y0);
            new (&tiles[r * cols + c]) Tile();
            tiles[r * cols + c].init(x0, y0, tw, th);
        }
    }
}

void TileManager::writePixelGlobal(int16_t x, int16_t y, uint8_t color) {
    if (x < 0 || y < 0 || x >= (int)screen_w || y >= (int)screen_h) return;
#ifdef TILE_SHIFT
    uint16_t tx = (uint16_t)(x >> TILE_SHIFT);
    uint16_t ty = (uint16_t)(y >> TILE_SHIFT);
    uint16_t lx = (uint16_t)(x & (tile_size - 1));
    uint16_t ly = (uint16_t)(y & (tile_size - 1));
#else
    uint16_t tx = (uint16_t)(x / tile_size);
    uint16_t ty = (uint16_t)(y / tile_size);
    uint16_t lx = (uint16_t)(x - tx * tile_size);
    uint16_t ly = (uint16_t)(y - ty * tile_size);
#endif
    Tile* t = tileAtIdx(tx, ty);
    if (!t || !t->buf) return;
    t->buf[(size_t)ly * (size_t)t->w + (size_t)lx] = color;
    t->dirty_curr = true;
}

void TileManager::drawLine(int x0, int y0, int x1, int y1, uint8_t color) {
    if ((x0 < 0 && x1 < 0) || (x0 >= (int)screen_w && x1 >= (int)screen_w) ||
        (y0 < 0 && y1 < 0) || (y0 >= (int)screen_h && y1 >= (int)screen_h)) return;

    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;

    while (true) {
        writePixelGlobal(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void TileManager::startFrame() {
    uint32_t count = (uint32_t)cols * rows;
    for (uint32_t i = 0; i < count; ++i) tiles[i].prepareFrame();
}

void TileManager::flush(lgfx::LGFX_Device &dev) {
    dev.endWrite();
    dev.startWrite();
    uint32_t total = (uint32_t)cols * rows;
    for (uint32_t i = 0; i < total; ++i) {
        Tile &t = tiles[i];
        if (t.dirty_curr || t.dirty_prev) {
            dev.pushImage(t.x0, t.y0, t.w, t.h, t.buf, lgfx::grayscale_8bit);
        }
    }
//    dev.endWrite();
}

static TileManager& get_canvas() {
    static TileManager tm;
    return tm;
}


// ------------------ Visualizer Implementation ------------------

static TileManager g_tiled_canvas;
static float last_v_min = -0.1f;
static float last_v_max = 0.1f;

static constexpr int TEXT_ROW_HEIGHT = 10;
static constexpr int ERROR_BAR_Y = SOGIVisualizer::SCREEN_HEIGHT - 1;
static constexpr int WAVE_AREA_HEIGHT = SOGIVisualizer::SCREEN_HEIGHT - TEXT_ROW_HEIGHT - 1;
static constexpr float MIN_RANGE = 0.05f;
static constexpr float PEAK_HISTORY_WEIGHT = 0.95f; // Slower adaptation for stability
static constexpr float PEAK_NEW_WEIGHT = 0.05f;

SOGIVisualizer::SOGIVisualizer() : _canvas(nullptr) {}

void SOGIVisualizer::begin() {
    auto& dev = get_hw();
    dev.init();
    dev.setRotation(0);
    //dev.setBrightness(255);
    dev.clear();

    gpio_set_drive_capability((gpio_num_t)18, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability((gpio_num_t)23, GPIO_DRIVE_CAP_3); 
    g_tiled_canvas.init(SCREEN_WIDTH, SCREEN_HEIGHT, TILE_SIZE);
}

void SOGIVisualizer::update(const float* buffer, int bufLen, int startIdx, int count, 
                            float freq, float magnitude, float error) {
    if (count <= 0 || buffer == nullptr) return;
    
    auto& dev = get_hw();
    g_tiled_canvas.startFrame();
    
    float range = last_v_max - last_v_min;
    if (range < MIN_RANGE) range = MIN_RANGE;
    
    const int wave_h = WAVE_AREA_HEIGHT;
    const int screen_w = SCREEN_WIDTH;
    const float scale_y = (WAVE_AREA_HEIGHT - 2) / range;
    const float mid_point = (last_v_max + last_v_min) * 0.5f;
    const int center_y = WAVE_AREA_HEIGHT / 2;


    // Draw zero line using fast float math (single multiply)
    int zero_line_y = center_y - (int)lrintf((0.0f - mid_point) * scale_y);
    if (zero_line_y >= 0 && zero_line_y < wave_h) {
        for (int x = 0; x < screen_w; x += 8) {
              g_tiled_canvas.writePixelGlobal(x, zero_line_y, 255);
            // bounds: x+1 always < screen_w for typical screen widths that are multiple of 4.
            //if (x + 1 < screen_w)   g_tiled_canvas.writePixelGlobal(x+1, zero_line_y, 255);
        }
    }


    // 1. Plot Waveform
    float current_min = 100.0f;
    float current_max = -100.0f;
    int prev_x = -1, prev_y = -1;
    
    for (int x = 0; x < SCREEN_WIDTH; x++) {
        int sample_idx = (startIdx + (x * count / SCREEN_WIDTH)) % bufLen;
        float val = buffer[sample_idx];
        
        if (val < current_min) current_min = val;
        if (val > current_max) current_max = val;
        
        // Calculate Y: (0,0) is top-left in LGFX
        int y = center_y - (int)((val - mid_point) * scale_y);
        if (y < 0) y = 0;
        if (y >= WAVE_AREA_HEIGHT) y = WAVE_AREA_HEIGHT - 1;
        
        if (prev_x != -1) {
            g_tiled_canvas.drawLine(prev_x, prev_y, x, y, 255); // Use 255 for "White" in grayscale
        }
        prev_x = x;
        prev_y = y;
    }
    
    // Smoothly update scale
    last_v_min = (current_min * PEAK_NEW_WEIGHT) + (last_v_min * PEAK_HISTORY_WEIGHT);
    last_v_max = (current_max * PEAK_NEW_WEIGHT) + (last_v_max * PEAK_HISTORY_WEIGHT);
    
    // 2. Error Bar
    //int error_w = (int)(fminf(1.0f, fabsf(error) * 2.0f) * SCREEN_WIDTH);
    //for(int x=0; x<error_w; x++) { g_tiled_canvas.writePixelGlobal(x, ERROR_BAR_Y, 255);} // very cpu intensive

    g_tiled_canvas.flush(dev);
}
