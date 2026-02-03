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
            cfg.freq_write = 20000000;  // 20MHz for fast transfers
            cfg.pin_sclk = 18; 
            cfg.pin_mosi = 23; 
            cfg.pin_miso = -1;
            cfg.pin_dc   = 2;  
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

// Static variables for peak tracking across frames
static float last_v_min = -0.1f;
static float last_v_max = 0.1f;

// Constants for layout
static constexpr int TEXT_ROW_HEIGHT = 10;
static constexpr int ERROR_BAR_Y = SOGIVisualizer::SCREEN_HEIGHT - 1;
static constexpr int WAVE_AREA_HEIGHT = SOGIVisualizer::SCREEN_HEIGHT - TEXT_ROW_HEIGHT - 1;
static constexpr int TEXT_Y = WAVE_AREA_HEIGHT;
static constexpr float MIN_RANGE = 0.05f;
static constexpr float PEAK_HISTORY_WEIGHT = 0.8f;
static constexpr float PEAK_NEW_WEIGHT = 0.2f;

SOGIVisualizer::SOGIVisualizer() : _canvas(&get_hw()) {}

void SOGIVisualizer::begin() {
    auto& dev = get_hw();
    dev.init();
    dev.setRotation(0);
    
    if (_canvas.getBuffer() == nullptr) {
        _canvas.setColorDepth(1); 
        _canvas.createSprite(SCREEN_WIDTH, SCREEN_HEIGHT);
    }
}

void SOGIVisualizer::update(const float* buffer, int bufLen, int startIdx, int count, 
                            float freq, float magnitude, float error) {
    // Early validation
    if (_canvas.getBuffer() == nullptr || count <= 0 || buffer == nullptr) {
        return;
    }
    
    auto& dev = get_hw();
    _canvas.clear();
    
    // 1. Calculate Scaling based on previous frame's peaks
    float range = last_v_max - last_v_min;
    if (range < MIN_RANGE) range = MIN_RANGE;
    
    const float scale_y = (WAVE_AREA_HEIGHT - 1) / range; // Margin of 1px top/bottom
    const int center_y = WAVE_AREA_HEIGHT >> 1; 
    const float mid_point = (last_v_max + last_v_min) * 0.5f;
    
    // Calculate the pixel position of 0.0V relative to the scaling
    // Formula: y = center_y - (value - mid_point) * scale
    int zero_line_y = center_y - (int)((0.0f - mid_point) * scale_y);

    // 2. Draw Zero Line (Dashed for clarity)
    if (zero_line_y >= 0 && zero_line_y < WAVE_AREA_HEIGHT) {
        for (int x = 0; x < SCREEN_WIDTH; x += 4) {
            _canvas.drawFastHLine(x, zero_line_y, 2, 1); // 2px on, 2px off
        }
    }

    float current_min = 100.0f;
    float current_max = -100.0f;
    
    // 3. Plot Waveform
    int prev_x = -1;
    int prev_y = -1;
    
    const float samples_per_pixel = (float)count / SCREEN_WIDTH;
    const int bufLen_mask = bufLen - 1; 
    const bool is_power_of_two = (bufLen > 0) && ((bufLen & bufLen_mask) == 0);
    
    for (int x = 0; x < SCREEN_WIDTH; x++) {
        const int sample_offset = (int)(x * samples_per_pixel);
        const int idx = is_power_of_two ? 
            ((startIdx + sample_offset) & bufLen_mask) :
            ((startIdx + sample_offset) % bufLen);
        
        const float val = buffer[idx];
        
        // Track peaks for NEXT frame
        current_min = (val < current_min) ? val : current_min;
        current_max = (val > current_max) ? val : current_max;
        
        // Calculate Y
        int y = center_y - (int)((val - mid_point) * scale_y);
        
        // Clamp to valid area
        if (y < 0) y = 0;
        else if (y >= WAVE_AREA_HEIGHT) y = WAVE_AREA_HEIGHT - 1;
        
        if (prev_x != -1) {
            _canvas.drawLine(prev_x, prev_y, x, y, 1);
        }
        prev_x = x;
        prev_y = y;
    }
    
    // Smooth peak tracking
    last_v_min = (current_min * PEAK_NEW_WEIGHT) + (last_v_min * PEAK_HISTORY_WEIGHT);
    last_v_max = (current_max * PEAK_NEW_WEIGHT) + (last_v_max * PEAK_HISTORY_WEIGHT);
    
    // 4. Telemetry
    _canvas.setTextSize(1);
    _canvas.setTextColor(1);
    
    _canvas.setCursor(0, TEXT_Y + 1);
    _canvas.printf("F:%5.2fHz", freq);
    
    _canvas.setCursor(68, TEXT_Y + 1);
    _canvas.printf("M:%4.2fV", magnitude);
    
    // 5. Error Bar
    const int error_bar_w = (int)(fabsf(error) * 150.0f);
    if (error_bar_w > 0) {
        int w = (error_bar_w > SCREEN_WIDTH) ? SCREEN_WIDTH : error_bar_w;
        _canvas.drawFastHLine(0, ERROR_BAR_Y, w, 1);
    }
    
    _canvas.pushSprite(0, 0);
}
