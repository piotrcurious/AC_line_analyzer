#include "SOGIvisualizer.h"

// Configuration for a standard SSD1306 SPI OLED
class LGFX_SOGI : public lgfx::LGFX_Device {
    lgfx::Panel_SSD1306 _panel_instance;
    lgfx::Bus_SPI       _bus_instance;
public:
    LGFX_SOGI() {
        {
            auto cfg = _bus_instance.config();
            cfg.spi_host = VSPI_HOST; 
            cfg.spi_mode = 0;
            cfg.freq_write = 8000000;
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
            cfg.panel_width      = 128;
            cfg.panel_height     = 64;
            cfg.offset_x         = 0;
            cfg.offset_y         = 0;
            _panel_instance.config(cfg);
        }
        setPanel(&_panel_instance);
    }
};

// Use a static instance inside a getter to ensure initialization order
static LGFX_SOGI& get_hw() {
    static LGFX_SOGI dev;
    return dev;
}

SOGIVisualizer::SOGIVisualizer() {}

void SOGIVisualizer::begin() {
    auto& dev = get_hw();
    dev.init();
    dev.setRotation(0);
    
    // Initialize sprite for flicker-free updates
    if (_canvas.getBuffer() == nullptr) {
        _canvas.setColorDepth(1); 
        // Corrected: createSprite only takes (width, height)
        _canvas.createSprite(SCREEN_WIDTH, SCREEN_HEIGHT);
    }
}

void SOGIVisualizer::update(const float* buffer, int bufLen, int startIdx, int count, 
                            float freq, float magnitude, float error) {
    auto& dev = get_hw();
    
    // Safety check: ensure canvas buffer exists and hardware is initialized
    if (_canvas.getBuffer() == nullptr || count <= 0 || buffer == nullptr) {
        return;
    }
    _canvas.clear();
    
    // 1. Draw UI Borders and Labels
    _canvas.drawFastHLine(0, WAVE_HEIGHT, SCREEN_WIDTH, 1); 
    _canvas.setTextSize(1);
    _canvas.setTextColor(1);
    
    // 2. Plot Waveform
    int prev_x = -1;
    int prev_y = -1;
    
    // Scale factor: Peak-to-peak visualization
    // magnitude is smoothed amplitude, typically 0 to ~1.2V for full-scale ADC
    float scale_y = (magnitude > 0.05f) ? (8.0f / 0.15f) : 10.0f;
    int center_y = WAVE_HEIGHT / 2;
    
    for (int x = 0; x < SCREEN_WIDTH; x++) {
        int sample_offset = (x * count) / SCREEN_WIDTH;
        int idx = (startIdx + sample_offset) % bufLen;
        
        float val = buffer[idx];
        int y = center_y - (int)(val * scale_y);
        
        y = constrain(y, 0, WAVE_HEIGHT - 1);
        if (prev_x != -1) {
            _canvas.drawLine(prev_x, prev_y, x, y, 1);
        }
        prev_x = x;
        prev_y = y;
    }
    
    // 3. Draw Telemetry Text
    _canvas.setCursor(0, WAVE_HEIGHT + 4);
    _canvas.printf("F:%5.2fHz", freq);
    
    _canvas.setCursor(68, WAVE_HEIGHT + 4);
    _canvas.printf("M:%4.2fV", magnitude);
    
    // Error Bar
    int error_bar_w = constrain((int)(fabs(error) * 100.0f), 0, 120);
    _canvas.drawRect(0, SCREEN_HEIGHT - 8, 124, 6, 1);
    _canvas.fillRect(2, SCREEN_HEIGHT - 6, error_bar_w, 2, 1);
    
    // Push to physical display - Explicitly pass the device reference here
    _canvas.pushSprite(&dev, 0, 0);
}
