/*
 * ESP32 SOGI-PLL - ADC Continuous Mode with Timestamp Interpolation
 */

#include <Arduino.h>
#include <math.h>
#include "esp_adc/adc_continuous.h"
#include "SOGI.h"
#include "SOGIvisualizer.h"

#define ADC_PIN_V 36  // ADC1_CH0
#define ADC_PIN_I 39  // ADC1_CH3
#define NOMINAL_FREQ 50.0f
#define SOGI_K 0.7071f
#define PLL_KP 1.25f
#define PLL_KI 0.000000f
#define SAMPLES_PER_CYCLE 220
#define V_REF 3.3f

#define V_CHANNEL 0
#define I_CHANNEL 3
#define ADC_OVERSAMPLE_RATE 200000  // Total frequency (100kHz per channel)

SOGIVisualizer vis;
SOGI sogi_v(SOGI_K);
AdaptivePLL pll(NOMINAL_FREQ, PLL_KP, PLL_KI, 0.1001f);

struct {
    float prev_phase = 0.0f;
    float phase_offset = 0.0f;
    bool initialized = false;
} phase_track;

adc_continuous_handle_t adc_handle = NULL;

// --- NEW: History Buffer for Interpolation ---
#define HISTORY_SIZE 64
struct ADCSample {
    uint32_t timestamp;
    float v_mv;
    float i_mv;
};
volatile ADCSample adc_history[HISTORY_SIZE];
volatile int history_idx = 0;

uint64_t total_samples_read = 0;
uint32_t start_adc_cycles = 0;
float temp_v = 0, temp_i = 0;
bool got_v = false, got_i = false;
// ---------------------------------------------

uint32_t base_ticks_per_sample = 0;
uint32_t ticks_remainder = 0;
uint32_t bresenham_acc = 0;
uint32_t sample_slot_count = 0;

float v_dc_offset = 1650.0f;
float i_dc_offset = 1650.0f;
uint32_t last_sample_cycles = 0;
uint32_t last_cycle_boundary = 0;
uint32_t single_cycle_cycles = 0;
uint32_t ticks_per_sample = 0;
float inv_cpu_freq = 0;

int current_cycle = 0;
int cycle_start_idx[4] = {0,0,0,0};

const int BUF_N = 4 * SAMPLES_PER_CYCLE;
float v_samp_buf[BUF_N];
float i_samp_buf[BUF_N];
int buf_idx = 0;

uint8_t adc_result[512] = {0};
uint32_t max_jitter_ticks = 0;
uint32_t overrun_count = 0;

static inline uint32_t get_cycle_count() {
    uint32_t ccount;
    asm volatile("rsr %0, ccount" : "=a" (ccount));
    return ccount;
}

float adcRawToMillivolts(uint16_t raw_value) {
    return (float)raw_value * 3300.0f / 4095.0f;
}

void updateTimingParameters(float frequency) {
    float f_clamped = constrain(frequency, 40.0f, 60.0f);
    uint32_t cpu_hz = (uint32_t)ESP.getCpuFreqMHz() * 1000000U;
    inv_cpu_freq = 1.0f / (float)cpu_hz;

    single_cycle_cycles = (uint32_t) lrintf((float)cpu_hz / f_clamped);
    base_ticks_per_sample = single_cycle_cycles / (uint32_t)SAMPLES_PER_CYCLE;
    ticks_remainder = single_cycle_cycles % (uint32_t)SAMPLES_PER_CYCLE;
    bresenham_acc = 0;
    ticks_per_sample = base_ticks_per_sample;
}

bool initADCContinuous() {
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 4096,
        .conv_frame_size = sizeof(adc_result),
    };
    
    if (adc_continuous_new_handle(&adc_config, &adc_handle) != ESP_OK) return false;
    
    adc_digi_pattern_config_t adc_pattern[2];
    adc_pattern[0].atten = ADC_ATTEN_DB_12;
    adc_pattern[0].channel = V_CHANNEL;
    adc_pattern[0].unit = ADC_UNIT_1;
    adc_pattern[0].bit_width = ADC_BITWIDTH_12;
    
    adc_pattern[1].atten = ADC_ATTEN_DB_12;
    adc_pattern[1].channel = I_CHANNEL;
    adc_pattern[1].unit = ADC_UNIT_1;
    adc_pattern[1].bit_width = ADC_BITWIDTH_12;
    
    adc_continuous_config_t dig_cfg = {
        .pattern_num = 2,
        .adc_pattern = adc_pattern,
        .sample_freq_hz = ADC_OVERSAMPLE_RATE, 
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };
    
    if (adc_continuous_config(adc_handle, &dig_cfg) != ESP_OK) return false;
    if (adc_continuous_start(adc_handle) != ESP_OK) return false;
    
    // Anchor time zero exactly as the hardware starts converting
    start_adc_cycles = get_cycle_count();
    
    return true;
}

// Read DMA frames, calculate hardware-accurate timestamps, push to ring buffer
void updateLatestADCValues() {
    uint32_t ret_num = 0;
    uint32_t cpu_hz = ESP.getCpuFreqMHz() * 1000000U;
    
    // Keep draining the buffer
    while (adc_continuous_read(adc_handle, adc_result, sizeof(adc_result), &ret_num, 0) == ESP_OK && ret_num > 0) {
        for (int i = 0; i < ret_num; i += 2) {
            adc_digi_output_data_t *p = (adc_digi_output_data_t*)&adc_result[i];
            uint32_t chan = p->type1.channel;
            if (chan > 7) continue;
            
            float mv = adcRawToMillivolts(p->type1.data);
            
            if (chan == V_CHANNEL) { temp_v = mv; got_v = true; } 
            else if (chan == I_CHANNEL) { temp_i = mv; got_i = true; }
            
            // Wait for both interleaved channels to form a unified sample
            if (got_v && got_i) {
                // Calculate rigid hardware timestamp
                uint32_t ts = start_adc_cycles + (uint32_t)((total_samples_read * (uint64_t)cpu_hz) / (ADC_OVERSAMPLE_RATE / 2));
                
                adc_history[history_idx] = {ts, temp_v, temp_i};
                history_idx = (history_idx + 1) % HISTORY_SIZE;
                total_samples_read++;
                
                got_v = false;
                got_i = false;
            }
        }
    }
}

// Find bounding points in the history buffer and linearly interpolate
bool getInterpolatedSample(uint32_t target_ts, float &out_v, float &out_i) {
    int idx = (history_idx - 1 + HISTORY_SIZE) % HISTORY_SIZE; // Newest sample
    
    for (int i = 0; i < HISTORY_SIZE - 1; i++) {
        int prev_idx = (idx - 1 + HISTORY_SIZE) % HISTORY_SIZE;
        
        uint32_t t2 = adc_history[idx].timestamp;
        uint32_t t1 = adc_history[prev_idx].timestamp;
        
        int32_t diff2 = (int32_t)(t2 - target_ts);
        int32_t diff1 = (int32_t)(target_ts - t1);
        
        // Is our target sandwiched between t1 and t2?
        if (diff2 >= 0 && diff1 >= 0) {
            float dt = (float)(int32_t)(t2 - t1);
            if (dt <= 0.0f) { // Edge-case fallback
                out_v = adc_history[idx].v_mv;
                out_i = adc_history[idx].i_mv;
                return true;
            }
            
            float fraction = (float)diff1 / dt;
            out_v = adc_history[prev_idx].v_mv + fraction * (adc_history[idx].v_mv - adc_history[prev_idx].v_mv);
            out_i = adc_history[prev_idx].i_mv + fraction * (adc_history[idx].i_mv - adc_history[prev_idx].i_mv);
            return true;
        }
        idx = prev_idx;
    }
    return false; // Target is out of bounds (too old)
}

void setup() {
    Serial.begin(115200);
    delay(100);
    
    updateTimingParameters(NOMINAL_FREQ);
    
    if (!initADCContinuous()) {
        Serial.println("ADC init failed!");
        while(1) delay(100);
    }
    
    // Base schedule entirely off the hardware anchor
    last_sample_cycles = start_adc_cycles;
    last_cycle_boundary = start_adc_cycles;
    
    for (int i=0; i<4; i++) cycle_start_idx[i] = 0;
    vis.begin();
    Serial.println("PLL paced via hardware-interpolated DMA buffers.");
}

void loop() {
    updateLatestADCValues();
    
    uint32_t target_cycles = last_sample_cycles;
    
    // Check our newest DMA sample timestamp
    int newest_idx = (history_idx - 1 + HISTORY_SIZE) % HISTORY_SIZE;
    uint32_t newest_ts = adc_history[newest_idx].timestamp;
    
    // GATE: We ONLY process a sample if the DMA has advanced past our PLL schedule.
    // This perfectly eliminates jitter, allowing the mathematical schedule to dictate pacing.
    if ((int32_t)(newest_ts - target_cycles) >= 0) {
        
        float interp_v = adc_history[newest_idx].v_mv; // fallback defaults
        float interp_i = adc_history[newest_idx].i_mv;
        getInterpolatedSample(target_cycles, interp_v, interp_i);

        if (current_cycle < 3) {
            v_samp_buf[buf_idx] = interp_v;
            i_samp_buf[buf_idx] = interp_i;
            buf_idx = (buf_idx + 1) % BUF_N;
        }

        last_sample_cycles += ticks_per_sample;
        sample_slot_count++;

        uint32_t step = base_ticks_per_sample;
        bresenham_acc += ticks_remainder;
        if (bresenham_acc >= (uint32_t)SAMPLES_PER_CYCLE) {
            bresenham_acc -= (uint32_t)SAMPLES_PER_CYCLE;
            step += 1;
        }
        ticks_per_sample = step;

        // Use the mathematically exact target_cycles here instead of `now` jitter!
        uint32_t elapsed_cycle = target_cycles - last_cycle_boundary;
        if (elapsed_cycle >= single_cycle_cycles) {
            last_cycle_boundary += single_cycle_cycles;

            int prev_cycle = current_cycle;
            current_cycle = (current_cycle + 1) % 4;
            cycle_start_idx[current_cycle] = buf_idx;

            if (prev_cycle == 2) {
                // Deferred heavy processing block...
                // (Paste your EXACT original heavy processing code here down to max_jitter_ticks = 0)
                // Nothing mathematically changes below this point!
            }
        }
    }
}
