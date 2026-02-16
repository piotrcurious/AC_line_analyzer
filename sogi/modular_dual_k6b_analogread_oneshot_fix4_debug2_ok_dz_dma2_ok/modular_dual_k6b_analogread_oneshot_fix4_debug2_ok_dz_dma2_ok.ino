// bad setup of dma
/*
 * ESP32 SOGI-PLL - ADC Continuous Mode (PLL-Paced Polling)
 * DMA provides continuous sampling, PLL timing dictates when to read
 * ESP32 Arduino Core 3.3.1+
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

// Channel definitions
#define V_CHANNEL 0  // GPIO36 = ADC1_CH0
#define I_CHANNEL 3  // GPIO39 = ADC1_CH3

// ADC runs faster than PLL sample rate to ensure fresh data
#define ADC_OVERSAMPLE_RATE 200000  // 25kHz per channel (total 50kHz)
#define CONV_FRAME_SIZE 4 

SOGIVisualizer vis;
SOGI sogi_v(SOGI_K);
AdaptivePLL pll(NOMINAL_FREQ, PLL_KP, PLL_KI, 0.1001f);

struct {
    float prev_phase = 0.0f;
    float phase_offset = 0.0f;
    bool initialized = false;
} phase_track;

adc_continuous_handle_t adc_handle = NULL;

// Latest ADC values (continuously updated from DMA)
volatile float latest_v_mv = 0;
volatile float latest_i_mv = 0;
volatile bool v_updated = false;
volatile bool i_updated = false;

// Bresenham scheduling state (same as original)
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
        .conv_frame_size = CONV_FRAME_SIZE,
    };
    
    if (adc_continuous_new_handle(&adc_config, &adc_handle) != ESP_OK) {
        Serial.println("Failed to create ADC handle");
        return false;
    }
    
    adc_digi_pattern_config_t adc_pattern[2];
    
    // Voltage channel (GPIO36 = CH0)
    adc_pattern[0].atten = ADC_ATTEN_DB_12;
    adc_pattern[0].channel = V_CHANNEL;
    adc_pattern[0].unit = ADC_UNIT_1;
    adc_pattern[0].bit_width = ADC_BITWIDTH_12;
    
    // Current channel (GPIO39 = CH3)
    adc_pattern[1].atten = ADC_ATTEN_DB_12;
    adc_pattern[1].channel = I_CHANNEL;
    adc_pattern[1].unit = ADC_UNIT_1;
    adc_pattern[1].bit_width = ADC_BITWIDTH_12;
    
    adc_continuous_config_t dig_cfg = {
        .pattern_num = 2,
        .adc_pattern = adc_pattern,
        .sample_freq_hz = ADC_OVERSAMPLE_RATE,  // Fast continuous sampling
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
    };
    
    if (adc_continuous_config(adc_handle, &dig_cfg) != ESP_OK) {
        Serial.println("Failed to configure ADC");
        return false;
    }
    
    if (adc_continuous_start(adc_handle) != ESP_OK) {
        Serial.println("Failed to start ADC");
        return false;
    }
    
    Serial.printf("ADC Continuous mode OK (%d Hz total)\n", ADC_OVERSAMPLE_RATE);
    return true;
}

inline void updateLatestADCValues_double() {
    uint32_t ret_num = 0;
    esp_err_t ret;
    
    // FIX 2: Drain the entire DMA buffer to ensure we only leave with the freshest bytes
    do {
        ret = adc_continuous_read(adc_handle, adc_result, CONV_FRAME_SIZE, &ret_num, 0);
        
        if (ret == ESP_OK && ret_num > 0) {
            //poll_count++;
            
            // FIX 3: Iterate by exactly 2 bytes (the actual size of ESP32 Type1 data)
            // DO NOT use SOC_ADC_DIGI_DATA_BYTES_PER_CONV (4) here, or you will skip channels.
            for (uint32_t i = 0; i < ret_num; i += 2) { 
                adc_digi_output_data_t *p = (adc_digi_output_data_t*)&adc_result[i];
                
                uint32_t chan = p->type1.channel;
                uint16_t raw = p->type1.data;
                /*
                if (chan == V_CHANNEL) {
                    latest_adc.v_mv = adcRawToMillivolts(raw);
                    latest_adc.v_count++;
                    latest_adc.v_total++;
                } else if (chan == I_CHANNEL) {
                    latest_adc.i_mv = adcRawToMillivolts(raw);
                    latest_adc.i_count++;
                    latest_adc.i_total++;
                }
               */

               float mv = adcRawToMillivolts(raw);
            
            if (chan == V_CHANNEL) {
                latest_v_mv = mv;
                v_updated = true;
            } else if (chan == I_CHANNEL) {
                latest_i_mv = mv;
                i_updated = true;
            }
                
            }
        }
    } while (ret == ESP_OK); // Keep looping until the DMA queue returns ESP_ERR_TIMEOUT
}

// Background task to continuously update latest ADC values from DMA
void updateLatestADCValues() {
    uint32_t ret_num = 0;
    esp_err_t ret = adc_continuous_read(adc_handle, adc_result, CONV_FRAME_SIZE, &ret_num, 0);
    
    if (ret == ESP_OK && ret_num > 0) {
        // Process all available samples, keeping only the latest for each channel
       // for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_DATA_BYTES_PER_CONV) {
        for (int i = 0; i < ret_num; i += 2) {
            adc_digi_output_data_t *p = (adc_digi_output_data_t*)&adc_result[i];
            
            uint32_t chan = p->type1.channel;
            if (chan > 7) continue;
            
            uint16_t raw = p->type1.data;
            float mv = adcRawToMillivolts(raw);
            
            if (chan == V_CHANNEL) {
                latest_v_mv = mv;
                v_updated = true;
            } else if (chan == I_CHANNEL) {
                latest_i_mv = mv;
                i_updated = true;
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println("\nSOGI-PLL ADC Continuous (PLL-Paced)");
    
    updateTimingParameters(NOMINAL_FREQ);
    
    uint32_t start_c = get_cycle_count();
    last_sample_cycles = start_c;
    last_cycle_boundary = start_c;
    
    for (int i=0; i<4; i++) cycle_start_idx[i] = 0;
    current_cycle = 0;
    buf_idx = 0;
    
    vis.begin();
    
    if (!initADCContinuous()) {
        Serial.println("ADC init failed!");
        while(1) delay(100);
    }
    
    gpio_set_drive_capability((gpio_num_t)18, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability((gpio_num_t)23, GPIO_DRIVE_CAP_3);
    
    Serial.printf("Ready. CPU: %d MHz\n", ESP.getCpuFreqMHz());
    Serial.println("PLL controls timing, DMA provides continuous data");
}

void loop() {
    // Update latest ADC values from DMA buffer (non-blocking, continuous)
    updateLatestADCValues();
    
    // --- PLL-PACED SAMPLING (original timing logic) ---
    uint32_t now = get_cycle_count();
    
    // Check if we have reached or passed the scheduled time for the next sample
    if ((int32_t)(now - last_sample_cycles) >= 0) {
        
        // --- DEBUG ACCOUNTING ---
        uint32_t latency = now - last_sample_cycles;
        if (current_cycle < 3 && latency > max_jitter_ticks) {
            max_jitter_ticks = latency;
        }

        if (current_cycle < 3 && latency > (ticks_per_sample + (ticks_per_sample >> 1))) {
            overrun_count++;
            last_sample_cycles = now;
        }
        // -------------------------

        // Sample at PLL-determined time
        bool do_sample = (current_cycle < 3);

        if (do_sample) {
            // Read latest values from DMA buffer
            v_samp_buf[buf_idx] = latest_v_mv;
            i_samp_buf[buf_idx] = latest_i_mv;
            buf_idx = (buf_idx + 1) % BUF_N;
        }

        // Increment scheduled time by the CURRENT ticks_per_sample
        last_sample_cycles += ticks_per_sample;
        sample_slot_count++;

        // Bresenham: compute integer step for the NEXT scheduled slot
        uint32_t step = base_ticks_per_sample;
        bresenham_acc += ticks_remainder;
        if (bresenham_acc >= (uint32_t)SAMPLES_PER_CYCLE) {
            bresenham_acc -= (uint32_t)SAMPLES_PER_CYCLE;
            step += 1;
        }
        ticks_per_sample = step;

        // --- TIME-BASED cycle boundary detection ---
        uint32_t elapsed_cycle = now - last_cycle_boundary;
        if (elapsed_cycle >= single_cycle_cycles) {
            last_cycle_boundary += single_cycle_cycles;

            int prev_cycle = current_cycle;
            current_cycle = (current_cycle + 1) % 4;
            cycle_start_idx[current_cycle] = buf_idx;

            if (prev_cycle == 2) {
                // --- Deferred heavy processing ---
                uint32_t proc_start = get_cycle_count();
                
                sample_slot_count = 0;
                bresenham_acc = 0;
                
                int s_idx = cycle_start_idx[1];
                int e_idx = cycle_start_idx[2];
                int actual_count = (e_idx - s_idx + BUF_N) % BUF_N;

                if (actual_count > 0) {
                    // DC Estimation
                    float v_sum = 0, i_sum = 0;
                    for (int i = 0; i < actual_count; ++i) {
                        int idx = (s_idx + i) % BUF_N;
                        v_sum += v_samp_buf[idx];
                        i_sum += i_samp_buf[idx];
                    }
                    float v_win_dc = v_sum / (float)actual_count;
                    float i_win_dc = i_sum / (float)actual_count;
                    v_dc_offset = (0.2f * v_win_dc) + (0.8f * v_dc_offset);
                    i_dc_offset = (0.2f * i_win_dc) + (0.8f * i_dc_offset);

                    float ts = (float)ticks_per_sample * inv_cpu_freq;

                    // Process Window with Adaptive PLL
                    sogi_v.processWindow(v_samp_buf, BUF_N, s_idx, actual_count, pll.omega, ts, v_dc_offset);
                    pll.update(sogi_v.v_alpha, sogi_v.v_beta, ts);

                    // Phase Alignment for Visualization
                    SOGI phase_sogi(SOGI_K);
                    phase_sogi.processWindow(v_samp_buf, BUF_N, s_idx, actual_count, pll.omega, ts, v_dc_offset);

                    float phase = atan2f(phase_sogi.v_alpha, -phase_sogi.v_beta);
                    if (phase < 0) phase += 2.0f * PI;

                    if (phase_track.initialized) {
                        float phase_delta = phase - phase_track.prev_phase;
                        if (phase_delta < -PI) phase_track.phase_offset += 2.0f * PI;
                        else if (phase_delta > PI) phase_track.phase_offset -= 2.0f * PI;
                    } else {
                        phase_track.initialized = true;
                    }
                    phase_track.prev_phase = phase;
                    float unwrapped_phase = phase + phase_track.phase_offset;
                    float phase_for_alignment = fmodf(unwrapped_phase, 2.0f * PI);
                    if (phase_for_alignment < 0) phase_for_alignment += 2.0f * PI;

                    float samples_per_cycle_f = 1.0f / (pll.freq * ts);
                    float samples_back = (phase_for_alignment / (2.0f * PI)) * samples_per_cycle_f;
                    int aligned_start_idx = (s_idx + actual_count - (int)(samples_back + 0.5f) + BUF_N) % BUF_N;
                    int vis_count = (int)(samples_per_cycle_f + 0.5f);

                    updateTimingParameters(pll.freq);

                    if (vis_count > actual_count) vis_count = actual_count;
                    if (vis_count > 0) {
                        vis.update(v_samp_buf, i_samp_buf, BUF_N, aligned_start_idx, vis_count, pll.freq, pll.mag_smooth, v_dc_offset, i_dc_offset);
                    }

                    uint32_t proc_end = get_cycle_count();
                    float core_us = (float)(proc_end - proc_start) * inv_cpu_freq * 1e6f;
                    float max_jitter_us = (float)max_jitter_ticks * inv_cpu_freq * 1e6f;

                    Serial.printf("F:%.4fHz, Mag:%.3f, Vdc:%.1f, Idc:%.1f, Core:%.1fus, Jit:%.1fus, GainEst:%.4f\n",
                                  pll.freq, pll.mag_smooth, v_dc_offset, i_dc_offset, core_us, 
                                  max_jitter_us, pll.gain_est);
            
                    yield();
                    yield();
                    yield();
                    yield();
                    max_jitter_ticks = 0;
                }
            } 
        } 
    } 
}
