/*
 * ESP32 SOGI-PLL - ADC Continuous Mode with Frame Timestamping
 * ISR stores entire DMA frames with timestamps, interpolation uses frame timing
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
#define SAMPLES_PER_CYCLE 1024 // 100 is 10khz, 200 is 20khz 400 is 40khz 800 is 80khz memory is the limit
  // for udp frames 128 (2 bytes per data point, voltage and current) is quite max it can fit
#define V_REF 3.3f

// Channel definitions
#define V_CHANNEL 0  // GPIO36 = ADC1_CH0
#define I_CHANNEL 3  // GPIO39 = ADC1_CH3

// ADC configuration
#define ADC_OVERSAMPLE_RATE 200000  // 100kHz per channel (total 200kHz)
#define CONV_FRAME_SIZE 4  // Can be adjusted 32-512

// Maximum samples per frame (CONV_FRAME_SIZE / 2 bytes per sample / 2 channels)
#define MAX_SAMPLES_PER_CHANNEL (CONV_FRAME_SIZE / 4)

// Frame buffer - stores complete DMA frames with timestamps
// Increase if frames are being dropped at small CONV_FRAME_SIZE
#define FRAME_BUFFER_SIZE 5000  // Number of frames to buffer

#define MAX_SEARCH 8

SOGIVisualizer vis;
SOGI sogi_v(SOGI_K);
AdaptivePLL pll(NOMINAL_FREQ, PLL_KP, PLL_KI, 0.1001f);

struct {
    float prev_phase = 0.0f;
    float phase_offset = 0.0f;
    bool initialized = false;
} phase_track;

// Timestamped frame structure - store RAW data, parse only when needed
struct TimestampedFrame {
    uint32_t end_timestamp;        // CPU cycles at END of frame capture
    uint8_t raw_data[CONV_FRAME_SIZE];  // Raw frame data
    uint16_t data_size;            // Actual bytes in this frame
    uint8_t sample_count;          // Cached: samples per channel (set on first parse)
};

// Circular buffer for timestamped frames
volatile TimestampedFrame frame_buffer[FRAME_BUFFER_SIZE];
volatile uint32_t frame_write_idx = 0;
volatile uint32_t frame_read_idx = 0;
volatile uint32_t isr_callback_count = 0;
volatile uint32_t frames_dropped = 0;

adc_continuous_handle_t adc_handle = NULL;

// Timing parameters
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
uint32_t cpu_freq_hz = 0;

// Deterministic sample timing (calculated from ADC_OVERSAMPLE_RATE)
uint32_t cycles_per_adc_sample = 0;  // CPU cycles between consecutive ADC samples

int current_cycle = 0;
int cycle_start_idx[4] = {0,0,0,0};

const int BUF_N = 4 * SAMPLES_PER_CYCLE;
float v_samp_buf[BUF_N];
float i_samp_buf[BUF_N];
int buf_idx = 0;

uint32_t max_jitter_ticks = 0;
uint32_t overrun_count = 0;
uint32_t interpolation_count = 0;
uint32_t nearest_sample_count = 0;
uint32_t max_frame_age_us = 0;  // Track maximum frame age in microseconds

static inline uint32_t IRAM_ATTR get_cycle_count() {
    uint32_t ccount;
    asm volatile("rsr %0, ccount" : "=a" (ccount));
    return ccount;
}

float adcRawToMillivolts(uint16_t raw_value) {
    return (float)raw_value * 3300.0f / 4095.0f;
}

// ISR callback - ultra-fast: just copy raw frame with timestamp
static bool IRAM_ATTR adc_conv_done_callback(adc_continuous_handle_t handle, 
                                              const adc_continuous_evt_data_t *edata, 
                                              void *user_data) {
    // Get timestamp at END of frame
    uint32_t end_timestamp = get_cycle_count();
    
    // Calculate next write position
    uint32_t next_write = (frame_write_idx + 1) % FRAME_BUFFER_SIZE;
    
    // Check for buffer overflow
    if (next_write == frame_read_idx) {
        frames_dropped++;
        return false;
    }
    
    // Fast copy: store raw frame data without parsing
    uint32_t size = edata->size;
    if (size > CONV_FRAME_SIZE) size = CONV_FRAME_SIZE;
    
    // Use memcpy for fastest transfer
    memcpy((void*)frame_buffer[frame_write_idx].raw_data, 
           edata->conv_frame_buffer, 
           size);
    
    frame_buffer[frame_write_idx].end_timestamp = end_timestamp;
    frame_buffer[frame_write_idx].data_size = size;
    frame_buffer[frame_write_idx].sample_count = 0;  // Will be computed on first use
    
    // Advance write pointer
    frame_write_idx = next_write;
    isr_callback_count++;
    
    return false;
}

// Parse a specific sample from raw frame data
// Returns true if sample found, fills v_raw and i_raw
bool parseFrameSample(const TimestampedFrame* frame, uint8_t sample_idx, uint16_t &v_raw, uint16_t &i_raw) {
    uint8_t v_count = 0, i_count = 0;
    bool v_found = false, i_found = false;
    
    // Parse frame to find the sample_idx-th occurrence of each channel
    for (uint16_t i = 0; i < frame->data_size; i += 2) {
        const adc_digi_output_data_t *p = (const adc_digi_output_data_t*)&frame->raw_data[i];
        uint32_t chan = p->type1.channel;
        
        if (chan == V_CHANNEL) {
            if (v_count == sample_idx) {
                v_raw = p->type1.data;
                v_found = true;
                if (i_found) return true;
            }
            v_count++;
        } else if (chan == I_CHANNEL) {
            if (i_count == sample_idx) {
                i_raw = p->type1.data;
                i_found = true;
                if (v_found) return true;
            }
            i_count++;
        }
    }
    
    return v_found && i_found;
}

// Count samples in a frame (cached after first call)
uint8_t countFrameSamples(TimestampedFrame* frame) {
    // Return cached value if available
    if (frame->sample_count > 0) {
        return frame->sample_count;
    }
    
    // Count and cache
    uint8_t v_count = 0, i_count = 0;
    
    for (uint16_t i = 0; i < frame->data_size; i += 2) {
        const adc_digi_output_data_t *p = (const adc_digi_output_data_t*)&frame->raw_data[i];
        uint32_t chan = p->type1.channel;
        
        if (chan == V_CHANNEL) v_count++;
        else if (chan == I_CHANNEL) i_count++;
    }
    
    frame->sample_count = (v_count < i_count) ? v_count : i_count;
    return frame->sample_count;
}

// Get timestamp of a specific sample within a frame
uint32_t getFrameSampleTimestamp(uint32_t frame_end_time, uint8_t sample_idx, uint8_t sample_count) {
    uint32_t samples_from_end = sample_count - 1 - sample_idx;
    uint32_t cycles_back = samples_from_end * cycles_per_adc_sample;
    return frame_end_time - cycles_back;
}

// Optimized interpolation - search backwards from newest frames
bool interpolateSampleAtTime(uint32_t target_time, float &v_out, float &i_out) {
    uint32_t read_idx = frame_read_idx;
    uint32_t write_idx = frame_write_idx;
    
    if (read_idx == write_idx) {
        return false;
    }
    
    // Search BACKWARDS from newest frame (most likely to find recent data)
    // Start from most recent frame and work backwards
    uint32_t frame_idx = (write_idx - 1 + FRAME_BUFFER_SIZE) % FRAME_BUFFER_SIZE;
    uint32_t search_count = 0;
    uint32_t max_search = (write_idx >= read_idx) ? 
                          (write_idx - read_idx) : 
                          (FRAME_BUFFER_SIZE - read_idx + write_idx);
    
    // Limit search depth for performance - only check recent frames
    if (max_search > MAX_SEARCH) max_search = MAX_SEARCH;  // Only search last 20 frames
    
    while (search_count < max_search) {
        TimestampedFrame* frame = (TimestampedFrame*)&frame_buffer[frame_idx];
        
        if (frame->data_size > 0) {
            uint8_t sample_count = countFrameSamples(frame);
            
            if (sample_count > 0) {
                uint32_t frame_end = frame->end_timestamp;
                uint32_t frame_start = getFrameSampleTimestamp(frame_end, 0, sample_count);
                
                // Early termination: if frame is too old (>100ms), stop searching
                int32_t frame_age = (int32_t)(target_time - frame_end);
                if (frame_age > (int32_t)(cpu_freq_hz / 10)) {  // 100ms
                    break;  // Frame too old, won't find target
                }
                
                int32_t delta_start = (int32_t)(target_time - frame_start);
                int32_t delta_end = (int32_t)(target_time - frame_end);
                
                // Target is within this frame - FOUND IT!
                if (delta_start >= 0 && delta_end <= 0) {
                    // Track frame age for diagnostics
                    uint32_t now = get_cycle_count();
                    uint32_t frame_age_cycles = now - frame_end;
                    uint32_t frame_age_us = (uint32_t)((float)frame_age_cycles * inv_cpu_freq * 1e6f);
                    if (frame_age_us > max_frame_age_us) {
                        max_frame_age_us = frame_age_us;
                    }
                    
                    // Calculate which sample index
                    uint32_t time_into_frame = target_time - frame_start;
                    float sample_idx_float = (float)time_into_frame / (float)cycles_per_adc_sample;
                    uint8_t idx_before = (uint8_t)sample_idx_float;
                    
                    if (idx_before >= sample_count - 1) {
                        // Use last sample
                        idx_before = sample_count - 1;
                        uint16_t v_raw, i_raw;
                        if (parseFrameSample(frame, idx_before, v_raw, i_raw)) {
                            v_out = adcRawToMillivolts(v_raw);
                            i_out = adcRawToMillivolts(i_raw);
                            interpolation_count++;
                            return true;
                        }
                    } else {
                        // Interpolate between samples
                        uint8_t idx_after = idx_before + 1;
                        float alpha = sample_idx_float - (float)idx_before;
                        
                        uint16_t v_raw_before, i_raw_before, v_raw_after, i_raw_after;
                        
                        if (parseFrameSample(frame, idx_before, v_raw_before, i_raw_before) &&
                            parseFrameSample(frame, idx_after, v_raw_after, i_raw_after)) {
                            
                            float v_before = adcRawToMillivolts(v_raw_before);
                            float v_after = adcRawToMillivolts(v_raw_after);
                            float i_before = adcRawToMillivolts(i_raw_before);
                            float i_after = adcRawToMillivolts(i_raw_after);
                            
                            v_out = v_before + alpha * (v_after - v_before);
                            i_out = i_before + alpha * (i_after - i_before);
                            
                            interpolation_count++;
                            return true;
                        }
                    }
                }
                
                // Target is after this frame (more recent) - target must be in a newer frame
                // Continue searching backwards through older frames
                if (delta_end > 0) {
                    // Keep going backwards
                } else {
                    // Target is before this frame - use this frame's first sample as nearest
                    uint16_t v_raw, i_raw;
                    if (parseFrameSample(frame, 0, v_raw, i_raw)) {
                        v_out = adcRawToMillivolts(v_raw);
                        i_out = adcRawToMillivolts(i_raw);
                        nearest_sample_count++;
                        return true;
                    }
                }
            }
        }
        
        // Move backwards to older frame
        frame_idx = (frame_idx - 1 + FRAME_BUFFER_SIZE) % FRAME_BUFFER_SIZE;
        search_count++;
        
        if (frame_idx == read_idx) {
            break;  // Reached oldest frame
        }
    }
    
    // Fallback: use most recent frame's last sample
    uint32_t last_idx = (write_idx - 1 + FRAME_BUFFER_SIZE) % FRAME_BUFFER_SIZE;
    TimestampedFrame* last_frame = (TimestampedFrame*)&frame_buffer[last_idx];
    
    if (last_frame->data_size > 0) {
        uint8_t last_count = countFrameSamples(last_frame);
        if (last_count > 0) {
            uint16_t v_raw, i_raw;
            if (parseFrameSample(last_frame, last_count - 1, v_raw, i_raw)) {
                v_out = adcRawToMillivolts(v_raw);
                i_out = adcRawToMillivolts(i_raw);
                nearest_sample_count++;
                return true;
            }
        }
    }
    
    return false;
}

// Clean up old frames - keep only recent data (last 50ms)
void cleanupOldFrames(uint32_t current_time) {
    // Keep only ~50ms of data (2-3 grid cycles)
    uint32_t keep_duration_cycles = cpu_freq_hz / 20;  // 50ms
    
    while (frame_read_idx != frame_write_idx) {
        uint32_t frame_time = frame_buffer[frame_read_idx].end_timestamp;
        int32_t age = (int32_t)(current_time - frame_time);
        
        if (age < (int32_t)keep_duration_cycles) {
            break;
        }
        
        frame_read_idx = (frame_read_idx + 1) % FRAME_BUFFER_SIZE;
    }
}

void updateTimingParameters(float frequency) {
    float f_clamped = constrain(frequency, 40.0f, 60.0f);
    
    single_cycle_cycles = (uint32_t) lrintf((float)cpu_freq_hz / f_clamped);
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
    
    if (adc_continuous_config(adc_handle, &dig_cfg) != ESP_OK) {
        Serial.println("Failed to configure ADC");
        return false;
    }
    
    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = adc_conv_done_callback,
    };
    
    if (adc_continuous_register_event_callbacks(adc_handle, &cbs, NULL) != ESP_OK) {
        Serial.println("Failed to register callback");
        return false;
    }
    
    if (adc_continuous_start(adc_handle) != ESP_OK) {
        Serial.println("Failed to start ADC");
        return false;
    }
    
    Serial.printf("ADC Continuous mode OK (%d Hz, frame size=%d)\n", ADC_OVERSAMPLE_RATE, CONV_FRAME_SIZE);
    return true;
}

void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println("\nSOGI-PLL ADC Continuous (Frame Timestamping)");
    
    cpu_freq_hz = (uint32_t)ESP.getCpuFreqMHz() * 1000000U;
    inv_cpu_freq = 1.0f / (float)cpu_freq_hz;
    
    // Calculate deterministic sample timing
    // ADC_OVERSAMPLE_RATE is total for both channels
    // Each channel gets ADC_OVERSAMPLE_RATE / 2 samples per second
    uint32_t samples_per_channel_per_sec = ADC_OVERSAMPLE_RATE / 2;
    cycles_per_adc_sample = cpu_freq_hz / samples_per_channel_per_sec;
    
    Serial.printf("Cycles per ADC sample: %lu (%.2f us)\n", 
                  cycles_per_adc_sample, 
                  (float)cycles_per_adc_sample * inv_cpu_freq * 1e6f);
    
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
    Serial.println("Frame-based timestamping with deterministic sample timing");
}

void loop() {
    static uint32_t cleanup_counter = 0;
    
    uint32_t now = get_cycle_count();
    
    // Run aggressive cleanup every 50 samples to avoid overhead
    if (++cleanup_counter >= 50) {
        cleanupOldFrames(now);
        cleanup_counter = 0;
    }
    
    // Check if we have reached or passed the scheduled time for the next sample
    if ((int32_t)(now - last_sample_cycles) >= 0) {
        
        uint32_t latency = now - last_sample_cycles;
        if (current_cycle < 3 && latency > max_jitter_ticks) {
            max_jitter_ticks = latency;
        }

        if (current_cycle < 3 && latency > (ticks_per_sample + (ticks_per_sample >> 1))) {
            overrun_count++;
        }

        bool do_sample = (current_cycle < 3);

        if (do_sample) {
            float v_interp, i_interp;
            
            if (interpolateSampleAtTime(last_sample_cycles, v_interp, i_interp)) {
                v_samp_buf[buf_idx] = v_interp;
                i_samp_buf[buf_idx] = i_interp;
            } else {
                v_samp_buf[buf_idx] = 0;
                i_samp_buf[buf_idx] = 0;
            }
            
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

        uint32_t elapsed_cycle = now - last_cycle_boundary;
        if (elapsed_cycle >= single_cycle_cycles) {
            last_cycle_boundary += single_cycle_cycles;

            int prev_cycle = current_cycle;
            current_cycle = (current_cycle + 1) % 4;
            cycle_start_idx[current_cycle] = buf_idx;

            if (prev_cycle == 2) {
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

                    sogi_v.processWindow(v_samp_buf, BUF_N, s_idx, actual_count, pll.omega, ts, v_dc_offset);
                    pll.update(sogi_v.v_alpha, sogi_v.v_beta, ts);

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
                    
                    // Frame buffer utilization
                    uint32_t buf_frames = (frame_write_idx >= frame_read_idx) ? 
                                          (frame_write_idx - frame_read_idx) : 
                                          (FRAME_BUFFER_SIZE - frame_read_idx + frame_write_idx);
                    float buf_util = 100.0f * buf_frames / FRAME_BUFFER_SIZE;

                    Serial.printf("F:%.4fHz, Mag:%.3f, Vdc:%.1f, Idc:%.1f, Core:%.1fus, Jit:%.1fus, FrameAge:%.0fus, Frames:%lu(%.0f%%), ISR:%lu, Interp:%lu/%lu, Drop:%lu\n",
                                  pll.freq, pll.mag_smooth, v_dc_offset, i_dc_offset, core_us, 
                                  max_jitter_us, (float)max_frame_age_us, buf_frames, buf_util, isr_callback_count, 
                                  interpolation_count, interpolation_count + nearest_sample_count, frames_dropped);
            
                    yield();
                    max_jitter_ticks = 0;
                    interpolation_count = 0;
                    nearest_sample_count = 0;
                    max_frame_age_us = 0;
                }
            } 
        } 
    } 
}
