/*
 * Optimized ESP32 SOGI-PLL - ADC Continuous Mode with Frame Timestamping
 * Focused micro-optimizations in parsing/interpolation & hot paths
 * (Behavior preserved; performance improved)
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
#define SAMPLES_PER_CYCLE 1024
#define V_REF 3.3f

#define V_CHANNEL 0
#define I_CHANNEL 3

#define ADC_OVERSAMPLE_RATE 200000
#define CONV_FRAME_SIZE 4

#define FRAME_BUFFER_SIZE 5500
#define MAX_SEARCH 8

SOGIVisualizer vis;
SOGI sogi_v(SOGI_K);
AdaptivePLL pll(NOMINAL_FREQ, PLL_KP, PLL_KI, 0.1001f);

struct {
    float prev_phase = 0.0f;
    float phase_offset = 0.0f;
    bool initialized = false;
} phase_track;

struct TimestampedFrame {
    uint32_t end_timestamp;
    uint8_t raw_data[CONV_FRAME_SIZE];
    uint16_t data_size;
    uint8_t sample_count; // cached
};

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

static const float RAW_TO_MV = 3300.0f / 4095.0f; // precomputed factor

static inline uint32_t IRAM_ATTR get_cycle_count() {
    uint32_t ccount;
    asm volatile("rsr %0, ccount" : "=a" (ccount));
    return ccount;
}

static inline float IRAM_ATTR adcRawToMillivolts(uint16_t raw_value) {
    // inlined multiply by constant
    return (float)raw_value * RAW_TO_MV;
}

// ISR callback - unchanged logic but keep minimal work
static bool IRAM_ATTR adc_conv_done_callback(adc_continuous_handle_t handle,
                                             const adc_continuous_evt_data_t *edata,
                                             void *user_data) {
    uint32_t end_timestamp = get_cycle_count();

    uint32_t next_write = (frame_write_idx + 1) % FRAME_BUFFER_SIZE;
    if (next_write == frame_read_idx) {
        frames_dropped++;
        return false;
    }

    uint32_t size = edata->size;
    if (size > CONV_FRAME_SIZE) size = CONV_FRAME_SIZE;

    // memcpy raw bytes (fast)
    memcpy((void*)frame_buffer[frame_write_idx].raw_data,
           edata->conv_frame_buffer,
           size);

    frame_buffer[frame_write_idx].end_timestamp = end_timestamp;
    frame_buffer[frame_write_idx].data_size = (uint16_t)size;
    frame_buffer[frame_write_idx].sample_count = 0; // lazy compute

    // publish
    frame_write_idx = next_write;
    isr_callback_count++;

    return false;
}

// Count samples in a frame. Cache the result for future calls.
// optimized to iterate by element rather than byte scattering
static inline uint8_t IRAM_ATTR countFrameSamples(TimestampedFrame* frame) {
    if (frame->sample_count) return frame->sample_count;

    // Work with typed pointer to avoid repeated casting inside loop
    const adc_digi_output_data_t *p = (const adc_digi_output_data_t*)frame->raw_data;
    size_t entries = frame->data_size / sizeof(adc_digi_output_data_t);

    uint8_t v_count = 0, i_count = 0;
    for (size_t n = 0; n < entries; ++n) {
        uint32_t chan = p[n].type1.channel;
        if (chan == V_CHANNEL) ++v_count;
        else if (chan == I_CHANNEL) ++i_count;
    }
    frame->sample_count = (v_count < i_count) ? v_count : i_count;
    return frame->sample_count;
}

// Parse the N-th sample (occurrence) of each channel from frame.
// Optimized by single-pass using typed pointer and bounds from data_size
static inline bool IRAM_ATTR parseFrameSample(const TimestampedFrame* frame, uint8_t sample_idx, uint16_t &v_raw, uint16_t &i_raw) {
    const adc_digi_output_data_t *p = (const adc_digi_output_data_t*)frame->raw_data;
    size_t entries = frame->data_size / sizeof(adc_digi_output_data_t);

    uint8_t v_count = 0, i_count = 0;
    bool v_found = false, i_found = false;

    // Single pass, break early when both found
    for (size_t n = 0; n < entries; ++n) {
        uint32_t chan = p[n].type1.channel;
        if (chan == V_CHANNEL) {
            if (v_count == sample_idx) {
                v_raw = p[n].type1.data;
                v_found = true;
                if (i_found) return true;
            }
            ++v_count;
        } else if (chan == I_CHANNEL) {
            if (i_count == sample_idx) {
                i_raw = p[n].type1.data;
                i_found = true;
                if (v_found) return true;
            }
            ++i_count;
        }
    }
    return (v_found && i_found);
}

uint32_t cpu_freq_hz = 0;
float inv_cpu_freq = 0.0f;
uint32_t cycles_per_adc_sample = 0;
uint32_t single_cycle_cycles = 0;
uint32_t ticks_per_sample = 0;

//uint32_t base_ticks_per_sample = 0;
//uint32_t ticks_remainder = 0;
//uint32_t bresenham_acc = 0;

static inline uint32_t IRAM_ATTR getFrameSampleTimestamp(uint32_t frame_end_time, uint8_t sample_idx, uint8_t sample_count) {
    uint32_t samples_from_end = sample_count - 1 - sample_idx;
    return frame_end_time - (samples_from_end * cycles_per_adc_sample);
}

// Search recent frames backwards and interpolate.
// Micro-optimizations: copy volatile indices locally, avoid redundant work.
bool interpolateSampleAtTime(uint32_t target_time, float &v_out, float &i_out) {
    uint32_t read_idx = frame_read_idx;   // local copy (volatile -> local)
    uint32_t write_idx = frame_write_idx; // local copy

    if (read_idx == write_idx) return false;

    // compute available frames
    uint32_t available = (write_idx >= read_idx) ? (write_idx - read_idx) : (FRAME_BUFFER_SIZE - read_idx + write_idx);
    uint32_t max_search = available;
    if (max_search > MAX_SEARCH) max_search = MAX_SEARCH;

    // start from newest
    uint32_t frame_idx = (write_idx + FRAME_BUFFER_SIZE - 1) % FRAME_BUFFER_SIZE;
    uint32_t searched = 0;

    // get a single "now" measurement later only if we need frame age diagnostics
    while (searched < max_search) {
        TimestampedFrame* frame = (TimestampedFrame*)&frame_buffer[frame_idx];
        if (frame->data_size) {
            uint8_t sample_count = countFrameSamples(frame);
            if (sample_count) {
                uint32_t frame_end = frame->end_timestamp;
                uint32_t frame_start = getFrameSampleTimestamp(frame_end, 0, sample_count);

                // If frame is too old relative to target_time, stop (frames earlier than target won't help)
                int32_t frame_age_from_target = (int32_t)(target_time - frame_end);
                if (frame_age_from_target > (int32_t)(cpu_freq_hz / 10)) { // 100 ms
                    break;
                }

                int32_t delta_start = (int32_t)(target_time - frame_start);
                int32_t delta_end   = (int32_t)(target_time - frame_end);

                if (delta_start >= 0 && delta_end <= 0) {
                    // Target within this frame -> interpolate
                    uint32_t time_into_frame = target_time - frame_start;
                    float sample_idx_f = (float)time_into_frame / (float)cycles_per_adc_sample;
                    uint8_t idx_before = (uint8_t)sample_idx_f;

                    // clamp idx_before safely
                    if (idx_before >= (uint8_t)(sample_count - 1)) {
                        idx_before = sample_count - 1;
                        uint16_t v_raw, i_raw;
                        if (parseFrameSample(frame, idx_before, v_raw, i_raw)) {
                            v_out = adcRawToMillivolts(v_raw);
                            i_out = adcRawToMillivolts(i_raw);
                            return true;
                        }
                    } else {
                        uint8_t idx_after = idx_before + 1;
                        float alpha = sample_idx_f - (float)idx_before;

                        uint16_t v_b, i_b, v_a, i_a;
                        if (parseFrameSample(frame, idx_before, v_b, i_b) &&
                            parseFrameSample(frame, idx_after,  v_a, i_a)) {
                            float v_before = adcRawToMillivolts(v_b);
                            float v_after  = adcRawToMillivolts(v_a);
                            float i_before = adcRawToMillivolts(i_b);
                            float i_after  = adcRawToMillivolts(i_a);
                            v_out = v_before + alpha * (v_after - v_before);
                            i_out = i_before + alpha * (i_after - i_before);
                            return true;
                        }
                    }
                }

                // If target is before this frame, give nearest-first-sample fallback
                if (delta_end <= 0) {
                    uint16_t v_raw, i_raw;
                    if (parseFrameSample(frame, 0, v_raw, i_raw)) {
                        v_out = adcRawToMillivolts(v_raw);
                        i_out = adcRawToMillivolts(i_raw);
                        return true;
                    }
                }
                // else: target is after this frame -> continue searching newer frames (we are going backwards)
            }
        }

        // move to previous (older) frame
        frame_idx = (frame_idx + FRAME_BUFFER_SIZE - 1) % FRAME_BUFFER_SIZE;
        ++searched;
        if (frame_idx == read_idx) break;
    }

    // final fallback: use newest frame's last sample
    uint32_t last_idx = (write_idx + FRAME_BUFFER_SIZE - 1) % FRAME_BUFFER_SIZE;
    TimestampedFrame* last_frame = (TimestampedFrame*)&frame_buffer[last_idx];
    if (last_frame->data_size) {
        uint8_t last_count = countFrameSamples(last_frame);
        if (last_count) {
            uint16_t v_raw, i_raw;
            if (parseFrameSample(last_frame, last_count - 1, v_raw, i_raw)) {
                v_out = adcRawToMillivolts(v_raw);
                i_out = adcRawToMillivolts(i_raw);
                return true;
            }
        }
    }

    return false;
}

// Remove frames older than ~50ms
void cleanupOldFrames(uint32_t current_time) {
    uint32_t keep_duration_cycles = cpu_freq_hz / 20; // 50ms
    while (frame_read_idx != frame_write_idx) {
        uint32_t frame_time = frame_buffer[frame_read_idx].end_timestamp;
        int32_t age = (int32_t)(current_time - frame_time);
        if (age < (int32_t)keep_duration_cycles) break;
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

    adc_continuous_evt_cbs_t cbs = { .on_conv_done = adc_conv_done_callback };
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

/* ------------------- rest of your global state & buffers unchanged ------------------- */
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
uint32_t max_frame_age_us = 0;

float v_dc_offset = 1650.0f;
float i_dc_offset = 1650.0f;
uint32_t last_sample_cycles = 0;
uint32_t last_cycle_boundary = 0;

void setup() {
    Serial.begin(115200);
    delay(100);
    Serial.println("\nOptimized SOGI-PLL ADC Continuous (Frame Timestamping)");

    cpu_freq_hz = (uint32_t)ESP.getCpuFreqMHz() * 1000000U;
    inv_cpu_freq = 1.0f / (float)cpu_freq_hz;

    uint32_t samples_per_channel_per_sec = ADC_OVERSAMPLE_RATE / 2;
    cycles_per_adc_sample = cpu_freq_hz / samples_per_channel_per_sec;

    Serial.printf("Cycles per ADC sample: %lu (%.2f us)\n",
                  cycles_per_adc_sample,
                  (float)cycles_per_adc_sample * inv_cpu_freq * 1e6f);

    updateTimingParameters(NOMINAL_FREQ);

    uint32_t start_c = get_cycle_count();
    last_sample_cycles = start_c;
    last_cycle_boundary = start_c;

    for (int i=0;i<4;i++) cycle_start_idx[i]=0;
    current_cycle = 0;
    buf_idx = 0;

    vis.begin();

    if (!initADCContinuous()) {
        Serial.println("ADC init failed!");
        while (1) delay(100);
    }

    gpio_set_drive_capability((gpio_num_t)18, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability((gpio_num_t)23, GPIO_DRIVE_CAP_3);

    Serial.printf("Ready. CPU: %d MHz\n", ESP.getCpuFreqMHz());
}

void loop() {
    static uint32_t cleanup_counter = 0;
    uint32_t now = get_cycle_count();

    if (++cleanup_counter >= 50) {
        cleanupOldFrames(now);
        cleanup_counter = 0;
    }

    // Use a single compare and only do the heavy work when scheduling indicates
    if ((int32_t)(now - last_sample_cycles) >= 0) {
        uint32_t latency = now - last_sample_cycles;
        if (current_cycle < 3 && latency > max_jitter_ticks) max_jitter_ticks = latency;

        if (current_cycle < 3 && latency > (ticks_per_sample + (ticks_per_sample >> 1))) overrun_count++;

        if (current_cycle < 3) {
            float v_interp = 0.0f, i_interp = 0.0f;
            if (interpolateSampleAtTime(last_sample_cycles, v_interp, i_interp)) {
                v_samp_buf[buf_idx] = v_interp;
                i_samp_buf[buf_idx] = i_interp;
                interpolation_count++;
            } else {
                v_samp_buf[buf_idx] = 0.0f;
                i_samp_buf[buf_idx] = 0.0f;
                nearest_sample_count++;
            }

            buf_idx = (buf_idx + 1) % BUF_N;
        }

        last_sample_cycles += ticks_per_sample;
        sample_slot_count++;

        // Bresenham style step adjust
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

            if (prev_cycle == 2) { // process the window
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
                        int id = (s_idx + i) % BUF_N;
                        v_sum += v_samp_buf[id];
                        i_sum += i_samp_buf[id];
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

                    uint32_t buf_frames = (frame_write_idx >= frame_read_idx) ? 
                                          (frame_write_idx - frame_read_idx) : 
                                          (FRAME_BUFFER_SIZE - frame_read_idx + frame_write_idx);
                    float buf_util = 100.0f * buf_frames / FRAME_BUFFER_SIZE;

                    Serial.printf("F:%.4fHz, Mag:%.3f, Vdc:%.1f, Idc:%.1f, Core:%.1fus, Jit:%.1fus, Frames:%lu(%.0f%%), ISR:%lu, Interp:%lu/%lu, Drop:%lu\n",
                                  pll.freq, pll.mag_smooth, v_dc_offset, i_dc_offset, core_us,
                                  max_jitter_us, buf_frames, buf_util, isr_callback_count,
                                  interpolation_count, interpolation_count + nearest_sample_count, frames_dropped);

                    yield();

                    // reset per-cycle counters
                    max_jitter_ticks = 0;
                    interpolation_count = 0;
                    nearest_sample_count = 0;
                    max_frame_age_us = 0;
                }
            }
        }
    }
}
